/* ST25 FTM platform interface binding using ST25DV driver */
#include "lib/ST25FTM/Inc/st25ftm_interface.h"
#include "lib/ST25FTM/Inc/st25ftm.h"
#include "../Inc/st25ftm_config_hal.h"
#include "lib/st25dvxxkc/st25dvxxkc.h"
#include "st25ftm_packets.h"
#include "uf2.h"
#include <string.h>
#include "i2c.h"

/* Bindings for ST25DV driver
Basically we need to provide i2c functions to the ST25DV driver.
*/
static int32_t st25dv_bus_init(void) { return NFCTAG_OK; }
static int32_t st25dv_bus_deinit(void) { return NFCTAG_OK; }
static int32_t st25dv_bus_write(uint16_t DevAddr, uint16_t MemAddr, const uint8_t *pData,
                                uint16_t len) {
    uint8_t i2c_addr_7bit = (uint8_t)(DevAddr >> 1);
    uint8_t hdr[2] = {(uint8_t)(MemAddr >> 8), (uint8_t)(MemAddr & 0xFF)};
    if (len == 0) {
        return (i2c_write(i2c_addr_7bit, hdr, 2) == I2C_RESULT_SUCCESS) ? NFCTAG_OK : NFCTAG_ERROR;
    }
    if (len > 256)
        len = 256;
    uint8_t tmp[2 + 256];
    tmp[0] = hdr[0];
    tmp[1] = hdr[1];
    for (uint16_t i = 0; i < len; i++)
        tmp[2 + i] = pData[i];
    return (i2c_write(i2c_addr_7bit, tmp, (uint32_t)(2 + len)) == I2C_RESULT_SUCCESS)
               ? NFCTAG_OK
               : NFCTAG_ERROR;
}
static int32_t st25dv_bus_read(uint16_t DevAddr, uint16_t MemAddr, uint8_t *pData, uint16_t len) {
    uint8_t i2c_addr_7bit = (uint8_t)(DevAddr >> 1);
    uint8_t addr[2] = {(uint8_t)(MemAddr >> 8), (uint8_t)(MemAddr & 0xFF)};
    if (i2c_write(i2c_addr_7bit, addr, 2) != I2C_RESULT_SUCCESS)
        return NFCTAG_ERROR;
    return (i2c_read(i2c_addr_7bit, pData, len) == I2C_RESULT_SUCCESS) ? NFCTAG_OK : NFCTAG_ERROR;
}
static int32_t st25dv_bus_isready(uint16_t DevAddr, const uint32_t Trials) {
    uint8_t i2c_addr_7bit = (uint8_t)(DevAddr >> 1);
    for (uint32_t i = 0; i < Trials; i++) {
        if (i2c_write(i2c_addr_7bit, NULL, 0) == I2C_RESULT_SUCCESS)
            return NFCTAG_OK;
        delay(1);
    }
    return NFCTAG_ERROR;
}
static int32_t st25dv_bus_gettick(void) { return (int32_t)timerHigh; }



/* ST25DV driver object */
static ST25DVxxKC_Object_t g_st25dv;
ST25FTM_Error_t ST25FTM_InitSt25dvObject(void) {
    if (g_st25dv.IsInitialized == 0U) {
        ST25DVxxKC_IO_t io = {
            .Init = st25dv_bus_init,
            .DeInit = st25dv_bus_deinit,
            .IsReady = st25dv_bus_isready,
            .Write = st25dv_bus_write,
            .Read = st25dv_bus_read,
            .GetTick = st25dv_bus_gettick,
            .DeviceAddress = ST25DVXXKC_ADDR_DATA_I2C,
        };
        if (ST25DVxxKC_RegisterBusIO(&g_st25dv, &io) != NFCTAG_OK)
            return ST25FTM_ERROR;
    }

    if (St25Dvxxkc_Drv.Init(&g_st25dv) != NFCTAG_OK)
        return ST25FTM_ERROR;

    return ST25FTM_OK;
}

/* ST25FTM Mailbox functions
This connects our st25 driver to the FTM protocol.
Mostly these are directly copied from the ST25FTM examples.
*/


static ST25FTM_MessageOwner_t mailboxStatus = ST25FTM_MESSAGE_EMPTY;
static bool g_during_transfer = false;

ST25FTM_MessageOwner_t ST25FTM_GetMessageOwner(void) {
    //This may help reduce the number of I2C reads
    // if(mailboxStatus==ST25FTM_MESSAGE_ME) {
    //     return ST25FTM_MESSAGE_ME;
    // }

    ST25DVxxKC_MB_CTRL_DYN_STATUS_t ctrl;
    if (ST25DVxxKC_ReadMBCtrl_Dyn(&g_st25dv, &ctrl) != NFCTAG_OK)
        return ST25FTM_MESSAGE_OWNER_ERROR;
    /* RFPutMsg set => message from RF peer for host (us) */
    if (ctrl.RfPutMsg)
        return ST25FTM_MESSAGE_PEER;
    if (ctrl.HostPutMsg)
        return ST25FTM_MESSAGE_ME;
    return ST25FTM_MESSAGE_EMPTY;

}

ST25FTM_MessageStatus_t ST25FTM_ReadMessage(uint8_t *msg, uint32_t* msg_len) {
    g_during_transfer = true;
    int ret = NFCTAG_OK;
    uint16_t mblength = 0;
    ST25DVxxKC_MB_CTRL_DYN_STATUS_t   dummy = {0};
  
      /* Artificially Read Mailbox CTRL reg (See ErrataSheet ES0617: RF_PUT_MSG bit wrongly cleared after I²C reading MB_LEN_Dyn if message is 256 bytes long) */
    ret = ST25DVxxKC_ReadMBCtrl_Dyn(&g_st25dv, &dummy );
    if( ret != NFCTAG_OK )
    {
        return ST25FTM_MSG_ERROR;
    }

  /* Read length of message */
  ret = ST25DVxxKC_ReadMBLength_Dyn(&g_st25dv,  (uint8_t *)&mblength );
  if( ret != NFCTAG_OK )
  {
    return ST25FTM_MSG_ERROR;
  }
  *msg_len = mblength + 1;
  
  /* Read all data in Mailbox */
  ret = ST25DVxxKC_ReadMailboxData( &g_st25dv, msg, 0, *msg_len );
  if(ret == NFCTAG_OK)
  {
    mailboxStatus = ST25FTM_MESSAGE_EMPTY;

    /* Trick to automatically detect the max frame length of the reader
       To have this auto detection working, the reader must send a long command
       before receiveing a long response.
    */
    ST25FTM_Ctrl_Byte_t ctrl;
    ctrl.byte = msg[0];
    if((!ST25FTM_CTRL_HAS_PKT_LEN(ctrl)) && !(msg[0] & ST25FTM_STATUS_BYTE))
    {
      ST25FTM_SetRxFrameMaxLength(*msg_len);
      ST25FTM_SetTxFrameMaxLength(*msg_len);
    }

    return ST25FTM_MSG_OK;
  }
  return ST25FTM_MSG_ERROR;
}

ST25FTM_MessageStatus_t ST25FTM_WriteMessage(uint8_t* msg, uint32_t msg_len) {
    int ret = NFCTAG_OK;
    ST25DVxxKC_MB_CTRL_DYN_STATUS_t   data = {0};
    
    /* Check if Mailbox is available */
    ret = ST25DVxxKC_ReadMBCtrl_Dyn(&g_st25dv, &data );
    if( ret != NFCTAG_OK )
    {
      delay(20);
      return ST25FTM_MSG_ERROR;
    }
    
    /* If available, write data */
    if( (data.HostPutMsg == 0) && (data.RfPutMsg == 0) )
    {
        ret = ST25DVxxKC_ReadMailboxData(&g_st25dv, msg,0, msg_len );
    } 
    else 
    {
      delay(20);
      return ST25FTM_MSG_BUSY;
    }
    
    if(ret == NFCTAG_OK)
    {
      mailboxStatus = ST25FTM_MESSAGE_ME;
      return ST25FTM_MSG_OK;
    } else {
      return ST25FTM_MSG_ERROR;
    }
}

ST25FTM_Error_t ST25FTM_DeviceInit(void * initHandle) {

    /* Initialize ST25DV object */
    if (ST25FTM_InitSt25dvObject() != ST25FTM_OK)
        return ST25FTM_ERROR;

    /* Ensure I2C security session is open for system register writes */
    {
        ST25DVxxKC_I2CSSO_STATUS_E session = ST25DVXXKC_SESSION_CLOSED;
        if (ST25DVxxKC_ReadI2CSecuritySession_Dyn(&g_st25dv, &session) != NFCTAG_OK) {
            return ST25FTM_ERROR;
        }
        if (session == ST25DVXXKC_SESSION_CLOSED) {
            ST25DVxxKC_PASSWD_t pwd;
            pwd.MsbPasswd = 0x00000000u; /* default factory I2C password */
            pwd.LsbPasswd = 0x00000000u;
            if (ST25DVxxKC_PresentI2CPassword(&g_st25dv, pwd) != NFCTAG_OK) {
                return ST25FTM_ERROR;
            }
            /* Re-check session */
            if (ST25DVxxKC_ReadI2CSecuritySession_Dyn(&g_st25dv, &session) != NFCTAG_OK ||
                session != ST25DVXXKC_SESSION_OPEN) {
                return ST25FTM_ERROR;
            }
        }
    }

    /* Initialize and enable Mailbox */
    ST25DVxxKC_EN_STATUS_E mb_mode;
    if (ST25DVxxKC_ReadMBMode(&g_st25dv, &mb_mode) != NFCTAG_OK) {
        return ST25FTM_ERROR;
    }

    if (mb_mode == ST25DVXXKC_DISABLE) {
        if (ST25DVxxKC_WriteMBMode(&g_st25dv, ST25DVXXKC_ENABLE) != NFCTAG_OK) {
            return ST25FTM_ERROR;
        }
        if (ST25DVxxKC_SetMBEN_Dyn(&g_st25dv) != NFCTAG_OK) {
            return ST25FTM_ERROR;
        }
    } else {
        /* If already enabled, reset dynamic flag and enable again to clear state */
        if (ST25DVxxKC_ResetMBEN_Dyn(&g_st25dv) != NFCTAG_OK) {
            return ST25FTM_ERROR;
        }
        if (ST25DVxxKC_SetMBEN_Dyn(&g_st25dv) != NFCTAG_OK) {
            return ST25FTM_ERROR;
        }
    }

    /* Disable Mailbox watchdog */
    if (ST25DVxxKC_WriteMBWDG(&g_st25dv, 0) != NFCTAG_OK) {
        return ST25FTM_ERROR;
    }

    return ST25FTM_OK;
}

ST25FTM_Error_t ST25FTM_DeviceRelease(void * initHandle) {
    (void)initHandle;
    
    return ST25FTM_OK;
}

static uint32_t call_count = 0;
static ST25FTM_Field_State_t cached_field = ST25FTM_FIELD_OFF;
#define CHECK_INTERVAL 5  // Check actual field status every 5 calls
void ST25FTM_UpdateFieldStatus(ST25FTM_Field_State_t * rfField) {

    call_count++;
    if (call_count >= CHECK_INTERVAL) {
        call_count = 0;
        ST25DVxxKC_FIELD_STATUS_E field;
        if (ST25DVxxKC_GetRFField_Dyn(&g_st25dv, &field) == NFCTAG_OK) {
            if (field == ST25DVXXKC_FIELD_ON) {
                cached_field = ST25FTM_FIELD_ON;
            } else {
                cached_field = ST25FTM_FIELD_OFF;
            }
        }
    }
    *rfField = cached_field;
}

ST25FTM_Error_t ST25FTM_CRC_Initialize(void) {
    return ST25FTM_OK;
}

ST25FTM_Error_t ST25FTM_CRC_Release(void) {
    return ST25FTM_OK;
}

ST25FTM_Crc_t ST25FTM_GetCrc(uint8_t *data, uint32_t length, ST25FTM_crc_control_t control) {
    //We don't need CRC for our application
        return 0;
    
}

uint32_t ST25FTM_GetMsTick(void) {
    /* Use bootloader timer tick (ms) from TIMER_STEP scheduler */
    return timerHigh;
}

void ST25FTM_PlatformDelay(uint32_t ms) {
    delay(ms);
}
