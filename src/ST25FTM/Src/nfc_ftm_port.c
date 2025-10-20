/* ST25 FTM platform interface binding using ST25DV driver */
#include "lib/ST25FTM/Inc/st25ftm_interface.h"
#include "lib/ST25FTM/Inc/st25ftm.h"
#include "../Inc/st25ftm_config_hal.h"
#include "lib/st25dvxxkc/st25dvxxkc.h"
#include "st25ftm_packets.h"
#include "uf2.h"
#include <string.h>

/* Weak getter for platform-provided ST25DV handle; platform may override */
__attribute__((weak)) void* ST25FTM_GetSt25dvHandle(void) {
    return NULL;
}

/* Forward declarations for ready sequence functions */
// ST25FTM_Error_t ST25FTM_SetReadySequence(void);
// ST25FTM_Error_t ST25FTM_ClearReadySequence(void);

/* ST25DV driver object (provided by platform, already initialized) */
static ST25DVxxKC_Object_t *g_st25dv = NULL;
/* I2C wrappers for ST25DV driver (expects 8-bit I2C address) */
/* Bus layer is provided by platform with the ST driver; no local wrappers needed */

static ST25FTM_MessageOwner_t mailboxStatus = ST25FTM_MESSAGE_EMPTY;
static bool g_during_transfer = false;

ST25FTM_MessageOwner_t ST25FTM_GetMessageOwner(void) {
    //This may help reduce the number of I2C reads
    // if(mailboxStatus==ST25FTM_MESSAGE_ME) {
    //     return ST25FTM_MESSAGE_ME;
    // }

    ST25DVxxKC_MB_CTRL_DYN_STATUS_t ctrl;
    if (!g_st25dv)
        return ST25FTM_MESSAGE_OWNER_ERROR;
    if (ST25DVxxKC_ReadMBCtrl_Dyn(g_st25dv, &ctrl) != NFCTAG_OK)
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
    ret = ST25DVxxKC_ReadMBCtrl_Dyn(g_st25dv, &dummy );
    if( ret != NFCTAG_OK )
    {
        return ST25FTM_MSG_ERROR;
    }

  /* Read length of message */
  ret = ST25DVxxKC_ReadMBLength_Dyn(g_st25dv,  (uint8_t *)&mblength );
  if( ret != NFCTAG_OK )
  {
    return ST25FTM_MSG_ERROR;
  }
  *msg_len = mblength + 1;
  
  /* Read all data in Mailbox */
  ret = ST25DVxxKC_ReadMailboxData( g_st25dv, msg, 0, *msg_len );
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
    ret = ST25DVxxKC_ReadMBCtrl_Dyn(g_st25dv, &data );
    if( ret != NFCTAG_OK )
    {
      delay(20);
      return ST25FTM_MSG_ERROR;
    }
    
    /* If available, write data */
    if( (data.HostPutMsg == 0) && (data.RfPutMsg == 0) )
    {
        ret = ST25DVxxKC_ReadMailboxData(g_st25dv, msg,0, msg_len );
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
    /* Use already-initialized ST25DV object from platform */
    g_st25dv = (ST25DVxxKC_Object_t *)initHandle;
    if (!g_st25dv)
        return ST25FTM_ERROR;

    /* Ensure I2C security session is open for system register writes */
    {
        ST25DVxxKC_I2CSSO_STATUS_E session = ST25DVXXKC_SESSION_CLOSED;
        if (ST25DVxxKC_ReadI2CSecuritySession_Dyn(g_st25dv, &session) != NFCTAG_OK) {
            return ST25FTM_ERROR;
        }
        if (session == ST25DVXXKC_SESSION_CLOSED) {
            ST25DVxxKC_PASSWD_t pwd;
            pwd.MsbPasswd = 0x00000000u; /* default factory I2C password */
            pwd.LsbPasswd = 0x00000000u;
            if (ST25DVxxKC_PresentI2CPassword(g_st25dv, pwd) != NFCTAG_OK) {
                return ST25FTM_ERROR;
            }
            /* Re-check session */
            if (ST25DVxxKC_ReadI2CSecuritySession_Dyn(g_st25dv, &session) != NFCTAG_OK ||
                session != ST25DVXXKC_SESSION_OPEN) {
                return ST25FTM_ERROR;
            }
        }
    }

    /* Initialize and enable Mailbox */
    ST25DVxxKC_EN_STATUS_E mb_mode;
    if (ST25DVxxKC_ReadMBMode(g_st25dv, &mb_mode) != NFCTAG_OK) {
        return ST25FTM_ERROR;
    }

    if (mb_mode == ST25DVXXKC_DISABLE) {
        if (ST25DVxxKC_WriteMBMode(g_st25dv, ST25DVXXKC_ENABLE) != NFCTAG_OK) {
            return ST25FTM_ERROR;
        }
        if (ST25DVxxKC_SetMBEN_Dyn(g_st25dv) != NFCTAG_OK) {
            return ST25FTM_ERROR;
        }
    } else {
        /* If already enabled, reset dynamic flag and enable again to clear state */
        if (ST25DVxxKC_ResetMBEN_Dyn(g_st25dv) != NFCTAG_OK) {
            return ST25FTM_ERROR;
        }
        if (ST25DVxxKC_SetMBEN_Dyn(g_st25dv) != NFCTAG_OK) {
            return ST25FTM_ERROR;
        }
    }

    /* Disable Mailbox watchdog */
    if (ST25DVxxKC_WriteMBWDG(g_st25dv, 0) != NFCTAG_OK) {
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
#define CHECK_INTERVAL 5  // Check actual field status every 10 calls
void ST25FTM_UpdateFieldStatus(ST25FTM_Field_State_t * rfField) {

    call_count++;
    if (call_count >= CHECK_INTERVAL) {
        call_count = 0;
        ST25DVxxKC_FIELD_STATUS_E field;
        if (g_st25dv && ST25DVxxKC_GetRFField_Dyn(g_st25dv, &field) == NFCTAG_OK) {
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
   
        return 0;
    
}

uint32_t ST25FTM_GetMsTick(void) {
    /* Use bootloader timer tick (ms) from TIMER_STEP scheduler */
    return timerHigh;
}

void ST25FTM_PlatformDelay(uint32_t ms) {
    delay(ms);
}

// /* Set ready sequence in ST25 user memory to signal device is ready for FTM */
// ST25FTM_Error_t ST25FTM_SetReadySequence(void) {
//     const uint8_t *ready_seq = (const uint8_t *)ST25_READY_SEQUENCE;
//     if (St25Dv_Drv.WriteData(&g_st25dv, ready_seq, (uint16_t)ST25_USER_MEM_READY_START, (uint16_t)ST25_READY_SEQUENCE_LEN) != NFCTAG_OK)
//         return ST25FTM_ERROR;
//     return ST25FTM_OK;
// }

// /* Clear ready sequence from ST25 user memory */
// ST25FTM_Error_t ST25FTM_ClearReadySequence(void) {
//     uint8_t zeros[ST25_READY_SEQUENCE_LEN] = {0};
//     if (St25Dv_Drv.WriteData(&g_st25dv, zeros, (uint16_t)ST25_USER_MEM_READY_START, (uint16_t)ST25_READY_SEQUENCE_LEN) != NFCTAG_OK)
//         return ST25FTM_ERROR;
//     return ST25FTM_OK;
// }


