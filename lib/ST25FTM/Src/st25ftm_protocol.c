/******************************************************************************
  * @attention
  *
  * COPYRIGHT 2018 STMicroelectronics, all rights reserved
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
******************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "st25ftm.h"
#include "st25ftm_interface.h"
#include "st25ftm_packets.h"
#include "st25ftm_internal.h"

/// @brief ST25FTM FW Version
#define ST25FTM_FW_VERSION              0x030102
/// @brief ST25FTM FW Version Flavor
#define ST25FTM_FW_FLAVOR               0x00


#if (ST25FTM_ENABLE_LOG != 0)
/*! String version of the FTM Reception state machine for display */
const char * ST25FTM_RxState_Str[NUMBER_OF_ST25FTM_RX_STATE] = {

  "ST25FTM_RX_IDLE",
  "ST25FTM_RX_INIT_RECEPTION",
  "ST25FTM_RX_READ_PKT",
  "ST25FTM_RX_WRITE_ACK",
  "ST25FTM_RX_WRITE_NACK",
  "ST25FTM_RX_WRITE_ERR",
  "ST25FTM_RX_WAIT_ACK_READ",
  "ST25FTM_RX_ACK_READ",
  "ST25FTM_RX_DONE",
  "ST25FTM_RX_ERROR"
};

/*! String version of the FTM main state machine for display */
const char * ST25FTM_State_Str[NUMBER_OF_ST25FTM_STATE] = {
  "ST25FTM_IDLE",
  "ST25FTM_TX",
  "ST25FTM_RX"
};

/*! String version of the FTM transmission state machine for display */
const char * ST25FTM_TxState_Str[NUMBER_OF_ST25FTM_TX_STATE] = {
  "ST25FTM_TX_IDLE",
  "ST25FTM_TX_INIT_TRANSMISSION",
  "ST25FTM_TX_WRITE_SEGMENT",
  "ST25FTM_TX_WRITE_PKT",
  "ST25FTM_TX_WAIT_READ",
  "ST25FTM_TX_READ_ACK",
  "ST25FTM_TX_DONE",
  "ST25FTM_TX_ERROR"
};

const char * ST25FTM_AckCtrl_Str[NUMBER_OF_ST25FTM_PACKET_ACKS] = {

  "ST25FTM_NO_ACK_PACKET",
  "ST25FTM_SEGMENT_START",
  "ST25FTM_SEGMENT_END",
  "ST25FTM_ACK_SINGLE_PKT"
};

const char * ST25FTM_Position_Str[NUMBER_OF_ST25FTM_PACKET_POSITIONS] = {

  "ST25FTM_SINGLE_PACKET",
  "ST25FTM_FIRST_PACKET",
  "ST25FTM_MIDDLE_PACKET",
  "ST25FTM_LAST_PACKET"
};
#endif

/*! ST25FTM global variables */
ST25FTM_InternalState_t gFtmState;

static ST25FTM_Field_State_t gLastRfField = ST25FTM_FIELD_OFF;


static void ST25FTM_SetState(ST25FTM_State_t state)
{
    if (gFtmState.state != state) {
        gFtmState.state = state;
        ST25FTM_LOG("State = %s\n",ST25FTM_State_Str[gFtmState.state]);
    }
}


/*! Initialize the FTM state machines */
static void ST25FTM_StateInit(void)
{
  uint32_t i;

  gLastRfField = ST25FTM_FIELD_OFF;

  /*! Initiates the ST25FTM state machines */
  gFtmState.state = ST25FTM_IDLE;
  gFtmState.runnerPolicy = ST25FTM_RUNNER_LOOP_INTERNALLY;
  gFtmState.callIdleEventCb = ST25FTM_FALSE;     // Do not call the callback yet

  gFtmState.rfField = ST25FTM_FIELD_OFF;
  gFtmState.retryLength = 0;
  gFtmState.nbOfConsecutiveRegisterError = 0;
  gFtmState.lastTick = ST25FTM_TICK();
  for (i=0 ; i<NUMBER_OF_ST25FTM_EVENTS ; i++) {
      gFtmState.events[i].cb = NULL;
      gFtmState.events[i].userData = NULL;
  }

  ST25FTM_TxStateInit();
  ST25FTM_RxStateInit();
}

/**
  * @brief  This method returns the ST25FTM revision
  * @retval version: 0xXYZF (8bits for each decimal, F for Flavor)
  */
uint32_t ST25FTM_GetVersion(void)
{
  return (ST25FTM_FW_VERSION << 8) | ST25FTM_FW_FLAVOR;
}

/*! OBSOLETE !! Initialize the FTM state machines and the NFC device */
void ST25FTM_Init(void)
{
    (void)ST25FTM_Initialize(NULL);
}

/*! Initialize the FTM state machines and the NFC device */
ST25FTM_Error_t ST25FTM_Initialize(void * initHandle)
{
  ST25FTM_Error_t   ret = ST25FTM_OK;

  /*! Initiates the ST25FTM state machines */
  ST25FTM_StateInit();
  
  /*! Initiates the NFC device */
  ret |= ST25FTM_DeviceInit(initHandle);

  /*! CRC is NOT initiated here as it is later on in component */
  
  ST25FTM_UpdateFieldStatus(&gFtmState.rfField);
  
  return ret;
}

/*! Releases the NFC device and CRC then reset the FTM state machine */
ST25FTM_Error_t ST25FTM_Release(void * initHandle)
{
  ST25FTM_Error_t   ret = ST25FTM_OK;

  /*! Releases the CRC */
  ret |= ST25FTM_CRC_Release();

  /*! Releases the NFC device */
  ret |= ST25FTM_DeviceRelease(initHandle);

  /*! Reset the ST25FTM state machines */
  ST25FTM_StateInit();
  
  return ret;
}

/*! Register the maximum frame length while transmitting
 *  @param len Maximum frame length in bytes
 */
void ST25FTM_SetTxFrameMaxLength(uint32_t len)
{
  gFtmState.tx.frameMaxLength = len;
}

/*! Get the maximum frame length while transmitting
 * @return The maxmum number of bytes per transmitted frame
 */
uint32_t ST25FTM_GetTxFrameMaxLength(void)
{
  return gFtmState.tx.frameMaxLength;
}

/*! Register the maximum frame length while receiving
 *  @param len Maximum frame length in bytes
 */
void ST25FTM_SetRxFrameMaxLength(uint32_t len)
{
  gFtmState.rx.frameMaxLength = len;
}

/*! Get the maximum frame length while receiving
 * @return The maxmum number of bytes per received frame
 */
uint32_t ST25FTM_GetRxFrameMaxLength(void)
{
  return gFtmState.rx.frameMaxLength;
}

/*! Register the duration of delay trimming while transmitting
 *  @param len delay in ms
 */
void ST25FTM_SetTxTrim(uint32_t delay)
{
  gFtmState.tx.trimmingDelay = delay;
}

/*! Get the duration of delay trimming while transmitting
 * @return The duration of delay trimming
 */
uint32_t ST25FTM_GetTxTrim(void)
{
  return gFtmState.tx.trimmingDelay;
}

/*! Register the runner policy
 *  @param policy: Runner policy to be applied: 
  *             ST25FTM_RUNNER_LOOP_INTERNALLY: loop internally until no more state transition (default policy)
  *             ST25FTM_RUNNER_LOOP_WITHIN_APPLICATION: leave application to handle loop for transitions
 */
ST25FTM_Error_t ST25FTM_SetRunnerPolicy(uint8_t policy)
{
  ST25FTM_Error_t  ret = ST25FTM_ERROR;
  if (policy <= ST25FTM_RUNNER_LOOP_WITHIN_APPLICATION) {
    gFtmState.runnerPolicy = policy;
    ret = ST25FTM_OK;
  }
  return ret;
}


/*! OBSOLETE !! Initialize a transmission
  * @param  data Pointer to the data buffer to be transmitted
  * @param length Number of bytes to be transmitted
  * @param ack Enables handchecks during the transfer
  * @param data_cb Optional callback function, called to request data to send (to be set to NULL if not used)
  */
void ST25FTM_SendCommand(uint8_t* data, uint32_t length, ST25FTM_Send_Ack_t ack, ftm_data_cb data_cb)
{
  (void)ST25FTM_SendStart(data, length, ack);
  gFtmState.tx.getdata_cb = data_cb;         // If Cb==NULL, default function (simple memcpy) will be used
}

/*! Initialize a transmission
  * @param  data Pointer to the data buffer to be transmitted
  * @param length Number of bytes to be transmitted
  * @param ack Enables handchecks during the transfer
  */
ST25FTM_Error_t ST25FTM_SendStart(uint8_t* data, uint32_t length, ST25FTM_Send_Ack_t ack)
{
  ST25FTM_Error_t   ret = ST25FTM_OK;

  ST25FTM_LOG("### ST25FTM_SendStart of %d bytes\n", length);
  gFtmState.tx.cmdPtr = data;
  gFtmState.tx.cmdLen = length;
  gFtmState.tx.sendAck = ack;
  gFtmState.tx.getdata_cb = NULL;

  // Reset Rx as not used
  gFtmState.rx.cmdPtr = NULL;
  ST25FTM_ResetRxState(gFtmState.rx.cmdPtr);

  // Tx mode
  ST25FTM_SetState(ST25FTM_TX);
  ST25FTM_SetTxState(ST25FTM_TX_IDLE);
    
  if (gFtmState.tx.trimmingDelay) {
    ST25FTM_DELAY(gFtmState.tx.trimmingDelay);
  }
  
  return ret;
}


/*! OBSOLETE !!! Initialize a reception
  * @param  data Pointer to the data buffer used for the reception
  * @param length Pointer to a word defining the maximum number of bytes that can be received.
                  This parameter is also used to return the number of bytes actually read
  * @param ack Enables handchecks during the transfer
  * @param data_cb Optional callback function, called to write received datad (to be set to NULL if not used)
  */
void ST25FTM_ReceiveCommand(uint8_t* data, uint32_t *length, ftm_data_cb data_cb)
{
  (void)ST25FTM_ReceiveStart(data, *length, length);
  gFtmState.rx.recvdata_cb = data_cb;         // If Cb==NULL, default function (simple memcpy) will be used
}

/*! Initialize a reception
  * @param  data Pointer to the data buffer used for the reception
  * @param length Pointer to a word defining the maximum number of bytes that can be received.
                  This parameter is also used to return the number of bytes actually read
  * @param ack Enables handchecks during the transfer
  */
ST25FTM_Error_t ST25FTM_ReceiveStart(uint8_t* data, uint32_t maxLength, uint32_t *length)
{
  ST25FTM_Error_t   ret = ST25FTM_OK;

  ST25FTM_LOG("### ST25FTM_ReceiveStart for max %d bytes\n", maxLength);
  gFtmState.rx.cmdPtr = data;
  gFtmState.rx.cmdLen = length;
  gFtmState.rx.maxCmdLen = maxLength;
  gFtmState.rx.recvdata_cb = NULL;

  // Reset Tx as not used
  gFtmState.tx.cmdPtr = NULL;
  gFtmState.tx.cmdLen = 0;
  ST25FTM_ResetTxState(gFtmState.tx.cmdPtr, gFtmState.tx.cmdLen);

  // Rx mode
  ST25FTM_SetState(ST25FTM_RX);
  ST25FTM_SetRxState(ST25FTM_RX_IDLE);
    
  return ret;
}

/*! Registers FTM events callbacks */
ST25FTM_Error_t ST25FTM_RegisterEvent(ST25FTM_Protocol_Event_t event, ftm_events_cb_t event_cb, void *userData)
{
  ST25FTM_Error_t   ret = ST25FTM_ERROR;

  if (event < NUMBER_OF_ST25FTM_EVENTS) {
      if ((event_cb != NULL) && (event == EVENT_FTM_RX_NEW_PKT) && (gFtmState.rx.recvdata_cb != NULL)) {
          // CB has already been defined through ST25FTM_ReceiveCommand()
          ret = ST25FTM_ERROR;
      }
      else if ((event_cb != NULL) && (event == EVENT_FTM_TX_NEW_PKT) && (gFtmState.tx.getdata_cb != NULL)) {
          // CB has already been defined through ST25FTM_SendCommand()
          ret = ST25FTM_ERROR;
      }
      else {
          gFtmState.events[event].userData = userData;
          gFtmState.events[event].cb = event_cb;
          ret = ST25FTM_OK;
      }
  }
  
  return ret;
}

/*! Run the FTM state machine */
uint8_t ST25FTM_Runner(void)
{
  do {
    gFtmState.stateMachineTransition = 0;     // Will be updated by any Rx/Tx StateMachine related functions

    ST25FTM_UpdateFieldStatus(&gFtmState.rfField);
    /* Do nothing if field is off */
    if(gFtmState.rfField == ST25FTM_FIELD_ON) {
      if(gFtmState.state == ST25FTM_TX) {
        /* Check if a field OFF/field ON occured while transmitting */
        if(gLastRfField == ST25FTM_FIELD_OFF) {
          /* field OFF/field ON occured while transmitting: restart at the beg of the segment
             We don't know if last packet has been read or not => restart segment */
          if(gFtmState.tx.state >= ST25FTM_TX_WRITE_SEGMENT) {
            ST25FTM_LOG("FIELD OFF while transmiting, restart segment\r\n");
            ST25FTM_LOG("  segmentRemainingData=%d\r\n",gFtmState.tx.segmentRemainingData);
            ST25FTM_LOG("  pktIndex=%d\r\n",gFtmState.tx.pktIndex);
//            ST25FTM_LOG("  segmentPtr=%X\r\n",gFtmState.tx.segmentPtr);
            ST25FTM_LOG("  segmentIndex=%d\r\n",gFtmState.tx.segmentIndex);
            ST25FTM_LOG("  state=%s\r\n",ST25FTM_TxState_Str[gFtmState.tx.state]);
            ST25FTM_TxResetSegment();
            ST25FTM_LOG("After reset:\r\n");
            ST25FTM_LOG("  segmentRemainingData=%d\r\n",gFtmState.tx.segmentRemainingData);
            ST25FTM_LOG("  pktIndex=%d\r\n",gFtmState.tx.pktIndex);
//            ST25FTM_LOG("  segmentPtr=%X\r\n",gFtmState.tx.segmentPtr);
            ST25FTM_LOG("  segmentIndex=%d\r\n",gFtmState.tx.segmentIndex);
            ST25FTM_LOG("  state=%s\r\n",ST25FTM_TxState_Str[gFtmState.tx.state]);
          }
        }
      
        if(gFtmState.tx.state != gFtmState.tx.lastState) {
          gFtmState.tx.lastState = gFtmState.tx.state;
          gFtmState.lastTick = ST25FTM_TICK();
        }
        if(   (ST25FTM_CompareTime(ST25FTM_TICK(), gFtmState.lastTick) > ST25FTM_WAIT_TIMEOUT_IN_MS)
           && ((gFtmState.tx.state == ST25FTM_TX_WAIT_READ) || (gFtmState.tx.state == ST25FTM_TX_WAIT_ACK))) {
          /* a timeout occured while waiting for the RF to read packet or write a ack
             reset segment transmission */
          ST25FTM_LOG("Timeout while transmitting, restart segment\r\n");
          ST25FTM_TxResetSegment();
        }
        ST25FTM_TxStateMachine();
      }
      else if (gFtmState.state == ST25FTM_RX) {
        if(gFtmState.rx.state != gFtmState.rx.lastState) {
          gFtmState.rx.lastState = gFtmState.rx.state;
          gFtmState.lastTick = ST25FTM_TICK();
        }
        ST25FTM_RxStateMachine();
      }
      else if (gFtmState.state == ST25FTM_IDLE) {
        if (gFtmState.callIdleEventCb) {
            gFtmState.callIdleEventCb = ST25FTM_FALSE;

            // Calling Cb
            ST25FTM_CallEventCb(EVENT_FTM_BACK_TO_IDLE, NULL, NULL, 0);
        }
      }
    }
    gLastRfField = gFtmState.rfField;
  } while ((gFtmState.runnerPolicy == ST25FTM_RUNNER_LOOP_INTERNALLY) && (gFtmState.stateMachineTransition));   // stateMachineTransition updated whenever SetRx/TxState have been called

  return gFtmState.stateMachineTransition;
}

/*! Get the current FTM state.
  * @retval ST25FTM_IDLE State machine is Idle, the reception/transmission is over.
  * @retval ST25FTM_RX Reception is on-going.
  * @retval ST25FTM_TX Transmission is on-going.
 */
ST25FTM_State_t ST25FTM_Status(void)
{
  return gFtmState.state;
}

/*! Detect that a new reception has started.
  * @retval 1 when a new recpetion has started since last call
  * @retval 0 otherwise
*/
uint8_t ST25FTM_IsNewFrame(void)
{
  uint8_t status;
  if(gFtmState.rx.isNewFrame) {
    ST25FTM_LOG("*** Rx New Frame ***\r\n");
    gFtmState.rx.isNewFrame = ST25FTM_FALSE;
    status = ST25FTM_TRUE;
  }
  else {
    status = ST25FTM_FALSE;
  }
  return status;
}

/*! Compute the current transfer progress.
  * @return THe current transfer progress (percentage).
  */
uint32_t ST25FTM_GetTransferProgress(void)
{
  uint32_t progress = 0;
  if(gFtmState.state == ST25FTM_TX) {
    if (gFtmState.tx.cmdLen) {
        progress = ((gFtmState.tx.cmdLen - gFtmState.tx.remainingData - gFtmState.tx.segmentRemainingData) * 100U) / gFtmState.tx.cmdLen;
    }
  }
  else if (gFtmState.state == ST25FTM_RX) {
    if ((gFtmState.rx.cmdLen != NULL) && (*gFtmState.rx.cmdLen)) {
        progress = (gFtmState.rx.totalValidReceivedLength * 100U) / *gFtmState.rx.cmdLen;
    }
  }
  return progress;
}

/*! Get the number of byte received.
    It can be used by the application to process the received data before transfer completion.
  * @return The current number of valid bytes received.
  */
uint32_t ST25FTM_GetAvailableDataLength(void)
{
  return gFtmState.rx.validReceivedLength;
 }

/*! Read received bytes during the transmission, freeing space to continue the reception.
    It can be used by the application to process the received data before transfer completion.
  * @param dst The buffer to copy the received data.
  * @param length Number of bytes to copy.
  * @retval 0 if the data has been copied.
  * @retval 1 otherwise.
  */
uint8_t ST25FTM_ReadBuffer(uint8_t *dst,  uint32_t length)
{
  uint8_t status;

  ST25FTM_LOG("ST25FTM_ReadBuffer length:%d validReceivedLength:%d\n", length, gFtmState.rx.validReceivedLength);
  if(length <= gFtmState.rx.validReceivedLength)
  {
    gFtmState.rx.receivedLength -= length;
    gFtmState.rx.validReceivedLength -= length;
    (void)memcpy(dst,gFtmState.rx.cmdPtr,length);
    (void)memmove(gFtmState.rx.cmdPtr,&gFtmState.rx.cmdPtr[length],gFtmState.rx.receivedLength);
    gFtmState.rx.dataPtr -= length;
    gFtmState.rx.segmentPtr -= length;
    gFtmState.rx.readBufferOffset += length;
    status = ST25FTM_FALSE;
  } else {
    status = ST25FTM_TRUE;
  }
  return status;
}

/*! Get the current offset in the command of the next byte that will be read with ST25FTM_ReadBuffer.
    It can be used by the application to keep track of the incoming data before the transfer completes.
  * @return The offset of the next byte to read.
  */
uint32_t ST25FTM_GetReadBufferOffset(void)
{
  return gFtmState.rx.readBufferOffset;
}

/*! Get the current field state (only relevant for dynamic tag device).
  * @retval 1 if RF field is present.
  * @retval 0 otherwise.
*/
ST25FTM_Field_State_t ST25FTM_GetFieldState(void)
{
  return gFtmState.rfField;
}


/*! Get the total length of the transfer (including protocol metadata).
  * @return The total number of transfered or received bytes.
*/
uint32_t ST25FTM_GetTotalLength(void)
{
  uint32_t totalLg;
  if (gFtmState.state == ST25FTM_TX)        { totalLg = gFtmState.tx.totalDataLength; }
  else if (gFtmState.state == ST25FTM_RX)   { totalLg = gFtmState.rx.totalDataLength; }
  else                                      { totalLg = 0; }
  return totalLg;
}

/*! Get the number of bytes that have been resent.
  * @return The number of bytes that have been resent during this transfer.
*/
uint32_t ST25FTM_GetRetryLength(void)
{
  return gFtmState.retryLength;
}

/*! Check if the reception has been completed.
  * @retval 1 is the reception has completed.
  * @retval 0 otherwise.
*/
uint8_t ST25FTM_IsReceptionComplete(void)
{
  return ((gFtmState.state == ST25FTM_RX) && (gFtmState.rx.state == ST25FTM_RX_DONE)) ? ST25FTM_TRUE : ST25FTM_FALSE;
}

/*! Check if the transmission has been completed.
  * @retval 1 is the transmission has completed.
  * @retval 0 otherwise.
*/
uint8_t ST25FTM_IsTransmissionComplete(void)
{
  return ((gFtmState.state == ST25FTM_TX) && (gFtmState.tx.state == ST25FTM_TX_DONE)) ? ST25FTM_TRUE : ST25FTM_FALSE;
}

/*! Check if the ST25FTM state machine is idle.
  * @retval 1 The state machine is Idle.
  * @retval 0 otherwise.
*/
uint8_t ST25FTM_IsIdle(void)
{
  return (gFtmState.state == ST25FTM_IDLE) ? ST25FTM_TRUE : ST25FTM_FALSE;
}

/*! Check if an error occured.
  * @retval 1 An error occured.
  * @retval 0 otherwise.
*/
uint8_t ST25FTM_CheckError(void)
{
  return ((gFtmState.rx.state == ST25FTM_RX_ERROR)  || (gFtmState.tx.state == ST25FTM_TX_ERROR)) ? ST25FTM_TRUE : ST25FTM_FALSE;
}

/*! Reset the ST25FTM state machine.
  */
void ST25FTM_Reset(void)
{
  gFtmState.tx.cmdPtr = NULL;
  gFtmState.tx.cmdLen = 0;
  ST25FTM_ResetTxState(gFtmState.tx.cmdPtr, gFtmState.tx.cmdLen);

  gFtmState.rx.cmdPtr = NULL;
  ST25FTM_ResetRxState(gFtmState.rx.cmdPtr);

  ST25FTM_SetState(ST25FTM_IDLE);

  gFtmState.callIdleEventCb = ST25FTM_TRUE;  // Will generate an event at appropriate time
}


/*! Set the length of a segment (in bytes, max value is ST25FTM_SEGMENT_LEN)
 * @param length Tx segment maximum length in bytes
 */
void ST25FTM_SetTxSegmentMaxLength(uint32_t length)
{
  if(length <= ST25FTM_SEGMENT_LEN) {
    gFtmState.tx.segmentMaxLength = length;
  }
}

/*! Reset the length of a segment to its max value ST25FTM_SEGMENT_LEN */
void ST25FTM_ResetTxSegmentMaxLength(void)
{
  gFtmState.tx.segmentMaxLength = ST25FTM_SEGMENT_LEN;
}

/*! Get the length of a segment (in bytes)
 *  @return Maximum number of bytes in a segment
 */
uint32_t ST25FTM_GetTxSegmentMaxLength(void)
{
  return gFtmState.tx.segmentMaxLength;
}

void ST25FTM_PrintAckCtrl(uint8_t ackCtrl)
{
    ST25FTM_LOG("AckCtrl : %s\n",ST25FTM_AckCtrl_Str[ackCtrl]);
}

void ST25FTM_PrintPosition(uint8_t position)
{
    ST25FTM_LOG("Position : %s\n",ST25FTM_Position_Str[position]);
}

void ST25FTM_PrintAckStatus(ST25FTM_Acknowledge_Status_t ack_status)
{
#if (ST25FTM_ENABLE_LOG != 0)
    switch(ack_status) {
        case ST25FTM_SEGMENT_OK:
            ST25FTM_LOG("Rcvd ST25FTM_SEGMENT_OK\n");
            break;
        case ST25FTM_CRC_ERROR:
            ST25FTM_LOG("Rcvd ST25FTM_CRC_ERROR\n");
            break;
        case ST25FTM_ACK_RFU:
            ST25FTM_LOG("Rcvd ST25FTM_ACK_RFU\n");
            break;
        case ST25FTM_ABORT_TRANSFER:
            ST25FTM_LOG("Rcvd ST25FTM_ABORT_TRANSFER\n");
            break;
        case ST25FTM_ACK_BUSY:
            ST25FTM_LOG("Rcvd ST25FTM_ACK_BUSY\n");
            break;
        case ST25FTM_ACK_ERROR:
            ST25FTM_LOG("Rcvd ST25FTM_ACK_ERROR\n");
            break;
        default:
            ST25FTM_LOG("Rcvd Unknown status!\n");
            break;
    }
#endif
}
