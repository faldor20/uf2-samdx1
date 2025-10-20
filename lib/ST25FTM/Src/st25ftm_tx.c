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

#include <string.h>

#include "st25ftm.h"
#include "st25ftm_interface.h"
#include "st25ftm_packets.h"
#include "st25ftm_internal.h"

/*! Helper to compute maximum number of bytes in a packet */
#define ST25FTM_MAX_DATA_IN_SINGLE_PACKET(state) (((state).tx.frameMaxLength) - sizeof(ST25FTM_Ctrl_Byte_t) )

#if (ST25FTM_ENABLE_LOG != 0)
/*! String version of the FTM transmission state machine for display */
extern const char * ST25FTM_TxState_Str[NUMBER_OF_ST25FTM_TX_STATE];
#endif

void ST25FTM_SetTxState(ST25FTM_TxState_t state)
{
    if (gFtmState.tx.state != state) {
        if (   (state == ST25FTM_TX_WAIT_READ)                      /* New state is TX_WAIT_READ : raise event with wait START */
            || (gFtmState.tx.state == ST25FTM_TX_WAIT_READ)         /* Current state was TX_WAIT_READ : raise event with wait END */
            ) {
            uint8_t status = (state == ST25FTM_TX_WAIT_READ) ? ST25FTM_EVENT_WAIT_START : ST25FTM_EVENT_WAIT_END;
            ST25FTM_CallEventCb(EVENT_FTM_TX_WAIT_READ, &status, NULL, 1);
        }

        if (   (state == ST25FTM_TX_WAIT_ACK)                      /* New state is TX_WAIT_READ : raise event with wait START */
            || (gFtmState.tx.state == ST25FTM_TX_WAIT_ACK)         /* Current state was TX_WAIT_READ : raise event with wait END */
            ) {
            uint8_t status = (state == ST25FTM_TX_WAIT_ACK) ? ST25FTM_EVENT_WAIT_START : ST25FTM_EVENT_WAIT_END;
            ST25FTM_CallEventCb(EVENT_FTM_TX_WAIT_ACK_READ, &status, NULL, 1);
        }

        gFtmState.tx.state = state;
        ST25FTM_LOG("TxState = %s\n", ST25FTM_TxState_Str[gFtmState.tx.state]);

        gFtmState.stateMachineTransition = 1;
    }
}

static uint32_t  ST25FTM_Pack(ST25FTM_Packet_t *pkt, uint8_t *msg)
{
  uint32_t          index = 0;
  uint32_t          data_index = 0;
  uint32_t          data_payload = 0;

  msg[index] = pkt->ctrl.byte;
  index++;
  if (pkt->ctrl.b.pktLen != 0U) {
    msg[index] = (uint8_t)pkt->length;
    index++;
  }

  if (ST25FTM_CTRL_HAS_TOTAL_LEN(pkt->ctrl)) {
    uint32_t txTotalLen = pkt->totalLength;
    ST25FTM_CHANGE_ENDIANESS(txTotalLen);
    (void)memcpy(&msg[index],&txTotalLen,sizeof(txTotalLen));
    index += sizeof(pkt->totalLength);
  }

  /* Save data index for Crc computation */
  data_index = index;
  data_payload = pkt->length - ( pkt->has_crc ? 4 : 0);
  if (gFtmState.tx.getdata_cb != NULL) {
    gFtmState.tx.getdata_cb(&msg[index], pkt->data, data_payload);
  }
  else {
    if (gFtmState.events[EVENT_FTM_TX_NEW_PKT].cb != NULL) {
        ST25FTM_EventData_t  eventData;
        eventData.bufferPtr = &msg[index];
        eventData.sourcePtr = pkt->data;
        eventData.bufferLength = data_payload;
        gFtmState.events[EVENT_FTM_TX_NEW_PKT].cb(EVENT_FTM_TX_NEW_PKT, &eventData, gFtmState.events[EVENT_FTM_TX_NEW_PKT].userData);
    }
    else {
        // Default function (simple memcpy) is used
        memcpy(&msg[index], pkt->data, data_payload);
    }
  }

  index += data_payload;

  /* compute CRC */
  if (gFtmState.tx.sendAck == ST25FTM_SEND_WITH_ACK) {
    pkt->crc = 0;
    if (pkt->ctrl.b.ackCtrl == ST25FTM_SEGMENT_START) {
      ST25FTM_GetCrc(&msg[data_index],data_payload,ST25FTM_CRC_START);
    }
    else if (pkt->ctrl.b.ackCtrl == ST25FTM_SEGMENT_END) {
      /* pkt.length contains the CRC length, remove it */
      pkt->crc = ST25FTM_GetCrc(&msg[data_index],data_payload,ST25FTM_CRC_END);
    }
    else if (pkt->ctrl.b.ackCtrl == ST25FTM_ACK_SINGLE_PKT) {
      /* Single frame segment */
      pkt->crc = ST25FTM_GetCrc(&msg[data_index],data_payload,ST25FTM_CRC_ONESHOT);
    }
    else {
      /* middle of a segment */
      ST25FTM_GetCrc(&msg[data_index],data_payload,ST25FTM_CRC_ACCUMULATE);
    }
    if (pkt->has_crc) {
      ST25FTM_CHANGE_ENDIANESS(pkt->crc);
     (void)memcpy(&msg[index],&pkt->crc, 4);
      index += 4;
    }
  }

  return index;
}

void ST25FTM_TxStateInit(void)
{
  gFtmState.tx.state = ST25FTM_TX_IDLE;
  gFtmState.tx.lastState = ST25FTM_TX_IDLE;
  gFtmState.tx.frameMaxLength = 0xFF;
  gFtmState.tx.cmdPtr = NULL;
  gFtmState.tx.cmdLen = 0;
  gFtmState.tx.remainingData = 0;
  gFtmState.tx.nbError = 0;
  gFtmState.tx.lastError = ST25FTM_ERROR_NONE;
  gFtmState.tx.sendAck = ST25FTM_SEND_WITH_ACK;
  gFtmState.tx.dataPtr = NULL;
  gFtmState.tx.segmentPtr = NULL;
  gFtmState.tx.segmentStart = NULL;
  gFtmState.tx.segmentLength = 0;
  gFtmState.tx.segmentRemainingData = 0;
  gFtmState.tx.segmentMaxLength = ST25FTM_SEGMENT_LEN;
  gFtmState.tx.retransmit = ST25FTM_FALSE;
  gFtmState.tx.pktIndex = 0;
  gFtmState.tx.segmentIndex = 0;
  gFtmState.tx.packetLength = 0;
  gFtmState.tx.segmentNumber = 0;
  gFtmState.tx.totalDataLength = 0;
  gFtmState.tx.trimmingDelay = 0;
  gFtmState.tx.getdata_cb = NULL;         // Default function (simple memcpy) will be used
  (void)memset(gFtmState.tx.packetBuf, 0, sizeof(gFtmState.tx.packetBuf));
}

void ST25FTM_ResetTxState(uint8_t *cmdPtr, uint32_t cmdLen)
{
  ST25FTM_SetTxState(ST25FTM_TX_IDLE);
  gFtmState.tx.remainingData =  cmdLen;
  gFtmState.tx.dataPtr = cmdPtr;
  gFtmState.tx.segmentLength = 0;
  gFtmState.tx.segmentRemainingData = 0;
  gFtmState.tx.retransmit = ST25FTM_FALSE;
  gFtmState.tx.pktIndex = 0;
  gFtmState.tx.segmentIndex = 0;
  gFtmState.tx.totalDataLength = 0;
  gFtmState.tx.segmentNumber = 0;
  gFtmState.retryLength = 0;

  (void)ST25FTM_CRC_Initialize();
}

/*! Get the acknowledge from the peer device packet
 * @return Ackowledge status
 */
static ST25FTM_Acknowledge_Status_t ST25FTM_GetAcknowledgeStatus(void)
{
    uint8_t msg[ST25FTM_BUFFER_LENGTH];
    uint32_t msg_len = 0U;
    ST25FTM_Acknowledge_Status_t status;

    if (ST25FTM_ReadMessage(msg, &msg_len) != ST25FTM_MSG_OK) {
        gFtmState.nbOfConsecutiveRegisterError++;
        status = ST25FTM_ACK_BUSY;
    }
    else {
        gFtmState.nbOfConsecutiveRegisterError = 0;
        if (msg[0] == ((uint8_t)ST25FTM_SEGMENT_OK | (uint8_t)ST25FTM_STATUS_BYTE)) {
            status = ST25FTM_SEGMENT_OK;
        }
        else if (msg[0] == ((uint8_t)ST25FTM_CRC_ERROR | (uint8_t)ST25FTM_STATUS_BYTE)) {
            status = ST25FTM_CRC_ERROR;
        }
        else {
            /* Unexpected value, this is not a ACK */
            status = ST25FTM_ACK_ERROR;
        }
    }
    return status;
}


/************** Ftm Tx States ***************/
static void ST25FTM_StateTxCommand(void)
{
  if (gFtmState.tx.remainingData > 0U) {
    uint32_t data_processed;

    ST25FTM_LOG("Starting Segment %d\r\n", gFtmState.tx.segmentNumber);

    /* prepare next segment */
    if (gFtmState.tx.sendAck == ST25FTM_SEND_WITH_ACK) {
      data_processed = (gFtmState.tx.remainingData > (gFtmState.tx.segmentMaxLength - 4U)) ?
                         (gFtmState.tx.segmentMaxLength - 4U) :
                          gFtmState.tx.remainingData;
      gFtmState.tx.segmentLength = data_processed + 4;
      gFtmState.tx.segmentPtr = gFtmState.tx.dataPtr;
    }
    else {
      data_processed = gFtmState.tx.remainingData;
      gFtmState.tx.segmentLength = data_processed;
      gFtmState.tx.segmentPtr = gFtmState.tx.dataPtr;
    }
    gFtmState.tx.segmentStart = gFtmState.tx.segmentPtr;
    gFtmState.tx.remainingData -= data_processed;
    gFtmState.tx.dataPtr += data_processed;
    gFtmState.tx.segmentRemainingData = gFtmState.tx.segmentLength;
    ST25FTM_SetTxState(ST25FTM_TX_WRITE_SEGMENT);
  }
  else {
    // Calling Cb
    ST25FTM_CallEventCb(EVENT_FTM_TX_NEW_SEGMENT, gFtmState.tx.segmentStart, NULL, gFtmState.tx.segmentLength - 4U);

    ST25FTM_LOG("Ending Segment %d\r\n", gFtmState.tx.segmentNumber);
    ST25FTM_SetTxState(ST25FTM_TX_DONE);
    gFtmState.tx.callTxDoneEventCb = ST25FTM_TRUE;
  }
}

static void ST25FTM_StateTxSegment(void)
{
  ST25FTM_Packet_t pkt = {0};
  pkt.length = gFtmState.tx.frameMaxLength - sizeof(ST25FTM_Ctrl_Byte_t);
  pkt.has_crc = ST25FTM_FALSE;

  pkt.data = gFtmState.tx.segmentPtr;
  pkt.ctrl.b.segId = (uint8_t)(gFtmState.tx.segmentNumber % 2U);
  pkt.ctrl.b.ackCtrl = 0;
  pkt.ctrl.b.inSegment = gFtmState.tx.sendAck == ST25FTM_SEND_WITH_ACK;

  ST25FTM_LOG("SegmentLen %d\r\n",gFtmState.tx.segmentLength);
  /* Segment has to be sent over several packets */
  if (gFtmState.tx.segmentRemainingData > ST25FTM_MAX_DATA_IN_SINGLE_PACKET(gFtmState)) {
    if (gFtmState.tx.pktIndex == 0U) {
      /* First Packet
         don't mention packet length if the whole buffer is used */
      pkt.ctrl.b.pktLen = 0;
      pkt.totalLength = gFtmState.tx.cmdLen;
      pkt.ctrl.b.position = (uint8_t)(ST25FTM_FIRST_PACKET);
      pkt.length -= sizeof(pkt.totalLength);
    }
    else {
      /* Middle Packet
         don't mention packet length if the whole buffer is used */
      pkt.ctrl.b.pktLen = 0U;
      pkt.ctrl.b.position = (uint8_t)(ST25FTM_MIDDLE_PACKET);
    }
    if (gFtmState.tx.sendAck == ST25FTM_SEND_WITH_ACK) {
      if (gFtmState.tx.segmentIndex == 0U) {
        pkt.ctrl.b.ackCtrl |= (uint8_t)(ST25FTM_SEGMENT_START);
      }
      if ((gFtmState.tx.segmentRemainingData - pkt.length) < sizeof(pkt.crc)) {
        /* This is to make sure that CRC is not split between 2 packets */
        pkt.ctrl.b.pktLen = 1U;
        pkt.length = gFtmState.tx.segmentRemainingData - sizeof(pkt.crc);
      }
    }
  }
  else {
    /* Single or last Packet command */
    pkt.length = gFtmState.tx.segmentRemainingData;
    if (gFtmState.tx.segmentRemainingData == ST25FTM_MAX_DATA_IN_SINGLE_PACKET(gFtmState)) {
      /* exact fit */
      pkt.ctrl.b.pktLen = 0U;
    }
    else {
      /* smaller than data buffer */
      pkt.ctrl.b.pktLen = 1U;
    }
    if (gFtmState.tx.remainingData == 0U) {
      pkt.ctrl.b.position = (gFtmState.tx.pktIndex == 0U) ? (uint8_t)(ST25FTM_SINGLE_PACKET) : (uint8_t)(ST25FTM_LAST_PACKET);
    }
    else if (gFtmState.tx.pktIndex != 0U) {
      pkt.ctrl.b.position = (uint8_t)(ST25FTM_MIDDLE_PACKET);
    }
    else {
      /* do nothing */
    }
    if (gFtmState.tx.sendAck == ST25FTM_SEND_WITHOUT_ACK) {
      pkt.ctrl.b.ackCtrl = (uint8_t)(ST25FTM_NO_ACK_PACKET);
    }
    else {
      if (gFtmState.tx.sendAck == ST25FTM_SEND_WITH_ACK) {
        pkt.has_crc = ST25FTM_TRUE;
      }

      pkt.ctrl.b.ackCtrl = (gFtmState.tx.segmentLength <= ST25FTM_MAX_DATA_IN_SINGLE_PACKET(gFtmState)) ? (uint8_t)(ST25FTM_ACK_SINGLE_PKT) : (uint8_t)(ST25FTM_SEGMENT_END);
    }
  }
  

//  ST25FTM_LOG("segmentPtr = %x\r\n",gFtmState.tx.segmentPtr);
  /* Note: segmentPtr will overflow when CRC is added for last segment packet
           not an issue since it will be reset for next segment */
  gFtmState.tx.segmentPtr += pkt.length;
  gFtmState.tx.segmentRemainingData -= pkt.length;
  gFtmState.tx.pktIndex++;
  gFtmState.tx.segmentIndex++;

  gFtmState.tx.packetLength = 0;
  (void)memset(gFtmState.tx.packetBuf,0,sizeof(gFtmState.tx.packetBuf));
  gFtmState.tx.packetLength = ST25FTM_Pack(&pkt,gFtmState.tx.packetBuf);

  ST25FTM_LOG("PktId %d\r\n",gFtmState.tx.pktIndex);
  ST25FTM_LOG("PktLen total=%d payload=%d\r\n",gFtmState.tx.packetLength,pkt.length);
  ST25FTM_LOG("Tx (%d) ", gFtmState.tx.packetLength);
  logHexBuf(gFtmState.tx.packetBuf,gFtmState.tx.packetLength);

  ST25FTM_SetTxState(ST25FTM_TX_WRITE_PKT);
}

static void ST25FTM_StateTxPacket(void)
{
  ST25FTM_MessageStatus_t status = ST25FTM_WriteMessage(gFtmState.tx.packetBuf,gFtmState.tx.packetLength);
  if (status == ST25FTM_MSG_OK) {
    gFtmState.nbOfConsecutiveRegisterError = 0;
    gFtmState.tx.totalDataLength += gFtmState.tx.packetLength;
    ST25FTM_SetTxState(ST25FTM_TX_WAIT_READ);
  }
  else if (status == ST25FTM_MSG_BUSY) {
    /* If there is a message in the mailbox, the status is MAILBOX_BUSY
       this is not expected, a timeout may have occured
       it may be a new command or a NACK to request retransmit
       continue with reading the MB to know what to do */
    ST25FTM_LOG("Write error, mailbox busy\r\n");
    gFtmState.tx.nbError++;
    gFtmState.nbOfConsecutiveRegisterError++;
    ST25FTM_SetTxState(ST25FTM_TX_WAIT_READ);
  }
  else {
    /* If a RF operation is on-going, the I2C is NACKED retry later! */
    ST25FTM_LOG("Warning Mailbox busy, I2C is NACKED: retry later!\r\n");
    gFtmState.nbOfConsecutiveRegisterError++;
    ST25FTM_SetTxState(ST25FTM_TX_WRITE_PKT);
  }
}

static void ST25FTM_StateTxWaitRead(void)
{
  ST25FTM_MessageOwner_t    msgOwner = ST25FTM_GetMessageOwner();
  ST25FTM_Ctrl_Byte_t       ctrl_byte;

  ctrl_byte.byte = gFtmState.tx.packetBuf[0];
  if (ST25FTM_CTRL_HAS_CRC(ctrl_byte)) {
    if ((msgOwner == ST25FTM_MESSAGE_EMPTY) || (msgOwner == ST25FTM_MESSAGE_PEER)) {
      ST25FTM_SetTxState(ST25FTM_TX_WAIT_ACK);
    }
  }
  else {
    if (msgOwner == ST25FTM_MESSAGE_EMPTY) {
      if (gFtmState.tx.segmentRemainingData > 0U) {
        ST25FTM_SetTxState(ST25FTM_TX_WRITE_SEGMENT);
      } 
      else if (gFtmState.tx.remainingData > 0U) {
        ST25FTM_SetTxState(ST25FTM_TX_INIT_TRANSMISSION);
      } 
      else {
        // Calling Cb
        ST25FTM_CallEventCb(EVENT_FTM_TX_NEW_SEGMENT, gFtmState.tx.segmentStart, NULL, gFtmState.tx.segmentLength - 4U);

        ST25FTM_LOG("Ending Segment %d\r\n", gFtmState.tx.segmentNumber);
        ST25FTM_SetTxState(ST25FTM_TX_DONE);
        gFtmState.tx.callTxDoneEventCb = ST25FTM_TRUE;
      }
    }
    else if (msgOwner == ST25FTM_MESSAGE_PEER) {
      /* this is not expected
         continue with reading the MB to know what to do
         it may be a new command or a NACK to request retransmit */
      gFtmState.tx.nbError++;
      ST25FTM_SetTxState(ST25FTM_TX_WAIT_ACK);
      ST25FTM_LOG("Write error, mailbox busy 2\r\n");
    } 
    else {
      /* this is still our message, do nothing */
    }
  }
}

static void ST25FTM_StateTxReadAck(void)
{
  ST25FTM_Acknowledge_Status_t ack_status;
  
  ack_status = ST25FTM_GetAcknowledgeStatus();
  ST25FTM_LOG("Rx Ack=%d\r\n",ack_status);
  if (ack_status == ST25FTM_SEGMENT_OK) {
    // Calling Cb
    ST25FTM_CallEventCb(EVENT_FTM_TX_NEW_SEGMENT, gFtmState.tx.segmentStart, NULL, gFtmState.tx.segmentLength - 4U);

    ST25FTM_LOG("Ending Segment %d\r\n", gFtmState.tx.segmentNumber);
    gFtmState.tx.retransmit = ST25FTM_FALSE;
    gFtmState.tx.segmentIndex = 0U;
    gFtmState.tx.segmentNumber++;
    if (gFtmState.tx.remainingData == 0U) {
      ST25FTM_SetTxState(ST25FTM_TX_DONE);
      gFtmState.tx.callTxDoneEventCb = ST25FTM_TRUE;
    } 
    else { 
      /* there are other packets to send */
      ST25FTM_SetTxState(ST25FTM_TX_INIT_TRANSMISSION);
    }
  }
  else if (ack_status == ST25FTM_ACK_BUSY) { 
    /* do nothing */
  }
  else if (ack_status == ST25FTM_CRC_ERROR) {
    ST25FTM_TxResetSegment();
  } 
  else {
    /* this is not a ACK message, it must be a new command */
    gFtmState.tx.lastError = ST25FTM_ERROR_TX_ACK;
    ST25FTM_SetTxState(ST25FTM_TX_ERROR);
    gFtmState.tx.callTxErrorEventCb = ST25FTM_TRUE;
    ST25FTM_LOG("Write error, mailbox busy 3\r\n");
  }
}

void ST25FTM_TxResetSegment()
{
  gFtmState.retryLength += gFtmState.tx.segmentLength - gFtmState.tx.segmentRemainingData;
  gFtmState.lastTick = ST25FTM_TICK();

  /* rewind to retransmit */
  gFtmState.tx.segmentRemainingData = gFtmState.tx.segmentLength;
  gFtmState.tx.pktIndex -= gFtmState.tx.segmentIndex;
  gFtmState.tx.segmentPtr = gFtmState.tx.segmentStart;
  gFtmState.tx.retransmit = ST25FTM_TRUE;
  gFtmState.tx.segmentIndex = 0;
  ST25FTM_SetTxState(ST25FTM_TX_WRITE_SEGMENT);
}

void ST25FTM_TxStateMachine(void)
{
    switch (gFtmState.tx.state)
    {
        case ST25FTM_TX_IDLE:
            ST25FTM_ResetTxState(gFtmState.tx.cmdPtr, gFtmState.tx.cmdLen);
            ST25FTM_SetTxState(ST25FTM_TX_INIT_TRANSMISSION);
            break;
        case ST25FTM_TX_INIT_TRANSMISSION:
            ST25FTM_StateTxCommand();
            break;
        case ST25FTM_TX_WRITE_SEGMENT:
            ST25FTM_StateTxSegment();
            break;
        case ST25FTM_TX_WRITE_PKT:
            ST25FTM_StateTxPacket();
            break;
        case ST25FTM_TX_WAIT_READ:
            ST25FTM_StateTxWaitRead();
            break;
        case ST25FTM_TX_WAIT_ACK:
            if (ST25FTM_GetMessageOwner() == ST25FTM_MESSAGE_PEER) {
              ST25FTM_SetTxState(ST25FTM_TX_READ_ACK);
            }
            break;
        case ST25FTM_TX_READ_ACK:
            ST25FTM_StateTxReadAck();
            break;
//        case ST25FTM_TX_DONE:
//        case ST25FTM_TX_ERROR:
//            /* Nothing to do */
//            break;
        default:
            /* Nothing to do */
            break;
    }

    if (gFtmState.tx.callTxDoneEventCb) {
        gFtmState.tx.callTxDoneEventCb = ST25FTM_FALSE;
        ST25FTM_CallEventCb(EVENT_FTM_TX_DONE, gFtmState.tx.cmdPtr, NULL, gFtmState.tx.cmdLen);
    }

    if (   (gFtmState.nbOfConsecutiveRegisterError >= ST25FTM_MAX_NBR_OF_REGISTER_ERRORS)
        && (gFtmState.tx.lastError != ST25FTM_ERROR_TX_MAXREACHED)) {
        ST25FTM_LOG("ST25FTM_MAX_NBR_OF_REGISTER_ERRORS has been reached");
        gFtmState.tx.lastError = ST25FTM_ERROR_TX_MAXREACHED;
        ST25FTM_SetTxState(ST25FTM_TX_ERROR);
        gFtmState.tx.callTxErrorEventCb = ST25FTM_TRUE;
    }

    if (gFtmState.tx.callTxErrorEventCb) {
        gFtmState.tx.callTxErrorEventCb = ST25FTM_FALSE;
        ST25FTM_CallEventCb(EVENT_FTM_ERROR, &gFtmState.tx.lastError, NULL, sizeof(gFtmState.tx.lastError));
    }
}
