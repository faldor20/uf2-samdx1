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

#if (ST25FTM_ENABLE_LOG != 0)
/*! String version of the FTM Reception state machine for display */
extern const char * ST25FTM_RxState_Str[NUMBER_OF_ST25FTM_RX_STATE];
#endif

void ST25FTM_SetRxState(ST25FTM_RxState_t state)
{
    if (gFtmState.rx.state != state) {
        if (   (state == ST25FTM_RX_WAIT_ACK_READ)                      /* New state is RX_WAIT_ACK_READ : raise event with wait START */
            || (gFtmState.rx.state == ST25FTM_RX_WAIT_ACK_READ)         /* Current state was RX_WAIT_ACK_READ : raise event with wait END */
            ) {
            uint8_t status = (state == ST25FTM_RX_WAIT_ACK_READ) ? ST25FTM_EVENT_WAIT_START : ST25FTM_EVENT_WAIT_END;
            ST25FTM_CallEventCb(EVENT_FTM_RX_WAIT_ACK_READ, &status, NULL, 1);
        }

        gFtmState.rx.state = state;
        ST25FTM_LOG("RxState = %s\n", ST25FTM_RxState_Str[gFtmState.rx.state]);

        gFtmState.stateMachineTransition = 1;
    }
}

void ST25FTM_RxStateInit(void)
{
  gFtmState.rx.state = ST25FTM_RX_IDLE;
  gFtmState.rx.lastState = ST25FTM_RX_IDLE;
  gFtmState.rx.frameMaxLength = 0xFF;
  gFtmState.rx.isNewFrame = ST25FTM_FALSE;
  gFtmState.rx.cmdPtr = NULL;
  gFtmState.rx.cmdLen = NULL;
  gFtmState.rx.maxCmdLen = 0;
  gFtmState.rx.nbError = 0;
  gFtmState.rx.lastError = ST25FTM_ERROR_NONE;
  gFtmState.rx.unrecoverableError = ST25FTM_FALSE;
  gFtmState.rx.receivedLength = 0;
  gFtmState.rx.validReceivedLength = 0;
  gFtmState.rx.totalValidReceivedLength = 0;
  gFtmState.rx.totalDataLength = 0;
  gFtmState.rx.segmentPtr = NULL;
  gFtmState.rx.segmentLength = 0;
  gFtmState.rx.dataPtr = NULL;
  gFtmState.rx.validLength = 0;
  gFtmState.rx.lastAck = ST25FTM_FALSE;
  gFtmState.rx.rewriteOnFieldOff = ST25FTM_FALSE;
  gFtmState.rx.ignoreRetransSegment = ST25FTM_FALSE;
  gFtmState.rx.pktPosition = ST25FTM_SINGLE_PACKET;
  gFtmState.rx.segmentNumber = 0;
  gFtmState.rx.recvdata_cb = NULL;                      // Default function (simple memcpy) will be used
  gFtmState.rx.readBufferOffset = 0;
  gFtmState.rx.callRxDoneEventCb = ST25FTM_FALSE;
  gFtmState.rx.callRxErrorEventCb = ST25FTM_FALSE;
}

static ST25FTM_Packet_t ST25FTM_Unpack(uint8_t *msg)
{
  ST25FTM_Packet_t pkt = {0};
  uint32_t hdr_len = sizeof(pkt.ctrl);
  pkt.ctrl.byte = msg[0];

//  ST25FTM_LOG("\t Ctrl: inSegment: 0x%x, segId: 0x%x, pos: 0x%x, ackCtrl: 0x%x, pktLen: 0x%x, type: 0x%x\r\n", pkt.ctrl.b.inSegment, pkt.ctrl.b.segId, pkt.ctrl.b.position, pkt.ctrl.b.ackCtrl, pkt.ctrl.b.pktLen, pkt.ctrl.b.type);

  if (ST25FTM_CTRL_HAS_PKT_LEN(pkt.ctrl)) {
    pkt.length = ST25FTM_GET_PKT_LEN(msg);
    hdr_len += sizeof(ST25FTM_Packet_Length_t);

    if (ST25FTM_CTRL_HAS_TOTAL_LEN(pkt.ctrl)) {
      pkt.totalLength = ST25FTM_GET_TOTAL_LEN_WITH_LEN(msg);
      hdr_len += sizeof(pkt.totalLength);
    }
  }
  else {
    if (ST25FTM_CTRL_HAS_TOTAL_LEN(pkt.ctrl)) {
      pkt.totalLength = msg[1];
      pkt.totalLength = (pkt.totalLength << 8U) + msg[2];
      pkt.totalLength = (pkt.totalLength << 8U) + msg[3];
      pkt.totalLength = (pkt.totalLength << 8U) + msg[4];
      hdr_len += sizeof(pkt.totalLength);
    }
    pkt.length = gFtmState.rx.frameMaxLength - hdr_len;
  }
  if (ST25FTM_CTRL_HAS_CRC(pkt.ctrl)) {
    /* The pkt.length count the crc bytes, remove them */
    pkt.length -= 4;
  }

  /* compute the begining of the payload */
  pkt.data = msg;
  pkt.data += hdr_len;
  return pkt;
}

static void ST25FTM_RewindSegment(void)
{
  gFtmState.rx.dataPtr -= gFtmState.rx.segmentLength;
  gFtmState.rx.receivedLength -= gFtmState.rx.segmentLength;
  gFtmState.rx.segmentLength = 0;
  if (gFtmState.rx.dataPtr < gFtmState.rx.cmdPtr) {
    gFtmState.rx.lastError = ST25FTM_ERROR_RX_POINTER;
    ST25FTM_LOG("FtmRxError11: data pointer out of band\r\n");
  }
}

void ST25FTM_ResetRxState(uint8_t *cmdPtr)
{
  ST25FTM_SetRxState(ST25FTM_RX_IDLE);
  gFtmState.rx.segmentPtr = cmdPtr;
  gFtmState.rx.dataPtr = cmdPtr;
  gFtmState.rx.segmentLength = 0;
  gFtmState.rx.receivedLength = 0;
  gFtmState.rx.validReceivedLength = 0;
  gFtmState.rx.totalValidReceivedLength = 0;
  gFtmState.rx.totalDataLength=0;
  gFtmState.rx.segmentNumber = 0;
  gFtmState.rx.readBufferOffset = 0;
  gFtmState.retryLength = 0;

  (void)ST25FTM_CRC_Initialize();
}


/************** Ftm Rx States ***************/
static void ST25FTM_StateRxCommand(void)
{
  if (gFtmState.rx.receivedLength >= (gFtmState.rx.maxCmdLen)) {
    /* ERROR: receive more data than we can handle */
    gFtmState.rx.nbError++;
    gFtmState.rx.unrecoverableError = ST25FTM_TRUE;
    ST25FTM_SetRxState(ST25FTM_RX_INIT_RECEPTION);
    gFtmState.rx.lastError = ST25FTM_ERROR_RX_TOOMUCH;
    ST25FTM_LOG("FtmRxError1 too much data received\r\n");
    ST25FTM_LOG("gFtmState.rx.receivedLength=%d\r\n",gFtmState.rx.receivedLength);
    ST25FTM_LOG("gFtmState.rx.maxCmdLen=%d\r\n",gFtmState.rx.maxCmdLen);

    ST25FTM_RewindSegment();
  }
  else if (ST25FTM_GetMessageOwner() == ST25FTM_MESSAGE_PEER) {
    gFtmState.rx.rewriteOnFieldOff = ST25FTM_FALSE;
    ST25FTM_SetRxState(ST25FTM_RX_READ_PKT);
  }
  else {
    /* no error, do nothing */
  }
}
 
static void ST25FTM_StateRxPacket(void)
{
  uint8_t msg[ST25FTM_BUFFER_LENGTH];
  uint32_t msg_len = 0;
  ST25FTM_Packet_t pkt;

  (void)memset(msg,0,sizeof(msg));
  if (ST25FTM_ReadMessage(msg, &msg_len) != ST25FTM_MSG_OK) {
    gFtmState.nbOfConsecutiveRegisterError++;
    /* Cannot read MB, retry later */
    ST25FTM_SetRxState(ST25FTM_RX_READ_PKT);
  }
  else {
    gFtmState.nbOfConsecutiveRegisterError = 0;
    ST25FTM_LOG("Rx (%d) ", msg_len);
    logHexBuf(msg,msg_len);
    if (msg_len == 0U) {
      gFtmState.rx.lastError = ST25FTM_ERROR_RX_LENNULL;
      ST25FTM_LOG("FtmRxError5 len = 0\r\n");
      gFtmState.rx.nbError++;
      ST25FTM_SetRxState(ST25FTM_RX_ERROR);
      gFtmState.rx.callRxErrorEventCb = ST25FTM_TRUE;
    }
    else {
      pkt = ST25FTM_Unpack(msg);

      gFtmState.rx.pktPosition = (ST25FTM_Packet_Position_t)pkt.ctrl.b.position;
      if ((pkt.ctrl.b.position == (uint8_t)ST25FTM_SINGLE_PACKET) || (pkt.ctrl.b.position == (uint8_t)ST25FTM_FIRST_PACKET)) {
        gFtmState.rx.segmentNumber = 0;
        if (pkt.ctrl.b.position == (uint8_t)ST25FTM_SINGLE_PACKET) {
          /* pkt length represents the sent data including encryption (but not crc) */
          *gFtmState.rx.cmdLen = pkt.length;
        }
        else {
          /* this represents the payload (unencrypted) length */
          *gFtmState.rx.cmdLen = pkt.totalLength;
        }

        if (*gFtmState.rx.cmdLen > (gFtmState.rx.maxCmdLen)) {
            gFtmState.rx.lastError = ST25FTM_ERROR_RX_BIGGERTHANRX;
            gFtmState.rx.nbError++;
            gFtmState.rx.unrecoverableError = ST25FTM_TRUE;
            ST25FTM_LOG("FtmRxError12 Transfer is bigger than reception buffer (%d vs %d)\r\n", *gFtmState.rx.cmdLen, gFtmState.rx.maxCmdLen);
        }

        if (gFtmState.rx.totalDataLength > 0U) {
          gFtmState.retryLength += gFtmState.rx.totalDataLength;
          ST25FTM_LOG("FtmRxWarning0 Command restarted, length=%d, totalDataLength=%d\r\n",gFtmState.retryLength,gFtmState.rx.totalDataLength);
        }
        gFtmState.rx.totalDataLength = msg_len;
        gFtmState.rx.isNewFrame = ST25FTM_TRUE;
        if (gFtmState.rx.receivedLength != 0U) {
          /* the transmitter started a new command without completing the last one */
          gFtmState.rx.dataPtr = gFtmState.rx.cmdPtr;
          gFtmState.rx.segmentPtr = gFtmState.rx.cmdPtr;
          gFtmState.rx.segmentLength = 0U;
          gFtmState.rx.receivedLength = 0U;
          gFtmState.rx.validReceivedLength = 0U;
          gFtmState.rx.totalValidReceivedLength = 0U;
          gFtmState.rx.readBufferOffset = 0U;
        }
      }
      else {
        if (gFtmState.rx.totalDataLength == 0U) {
          /* we missed first packet: continue the reception and ask for retransmission */
          gFtmState.rx.nbError++;
          gFtmState.rx.lastError = ST25FTM_ERROR_RX_FIRSTMISSED;
          ST25FTM_LOG("FtmRxError13 First packet missed\r\n");
        }
        else {
          /* no error, so do nothing */
        }
        gFtmState.rx.totalDataLength += msg_len;
      }

      if(   ((pkt.ctrl.b.ackCtrl & (uint8_t)ST25FTM_SEGMENT_START) != 0U)
         || (pkt.ctrl.b.ackCtrl == (uint8_t)ST25FTM_ACK_SINGLE_PKT)) {
        ST25FTM_LOG("Starting Segment %d\r\n", gFtmState.rx.segmentNumber);
        /* detect retransmission */
        gFtmState.rx.ignoreRetransSegment = ST25FTM_FALSE;
        if (pkt.ctrl.b.segId != (gFtmState.rx.segmentNumber % 2U)) {
          ST25FTM_LOG("Retransmission %d\r\n", pkt.ctrl.b.segId );
          /* segment is retransmitted, so don't take it into account */
          gFtmState.rx.ignoreRetransSegment = ST25FTM_TRUE;
        }

        if (gFtmState.rx.segmentLength > 0U) {
          gFtmState.retryLength += gFtmState.rx.segmentLength;
          ST25FTM_LOG("FtmRxWarning2 Segment restarted, retryLength=%d\r\n",gFtmState.retryLength);
          ST25FTM_LOG("segmentLength=%d\r\n",gFtmState.rx.segmentLength);
          ST25FTM_LOG("receivedLength=%d\r\n",gFtmState.rx.receivedLength);
          ST25FTM_LOG("totalValidReceivedLength=%d\r\n",gFtmState.rx.totalValidReceivedLength);
        }

        /* rewind if necessary (i.e. when segment_data > 0)
           means that segment has been restarted */
        ST25FTM_RewindSegment();
      }
      if ((gFtmState.rx.receivedLength + pkt.length) > gFtmState.rx.maxCmdLen) {
        /* ERROR: receive more data than we can handle
           this may happen if we miss several start of segment, restart segment */
        gFtmState.rx.nbError++;
        ST25FTM_SetRxState(ST25FTM_RX_INIT_RECEPTION);
        gFtmState.rx.lastError = ST25FTM_ERROR_RX_SEGMENTISSUE;
        ST25FTM_LOG("FtmRxError14 too much data received\r\n");
        ST25FTM_LOG("gFtmState.rx.receivedLength=%d\r\n",gFtmState.rx.receivedLength);
        ST25FTM_LOG("gFtmState.rx.maxCmdLen=%d\r\n",gFtmState.rx.maxCmdLen);

        ST25FTM_RewindSegment();
      }
      else {
        /* don't register the data, if it has already been successfully transmitted */
        if (gFtmState.rx.ignoreRetransSegment == ST25FTM_FALSE) {
          if (gFtmState.rx.recvdata_cb != NULL) {
            gFtmState.rx.recvdata_cb(gFtmState.rx.dataPtr, pkt.data, pkt.length);
          }
          else {
              if (gFtmState.events[EVENT_FTM_RX_NEW_PKT].cb != NULL) {
                  ST25FTM_EventData_t  eventData;
                  eventData.bufferPtr = gFtmState.rx.dataPtr;
                  eventData.sourcePtr = pkt.data;
                  eventData.bufferLength = pkt.length;
                  gFtmState.events[EVENT_FTM_RX_NEW_PKT].cb(EVENT_FTM_RX_NEW_PKT, &eventData, gFtmState.events[EVENT_FTM_RX_NEW_PKT].userData);
              }
              else {
                  // Default function (simple memcpy) is used
                  memcpy(gFtmState.rx.dataPtr, pkt.data, pkt.length);
              }
          }
        }

        if (gFtmState.rx.isNewFrame) {
            // Call Cb
            if (gFtmState.events[EVENT_FTM_RX_NEW_FRAME].cb != NULL) {
              ST25FTM_EventData_t  eventData;
              eventData.bufferPtr = gFtmState.rx.dataPtr;
              eventData.sourcePtr = NULL;
              eventData.bufferLength = *gFtmState.rx.cmdLen;
              gFtmState.events[EVENT_FTM_RX_NEW_FRAME].cb(EVENT_FTM_RX_NEW_FRAME, &eventData, gFtmState.events[EVENT_FTM_RX_NEW_FRAME].userData);

              // Reset isNewFrame only if Cb is called (Resetted in ST25FTM_IsNewFrame() otherwise)
              gFtmState.rx.isNewFrame = ST25FTM_FALSE;
            }
        }

        gFtmState.rx.dataPtr += pkt.length;
        gFtmState.rx.segmentLength += pkt.length;
        gFtmState.rx.receivedLength += pkt.length;

        if (pkt.ctrl.b.ackCtrl == (uint8_t)(ST25FTM_SEGMENT_START)) {
      	  ST25FTM_GetCrc(pkt.data,pkt.length,ST25FTM_CRC_START);
          ST25FTM_SetRxState(ST25FTM_RX_INIT_RECEPTION);
        }
        else if ((pkt.ctrl.b.ackCtrl == (uint8_t)ST25FTM_SEGMENT_END) || (pkt.ctrl.b.ackCtrl == (uint8_t)ST25FTM_ACK_SINGLE_PKT)) {
          ST25FTM_Crc_t computed_crc = 0;
          if (pkt.ctrl.b.ackCtrl == (uint8_t)ST25FTM_ACK_SINGLE_PKT) {
            computed_crc = ST25FTM_GetCrc(pkt.data,pkt.length,ST25FTM_CRC_ONESHOT);
          }
          else if (pkt.ctrl.b.ackCtrl == (uint8_t)ST25FTM_SEGMENT_END) {
            computed_crc = ST25FTM_GetCrc(pkt.data,pkt.length,ST25FTM_CRC_END);
          }
          gFtmState.rx.validLength = gFtmState.rx.segmentLength;
          uint8_t* crc_p = pkt.data + pkt.length;
          uint32_t segment_crc = crc_p[0];
          segment_crc = (segment_crc << 8) + crc_p[1];
          segment_crc = (segment_crc << 8) + crc_p[2];
          segment_crc = (segment_crc << 8) + crc_p[3];
          if (segment_crc == computed_crc) {
            ST25FTM_SetRxState(ST25FTM_RX_WRITE_ACK);
          }
          else {
            ST25FTM_SetRxState(ST25FTM_RX_WRITE_NACK);
          }
        }
        else {
          /* not beginning or end of a segment */
          if (pkt.ctrl.b.inSegment) {
            /* Accumulate CRC */
            ST25FTM_GetCrc(pkt.data,pkt.length,ST25FTM_CRC_ACCUMULATE);
          }
          else {
            /* No segment is used, consider the data as valid */
            gFtmState.rx.totalValidReceivedLength += pkt.length;
            gFtmState.rx.validReceivedLength += pkt.length;

            // Calling Cb
            ST25FTM_CallEventCb(EVENT_FTM_RX_NEW_SEGMENT, (uint8_t *)((uint32_t)gFtmState.rx.segmentPtr - gFtmState.rx.segmentLength), NULL, gFtmState.rx.segmentLength);
          }
          if ((ST25FTM_CTRL_IS_SINGLE_PACKET(pkt.ctrl.b.position)) || (ST25FTM_CTRL_IS_LAST_PACKET(pkt.ctrl.b.position))) {
            if (gFtmState.rx.totalValidReceivedLength == *gFtmState.rx.cmdLen) {
              ST25FTM_SetRxState(ST25FTM_RX_DONE);
              gFtmState.rx.callRxDoneEventCb = ST25FTM_TRUE;
            }
            else {
              /* inconsistent data length */
              gFtmState.rx.nbError++;
              ST25FTM_SetRxState(ST25FTM_RX_ERROR);
              gFtmState.rx.callRxErrorEventCb = ST25FTM_TRUE;
              gFtmState.rx.lastError = ST25FTM_ERROR_RX_LENERROR;
              ST25FTM_LOG("FtmRxError2 Inconsistent length\r\n");
            }
          } 
          else {
            /* this is a start/middle frame, continue reception */
            ST25FTM_SetRxState(ST25FTM_RX_INIT_RECEPTION);
          }
        }
      }
    }
  }
}

static void ST25FTM_StateRxWriteAck(void)
{
  ST25FTM_MessageStatus_t status;
  uint8_t msg[ST25FTM_BUFFER_LENGTH];
  int32_t msg_len = 1;

  (void)memset(msg,0,sizeof(msg));
  if (gFtmState.rx.unrecoverableError) {
	ST25FTM_LOG("FtmRx TxStop (unrecoverableError)\r\n");
	gFtmState.rx.lastAck = ST25FTM_TRUE;
	msg[0] = ((uint8_t)ST25FTM_STATUS_BYTE | (uint8_t)ST25FTM_ABORT_TRANSFER);
	gFtmState.rx.unrecoverableError = ST25FTM_FALSE;
  }
  else if (gFtmState.rx.state == ST25FTM_RX_WRITE_ACK) {
    ST25FTM_LOG("### Send ACK\r\n");
    gFtmState.rx.lastAck = ST25FTM_TRUE;
    msg[0] = ((uint8_t)ST25FTM_STATUS_BYTE | (uint8_t)ST25FTM_SEGMENT_OK);
  }
  else if (gFtmState.rx.state == ST25FTM_RX_WRITE_NACK) {
    ST25FTM_LOG("### Send NACK\r\n");
    gFtmState.rx.lastAck = ST25FTM_FALSE;
    msg[0] = ((uint8_t)ST25FTM_STATUS_BYTE | (uint8_t)ST25FTM_CRC_ERROR);
  }
  else {
    /* Undefined Ack response */
    gFtmState.rx.lastError = ST25FTM_ERROR_RX_ACKSTATE;
    ST25FTM_LOG("FtmRxError8 Unknown ack state %d\r\n",gFtmState.rx.state);
  }
  status = ST25FTM_WriteMessage(msg,msg_len);
  if (status == ST25FTM_MSG_OK) {
    gFtmState.nbOfConsecutiveRegisterError = 0;
    ST25FTM_SetRxState(ST25FTM_RX_WAIT_ACK_READ);
  }
  else if (status == ST25FTM_MSG_BUSY) {
    /* If Mailbox is busy there is a message in the mailbox
    a timeout may have occured */
    gFtmState.nbOfConsecutiveRegisterError++;
    gFtmState.rx.nbError++;
    gFtmState.rx.lastError = ST25FTM_ERROR_RX_PEERBUSY;
    ST25FTM_LOG("FtmRxError7 Mailbox not empty\r\n");

    ST25FTM_SetRxState(ST25FTM_RX_INIT_RECEPTION);
  }
  else {
    /* If a RF operation is on-going, the I2C is NACKED: retry later! */
    ST25FTM_LOG("Warning Mailbox busy, I2C is NACKED: retry later!\r\n");
    gFtmState.nbOfConsecutiveRegisterError++;
  }
}

static void ST25FTM_StateRxWaitAckRead(void)
{
  ST25FTM_MessageOwner_t        msgOwner;

  msgOwner = ST25FTM_GetMessageOwner();
  ST25FTM_LOG("msgOwner=%d\r\n", msgOwner);
  if ((msgOwner == ST25FTM_MESSAGE_EMPTY) || (msgOwner == ST25FTM_MESSAGE_PEER)) {
    ST25FTM_LOG("Ack is read\r\n");
    ST25FTM_SetRxState(ST25FTM_RX_ACK_READ);
  }
}

static void ST25FTM_StateRxAckRead(void)
{
  ST25FTM_LOG("lastAck=%d\r\n",gFtmState.rx.lastAck);
  if (gFtmState.rx.lastAck) {
   /* only consider the data valid once the ack has been read */
    ST25FTM_LOG("ignoreRetrans=%d\r\n",gFtmState.rx.ignoreRetransSegment);
    if (gFtmState.rx.ignoreRetransSegment == ST25FTM_FALSE) {
      ST25FTM_LOG("Ending Segment %d\r\n", gFtmState.rx.segmentNumber);
      gFtmState.rx.segmentPtr = gFtmState.rx.dataPtr;
      gFtmState.rx.validReceivedLength += gFtmState.rx.validLength;
      gFtmState.rx.totalValidReceivedLength += gFtmState.rx.validLength;
      gFtmState.rx.segmentNumber++;

      // Calling Cb
      ST25FTM_CallEventCb(EVENT_FTM_RX_NEW_SEGMENT, (uint8_t *)((uint32_t)gFtmState.rx.segmentPtr - gFtmState.rx.segmentLength), NULL, gFtmState.rx.segmentLength);
    }
    else {
      ST25FTM_LOG("Dropping retrans %d\r\n", gFtmState.rx.segmentNumber);
      gFtmState.rx.dataPtr = gFtmState.rx.segmentPtr;
      gFtmState.rx.receivedLength -= gFtmState.rx.segmentLength;
    }
    gFtmState.rx.segmentLength = 0U;

    ST25FTM_LOG("receivedLen=%d\r\n",gFtmState.rx.receivedLength);
    ST25FTM_LOG("validLen=%d\r\n",gFtmState.rx.validLength);
    ST25FTM_LOG("totalvalid=%d\r\n",gFtmState.rx.totalValidReceivedLength);
    ST25FTM_LOG("expected=%d\r\n",*gFtmState.rx.cmdLen);
  }

  ST25FTM_LOG("pktPosition=%d\r\n",gFtmState.rx.pktPosition);
  if(   ( ST25FTM_CTRL_IS_SINGLE_PACKET(gFtmState.rx.pktPosition) || ST25FTM_CTRL_IS_LAST_PACKET(gFtmState.rx.pktPosition))
     && (gFtmState.rx.lastAck)) {
    if(    ST25FTM_CTRL_IS_LAST_PACKET(gFtmState.rx.pktPosition)
       && (gFtmState.rx.totalValidReceivedLength != *gFtmState.rx.cmdLen)) {
      /* inconsistent data length -> error */
      gFtmState.rx.lastError = ST25FTM_ERROR_RX_LENLASTPKT;
      ST25FTM_LOG("FtmRxError3: Inconsistent length\r\n");
      gFtmState.rx.nbError++;
      ST25FTM_SetRxState(ST25FTM_RX_ERROR);
      gFtmState.rx.callRxErrorEventCb = ST25FTM_TRUE;
    }
    else {
      /* no need to check received length in single packet */
      if (ST25FTM_CTRL_IS_SINGLE_PACKET(gFtmState.rx.pktPosition)) {
        *gFtmState.rx.cmdLen = gFtmState.rx.totalValidReceivedLength;
      }
      ST25FTM_LOG("FtmRx Ack has been read\r\n");
      ST25FTM_SetRxState(ST25FTM_RX_DONE);
      gFtmState.rx.callRxDoneEventCb = ST25FTM_TRUE;
    }
  }
  else {
    /* continue reception if this is not the last packet or this was a NACK */
    ST25FTM_LOG("FtmRx Continue reception\r\n");
    ST25FTM_SetRxState(ST25FTM_RX_INIT_RECEPTION);
  }
}

void ST25FTM_RxStateMachine(void)
{
    switch (gFtmState.rx.state)
    {
        case ST25FTM_RX_IDLE:
            ST25FTM_ResetRxState(gFtmState.rx.cmdPtr);
            ST25FTM_SetRxState(ST25FTM_RX_INIT_RECEPTION);
            break;
        case ST25FTM_RX_INIT_RECEPTION:
            ST25FTM_StateRxCommand();
            break;
        case ST25FTM_RX_READ_PKT:
            ST25FTM_StateRxPacket();
            break;
        case ST25FTM_RX_WRITE_ACK:
        case ST25FTM_RX_WRITE_NACK:
        case ST25FTM_RX_WRITE_ERR:
            ST25FTM_StateRxWriteAck();
            break;
        case ST25FTM_RX_WAIT_ACK_READ:
            ST25FTM_StateRxWaitAckRead();
            break;
        case ST25FTM_RX_ACK_READ:
            ST25FTM_StateRxAckRead();
        break;
//        case ST25FTM_RX_DONE:
//        case ST25FTM_RX_ERROR:
//            /* Nothing to do */
//            break;
        default:
            /* Nothing to do */
            break;
    }

    if (gFtmState.rx.callRxDoneEventCb) {
        gFtmState.rx.callRxDoneEventCb = ST25FTM_FALSE;
        ST25FTM_CallEventCb(EVENT_FTM_RX_DONE, gFtmState.rx.cmdPtr, NULL, *gFtmState.rx.cmdLen);
    }

    if ((gFtmState.nbOfConsecutiveRegisterError >= ST25FTM_MAX_NBR_OF_REGISTER_ERRORS) && (gFtmState.rx.lastError != ST25FTM_ERROR_RX_MAXREACHED)) {
        ST25FTM_LOG("ST25FTM_MAX_NBR_OF_REGISTER_ERRORS has been reached");
        gFtmState.rx.lastError = ST25FTM_ERROR_RX_MAXREACHED;
        ST25FTM_SetRxState(ST25FTM_RX_ERROR);
        gFtmState.rx.callRxErrorEventCb = ST25FTM_TRUE;
    }

    if (gFtmState.rx.callRxErrorEventCb) {
        gFtmState.rx.callRxErrorEventCb = ST25FTM_FALSE;
        ST25FTM_CallEventCb(EVENT_FTM_ERROR, &gFtmState.rx.lastError, NULL, sizeof(gFtmState.rx.lastError));
    }
}
