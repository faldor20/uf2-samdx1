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

#ifndef __FTM_INTERNAL_H__
#define __FTM_INTERNAL_H__

#include "st25ftm.h"
#include "st25ftm_interface.h"
#include "st25ftm_packets.h"

/*! Helper to invert word endianness */
#define ST25FTM_CHANGE_ENDIANESS(x) ( (x) = \
                                  (((x)>>24U)&0xFFU) | \
                                  (((x)>>8U)&0xFF00U)| \
                                  (((x)<<8U)&0xFF0000U)| \
                                  (((x)<<24U)&0xFF000000U))

/*! ST25FTM Transmission state machine states */
typedef enum {
  ST25FTM_TX_IDLE,                  /*!< The state machine is idle */
  ST25FTM_TX_INIT_TRANSMISSION,     /*!< Start a transfer */
  ST25FTM_TX_WRITE_SEGMENT,         /*!< Start a segment */
  ST25FTM_TX_WRITE_PKT,             /*!< Write a message */
  ST25FTM_TX_WAIT_READ,             /*!< Wait the message to be read by the peer device */
  ST25FTM_TX_WAIT_ACK,              /*!< Wait acknowledge from the peer device */
  ST25FTM_TX_READ_ACK,              /*!< Read the acknowledge sent by the peer device */
  ST25FTM_TX_DONE,                  /*!< End of the transfer */
  ST25FTM_TX_ERROR,                 /*!< An unrecoverable error occured */

  NUMBER_OF_ST25FTM_TX_STATE
} ST25FTM_TxState_t;

/*! ST25FTM Reception state machine states */
typedef enum {
  ST25FTM_RX_IDLE,              /*!< The state machine is idle */
  ST25FTM_RX_INIT_RECEPTION,    /*!< Start the reception */
  ST25FTM_RX_READ_PKT,          /*!< Read a message */
  ST25FTM_RX_WRITE_ACK,         /*!< Write an acknowledge */
  ST25FTM_RX_WRITE_NACK,        /*!< Write a non-ack  */
  ST25FTM_RX_WRITE_ERR,         /*!< Write an error while checking data integrity */
  ST25FTM_RX_WAIT_ACK_READ,     /*!< Wait the ack to be read */
  ST25FTM_RX_ACK_READ,          /*!< Executed when ack has been read */
  ST25FTM_RX_DONE,              /*!< Reception has completed */
  ST25FTM_RX_ERROR,             /*!< An unrecoverable error occured */

  NUMBER_OF_ST25FTM_RX_STATE
} ST25FTM_RxState_t;

/*! Tx state machine variables */
typedef struct {
  ST25FTM_TxState_t   state;                        /*!< Current Tx state */
  ST25FTM_TxState_t   lastState;                    /*!< Last Tx state */
  uint8_t*        cmdPtr;                           /*!< Pointer on the buffer to be sent */
  uint32_t        cmdLen;                           /*!< Number of bytes in the transfer */
  uint32_t        remainingData;                    /*!< Number of bytes still to be tranfsered */
  uint32_t        frameMaxLength;                   /*!< Max Tx frame length */
  uint32_t        nbError;                          /*!< Number of errors in Tx */
  uint8_t         lastError;                        /*!< Last error that occured in Tx */
  ST25FTM_Send_Ack_t  sendAck;                      /*!< Ackowledgment policy for the transfer */
  uint8_t*        dataPtr;                          /*!< Pointer on the data curently being transmitted */
  uint8_t*        segmentPtr;                       /*!< Pointer on the segment currently being transmitted */
  uint8_t*        segmentStart;                     /*!< Pointer on the beginning of the segment currently being transmitted, used for retransmission */
  uint32_t        segmentLength;                    /*!< Length of a segment, in bytes  */
  uint32_t        segmentRemainingData;             /*!< Number of bytes still to be transmitted in the segment */
  uint32_t        segmentMaxLength;                 /*!< Max segment length in bytes */
  uint8_t         retransmit;                       /*!< Segment id, used to detect retransmission */
  uint32_t        pktIndex;                         /*!< Packet index  */
  uint32_t        segmentIndex;                     /*!< Segment index */
  uint32_t        totalDataLength;                  /*!< Number of bytes actually transmitted (including metadata) */
  uint8_t         packetBuf[ST25FTM_BUFFER_LENGTH]; /*!< Packet buffer */
  uint32_t        packetLength;                     /*!< Packet length */
  uint32_t        segmentNumber;                    /*!< Number of segment */
  uint32_t        trimmingDelay;                    /*!< Tx delay trimming in ms */
  ftm_data_cb     getdata_cb;                       /*!< Callback when fetching data to be sent (optional) */
  uint8_t         callTxDoneEventCb;                /*!< Call ST25FTM_TX_DONE event callback */
  uint8_t         callTxErrorEventCb;               /*!< Call ST25FTM_TX_ERROR event callback */
} ST25Ftm_InternalTxState_t;

/*! Rx state machine variables */
typedef struct {
  ST25FTM_RxState_t state;                          /*!< Current Rx state */
  ST25FTM_RxState_t lastState;                      /*!< Last Rx state */
  uint8_t       isNewFrame;                         /*!< A new transfer started */
  uint8_t*      cmdPtr;                             /*!< Pointer to the reception buffer */
  uint32_t*     cmdLen;                             /*!< Reception buffer length / used to return transfer length */
  uint32_t      maxCmdLen;                          /*!< Max length of the reception */
  uint32_t      nbError;                            /*!< Number of errors in Rx */
  uint8_t       lastError;                          /*!< Last error in Rx */
  uint8_t       unrecoverableError;                 /*!< An unrecoverable error occured */
  uint32_t      frameMaxLength;                     /*!< Max Rx frame length */
  uint32_t      receivedLength;                     /*!< Number of bytes received */
  uint32_t      validLength;                        /*!< Number of bytes to be acknowledged */
  uint32_t      validReceivedLength;                /*!< Number of bytes acknowledged */
  uint32_t      totalValidReceivedLength;           /*!< Total number of bytes acknowledged */
  uint32_t      totalDataLength;                    /*!< Number of bytes actually received (including metadata) */
  uint8_t*      segmentPtr;                         /*!< Pointer on the segment */
  uint32_t      segmentLength;                      /*!< Length of the segment */
  uint8_t*      dataPtr;                            /*!< Pointer on the data currently being transmitted */
  uint8_t       lastAck;                            /*!< Last packet had a CRC, a ACK is expected from peer device */
  uint8_t       rewriteOnFieldOff;                  /*!< Used to rewrite the last message when RF field goes Off */
  uint8_t       ignoreRetransSegment;               /*!< Used to ignore a retransmitted segment */
  ST25FTM_Packet_Position_t pktPosition;            /*!< Indicated position of the packet in the transmission */
  uint32_t      segmentNumber;                      /*!< Index of current segment */
  ftm_data_cb   recvdata_cb;                        /*!< Optional callback used when data has been received */
  uint32_t      readBufferOffset;                   /*!< Number of bytes currently received */
  uint8_t       callRxDoneEventCb;                  /*!< Call ST25FTM_RX_DONE event callback */
  uint8_t       callRxErrorEventCb;                 /*!< Call ST25FTM_RX_ERROR event callback */
} ST25Ftm_InternalRxState_t;

/*! ST25FTM events callback data */
typedef struct {
  ftm_events_cb_t           cb;                     /*!< Optional callbacks used for events */
  void                      *userData;              /*!< User data used during callbacks call */
} ST25FTM_EventCallback_t;

/*! ST25FTM global variables */
typedef struct {
  ST25FTM_State_t           state;                  /*!< Current state */
  ST25FTM_Field_State_t     rfField;                /*!< Is RF field present */
  uint32_t                  retryLength;            /*!< Number of bytes retransmitted */
  uint32_t                  lastTick;               /*!< Timestamp of the previous call to the ST25FTM state machine */
  uint32_t                  nbOfConsecutiveRegisterError;           /*! Incremented when the driver failed to acces a FTM register */
  ST25FTM_EventCallback_t   events[NUMBER_OF_ST25FTM_EVENTS];       /*!< Optional callbacks used for events */
  uint8_t                   callIdleEventCb;        /*!< Call EVENT_FTM_BACK_TO_IDLE event callback */
  ST25Ftm_InternalTxState_t tx;                     /*!< Tx state variables */
  ST25Ftm_InternalRxState_t rx;                     /*!< Rx state variables */
  uint8_t                   stateMachineTransition; /*!< Reports any state machine transition during last Runner (used for sequencer type management) */
  uint8_t                   runnerPolicy;           /*!< Defines runner policy (either loop internally until no more state transition or leave application to handle loop for transitions) */
} ST25FTM_InternalState_t;

/* Global variables */
extern ST25FTM_InternalState_t gFtmState;


/* Utilities functions */
void        logHexBuf(uint8_t* buf, uint32_t len);
void        ST25FTM_PrintAckCtrl(uint8_t ackCtrl);
void        ST25FTM_PrintPosition(uint8_t position);
void        ST25FTM_PrintAckStatus(ST25FTM_Acknowledge_Status_t ack_status);
uint32_t    ST25FTM_CompareTime(uint32_t a, uint32_t b);

void        ST25FTM_CallEventCb(ST25FTM_Protocol_Event_t event, uint8_t  *bufferPtr, uint8_t *sourcePtr, uint32_t bufferLength);

void        ST25FTM_TxStateInit(void);
void        ST25FTM_RxStateInit(void);
void        ST25FTM_TxStateMachine(void);
void        ST25FTM_RxStateMachine(void);
void        ST25FTM_TxResetSegment(void);

void        ST25FTM_ResetRxState(uint8_t *cmdPtr);
void        ST25FTM_ResetTxState(uint8_t *cmdPtr, uint32_t cmdLen);
void        ST25FTM_SetRxState(ST25FTM_RxState_t state);
void        ST25FTM_SetTxState(ST25FTM_TxState_t state);

#endif  // __FTM_INTERNAL_H__
