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

#ifndef __FTM_H__
#define __FTM_H__

#include <stdint.h>

#include "st25ftm_config_hal.h"     // Embeds all BSP related defines and includes (defined through compiler INCLUDE path) 

/*! ST25FTM main state machine states */
typedef enum {
  ST25FTM_IDLE = 0, /*!< The state machine is idle */
  ST25FTM_TX,       /*!< A transmission is on-going */
  ST25FTM_RX,       /*!< A reception is on-going */

  NUMBER_OF_ST25FTM_STATE
} ST25FTM_State_t;

/*! RF field state (for dynamic tag) */
typedef enum {
  ST25FTM_FIELD_OFF,    /*!< RF field is OFF */
  ST25FTM_FIELD_ON      /*!< RF field is ON */
} ST25FTM_Field_State_t;

/*! Handshake selection */
typedef enum {
  ST25FTM_SEND_WITHOUT_ACK=0,       /*!< The transfer does not require acknowledges*/
  ST25FTM_SEND_WITH_ACK=1,          /*!< The transfer requests acknowledges from peer device */
} ST25FTM_Send_Ack_t;

/*! FTM events */
typedef enum {
  EVENT_FTM_BACK_TO_IDLE,       /*!< Event raised when the state machine goes back to IDLE state (bufferPtr=NULL, sourcePtr=NULL, bufferLength=0) */

  EVENT_FTM_RX_NEW_FRAME,       /*!< Event raised when the very first packet is received (bufferPtr=buffer start @, sourcePtr=NULL, bufferLength=awaited data lg) */
  EVENT_FTM_RX_WAIT_ACK_READ,   /*!< Event raised when waiting the RX ACK to be read is under process (bufferPtr=waitstatus @, sourcePtr=NULL, bufferLength=1  with waitstatus=0x00 if wait starts and waitstatus=0x01 if wait ends) */
  EVENT_FTM_RX_NEW_PKT,         /*!< Event raised when a new packet is received (bufferPtr=dest buffer, sourcePtr=src pkt buffer, bufferLength=data lg) */
  EVENT_FTM_RX_NEW_SEGMENT,     /*!< Event raised when a new validated segment is received (bufferPtr=segment start @, sourcePtr=NULL, bufferLength=segment data lg) */
  EVENT_FTM_RX_DONE,            /*!< Event raised when full data is received and validated (bufferPtr=buffer@, sourcePtr=NULL, bufferLength=total data lg) */

  EVENT_FTM_TX_WAIT_READ,       /*!< Event raised when waiting the TX data to be read is under process (bufferPtr=waitstatus @, sourcePtr=NULL, bufferLength=1  with waitstatus=0x00 if wait starts and waitstatus=0x01 if wait ends) */
  EVENT_FTM_TX_WAIT_ACK_READ,   /*!< Event raised when waiting the TX ACK to be read is under process (bufferPtr=waitstatus @, sourcePtr=NULL, bufferLength=1  with waitstatus=0x00 if wait starts and waitstatus=0x01 if wait ends) */
  EVENT_FTM_TX_NEW_PKT,         /*!< Event raised when a new packet is transmitted (bufferPtr=dest buffer, sourcePtr=src pkt buffer, bufferLength=data lg) */
  EVENT_FTM_TX_NEW_SEGMENT,     /*!< Event raised when a new segment is transmitted (bufferPtr=segment start @, sourcePtr=NULL, bufferLength=segment data lg) */
  EVENT_FTM_TX_DONE,            /*!< Event raised when full data is transmitted (bufferPtr=buffer@, sourcePtr=NULL, bufferLength=total data lg) */

  EVENT_FTM_ERROR,              /*!< Event raised when reception/transmission error raised (bufferPtr=last error @, sourcePtr=NULL, bufferLength=1) */

  NUMBER_OF_ST25FTM_EVENTS
} ST25FTM_Protocol_Event_t;

/*! FTM errors */
#define ST25FTM_ERROR_NONE              0x00
#define ST25FTM_ERROR_TX_ACK            0x01
#define ST25FTM_ERROR_TX_MAXREACHED     0x02
#define ST25FTM_ERROR_RX_TOOMUCH        0x81
#define ST25FTM_ERROR_RX_LENERROR       0x82
#define ST25FTM_ERROR_RX_LENLASTPKT     0x83
#define ST25FTM_ERROR_RX_LENNULL        0x85
#define ST25FTM_ERROR_RX_PEERBUSY       0x87
#define ST25FTM_ERROR_RX_ACKSTATE       0x88
#define ST25FTM_ERROR_RX_POINTER        0x8B
#define ST25FTM_ERROR_RX_BIGGERTHANRX   0x8C
#define ST25FTM_ERROR_RX_FIRSTMISSED    0x8D
#define ST25FTM_ERROR_RX_SEGMENTISSUE   0x8E
#define ST25FTM_ERROR_RX_MAXREACHED     0x8F

/*! FTM Runner policies */
#define ST25FTM_RUNNER_LOOP_INTERNALLY          0x00        /*!< Loop internally until no more state machine transition (default policy) */
#define ST25FTM_RUNNER_LOOP_WITHIN_APPLICATION  0x01        /*!< Leave the application handle loop for state machine transitions */

/*! FTM Wait event status */
#define ST25FTM_EVENT_WAIT_START  0x00
#define ST25FTM_EVENT_WAIT_END    0x01

/*! ST25FTM events structure */
typedef struct {
  uint8_t       *bufferPtr;         /*!< Buffer pointer */
  uint8_t       *sourcePtr;         /*!< Source data pointer (used mainly with RX/TX_NEW_PKT events) */
  uint32_t      bufferLength;       /*!< Buffer data length */
} ST25FTM_EventData_t;

/*! Error codes definitions */
#define ST25FTM_OK          (0)
#define ST25FTM_ERROR       (-1)
#define ST25FTM_BUSY        (-2)
#define ST25FTM_TIMEOUT     (-3)
#define ST25FTM_NACK        (-102)

#define ST25FTM_FALSE       0U
#define ST25FTM_TRUE        1U


typedef int32_t ST25FTM_Error_t;

/*! Prototype for callbacks to get data to send or to write received data */
typedef void (*ftm_data_cb)(uint8_t* buf, uint8_t *src, uint32_t length);
typedef void (*ftm_events_cb_t)(ST25FTM_Protocol_Event_t event, ST25FTM_EventData_t *eventData, void *userData);

/* Deprecated functions */
// /*! ST25FTM_Init() is DEPRECATED!           Please use ST25FTM_Initialize() instead. */
// ST25FTM_DEPRECATED void                    ST25FTM_Init(void);
// /*! ST25FTM_SendCommand() is DEPRECATED!    Please use ST25FTM_SendStart() instead (with callback being registered through event EVENT_FTM_TX_NEW_PKT). */
// ST25FTM_DEPRECATED void                    ST25FTM_SendCommand(uint8_t* data, uint32_t length, ST25FTM_Send_Ack_t ack, ftm_data_cb data_cb);
// /*! ST25FTM_ReceiveCommand() is DEPRECATED! Please use ST25FTM_ReceiveStart() instead (with callback being registered through event EVENT_FTM_RX_NEW_PKT). */
// ST25FTM_DEPRECATED void                    ST25FTM_ReceiveCommand(uint8_t* data, uint32_t *length, ftm_data_cb data_cb);


/* ST25FTM API */
uint32_t                ST25FTM_GetVersion(void);
ST25FTM_Error_t         ST25FTM_Initialize(void * initHandle);
ST25FTM_Error_t         ST25FTM_Release(void * initHandle);
ST25FTM_Error_t         ST25FTM_RegisterEvent(ST25FTM_Protocol_Event_t event, ftm_events_cb_t event_cb, void *userData);
ST25FTM_Error_t         ST25FTM_SendStart(uint8_t* data, uint32_t length, ST25FTM_Send_Ack_t ack);
ST25FTM_Error_t         ST25FTM_ReceiveStart(uint8_t* data, uint32_t maxLength, uint32_t *length);

uint8_t                 ST25FTM_Runner(void);

ST25FTM_Error_t         ST25FTM_SetRunnerPolicy(uint8_t policy);
ST25FTM_State_t         ST25FTM_Status(void);
void                    ST25FTM_SetTxFrameMaxLength(uint32_t len);
uint32_t                ST25FTM_GetTxFrameMaxLength(void);
void                    ST25FTM_SetRxFrameMaxLength(uint32_t len);
uint32_t                ST25FTM_GetRxFrameMaxLength(void);
void                    ST25FTM_SetTxTrim(uint32_t len);
uint32_t                ST25FTM_GetTxTrim(void);
ST25FTM_Field_State_t   ST25FTM_GetFieldState(void);
uint32_t                ST25FTM_GetTransferProgress(void);
uint32_t                ST25FTM_GetAvailableDataLength(void);
uint8_t                 ST25FTM_ReadBuffer(uint8_t *dst,  uint32_t length);
uint32_t                ST25FTM_GetReadBufferOffset(void);
uint32_t                ST25FTM_GetTotalLength(void);
uint32_t                ST25FTM_GetRetryLength(void);
void                    ST25FTM_Reset(void);
void                    ST25FTM_SetTxSegmentMaxLength(uint32_t length);
uint32_t                ST25FTM_GetTxSegmentMaxLength(void);
void                    ST25FTM_ResetTxSegmentMaxLength(void);

uint8_t                 ST25FTM_IsNewFrame(void);               /*! WARNING: Use of the event EVENT_FTM_RX_NEW_FRAME should be prefered */
uint8_t                 ST25FTM_IsReceptionComplete(void);      /*! WARNING: Use of the event EVENT_FTM_RX_DONE should be prefered */    
uint8_t                 ST25FTM_IsTransmissionComplete(void);   /*! WARNING: Use of the event EVENT_FTM_TX_DONE should be prefered */
uint8_t                 ST25FTM_CheckError(void);               /*! WARNING: Use of the event EVENT_FTM_ERROR should be prefered */
uint8_t                 ST25FTM_IsIdle(void);                   /*! WARNING: Use of the event EVENT_FTM_BACK_TO_IDLE should be prefered */

#endif  // __FTM_H__
