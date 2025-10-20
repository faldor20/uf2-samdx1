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

#ifndef __FTM_INTERFACE_H__
#define __FTM_INTERFACE_H__

#include "st25ftm.h"

#include "st25ftm_config_hal.h"     // Embeds all BSP related defines and includes (defined through compiler INCLUDE path) 


/*! Fast Transfer Mode buffer access status */
typedef enum {
  ST25FTM_MSG_OK =0,        /*!< Message read/write ok */
  ST25FTM_MSG_ERROR,        /*!< The peer device doesn't respond */
  ST25FTM_MSG_BUSY          /*!< The buffer is not empty while writing */
} ST25FTM_MessageStatus_t;

/*! Fast Transfer Mode current message owner */
typedef enum {
  ST25FTM_MESSAGE_EMPTY = 0,        /*!< There is no message */
  ST25FTM_MESSAGE_ME = 1,           /*!< Current message has been written by this device */
  ST25FTM_MESSAGE_PEER = 2,         /*!< Current message has been written by the peer device */
  ST25FTM_MESSAGE_OWNER_ERROR = 3   /*!< An error occured while getting the message owner */
} ST25FTM_MessageOwner_t;

typedef enum {
  ST25FTM_CRC_START,
  ST25FTM_CRC_END,
  ST25FTM_CRC_ACCUMULATE,
  ST25FTM_CRC_ONESHOT
} ST25FTM_crc_control_t;

typedef uint32_t ST25FTM_Crc_t;

#ifndef UNUSED
#define UNUSED(x) (void)x
#endif

/*! Define format of the packet length field, when present */
typedef uint8_t ST25FTM_Packet_Length_t;

/* Interface API */
/* Functions to implement for the platform */
/*! Check what device wrote the current message in the FTM buffer
  * @retval ST25FTM_MESSAGE_EMPTY       The buffer is empty.
  * @retval ST25FTM_MESSAGE_ME          Message has been written by this device.
  * @retval ST25FTM_MESSAGE_PEER        Message has been written by the peer device.
  * @retval ST25FTM_MESSAGE_OWNER_ERROR Message owner cannot be retrieved.
 */
ST25FTM_MessageOwner_t ST25FTM_GetMessageOwner(void);

/*! Read the content of the FTM buffer.
  * @param msg      A buffer used to store read data
                    Buffer length must be greater than ST25FTM_BUFFER_LENGTH.
  * @param msg_len  A pointer used to return the number of bytes read.
  * @retval ST25FTM_MSG_OK      Message successfully read.
  * @retval ST25FTM_MSG_ERROR   Unable to read the message.
*/
ST25FTM_MessageStatus_t ST25FTM_ReadMessage(uint8_t *msg, uint32_t* msg_len);

/*! Write the FTM buffer.
  * @param msg      The buffer containing the data to written.
  * @param msg_len  Number of bytes to write.
  * @retval ST25FTM_MSG_OK      Message successfully written.
  * @retval ST25FTM_MSG_ERROR   Unable to write the message (eg: tag has been removed).
  * @retval ST25FTM_MSG_BUSY    FTM buffer contains a meesage that has not been read yet.
*/
ST25FTM_MessageStatus_t ST25FTM_WriteMessage(uint8_t* msg, uint32_t msg_len);

/*! Initialize the NFC device (dynamic tag or reader) for the FTM.
*/
ST25FTM_Error_t ST25FTM_DeviceInit(void * initHandle);

/*! Release the NFC device (dynamic tag or reader) for the FTM.
*/
ST25FTM_Error_t ST25FTM_DeviceRelease(void * initHandle);

/*! Check if the RF field is present (for dynamic tag only)
*/
void ST25FTM_UpdateFieldStatus(ST25FTM_Field_State_t * rfField);

/*! Initialize the CRC computation */
ST25FTM_Error_t ST25FTM_CRC_Initialize(void);

/*! Releases the CRC computation */
ST25FTM_Error_t ST25FTM_CRC_Release(void);

/*! Compute a CRC32.
  * @param data     Buffer containing the data on which the CRC must be computed.
  * @param length   Number of bytes of data in the buffer.
  * @param control  Define how to compute the crc:
  *                 - starting from the initial value or from previous crc
  *                 - Adding remaining bytes (less than word) with padding
 */
ST25FTM_Crc_t ST25FTM_GetCrc(uint8_t *data, uint32_t length, ST25FTM_crc_control_t control);

#endif  // __FTM_INTERFACE_H__
