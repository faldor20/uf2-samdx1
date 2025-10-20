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

#ifndef __FTM_PACKETS_H__
#define __FTM_PACKETS_H__

#include "st25ftm.h"
#include "st25ftm_interface.h"

/*! Helper to decide if a packet has a length byte */
#define ST25FTM_CTRL_HAS_PKT_LEN(msg)   ((msg).b.pktLen != 0U)

/*! Helper to decide if a packet has total length bytes */
#define ST25FTM_CTRL_HAS_TOTAL_LEN(msg) (((msg).b.position) == (uint8_t)(ST25FTM_FIRST_PACKET))

/*! Helper to decide if a packet is the only packet of the transfer */
#define ST25FTM_CTRL_IS_SINGLE_PACKET(ctrl) ((ST25FTM_Packet_Position_t)(ctrl) == (ST25FTM_SINGLE_PACKET))
/*! Helper to decide if a packet is the last packet of the transfer */
#define ST25FTM_CTRL_IS_LAST_PACKET(ctrl) ((ST25FTM_Packet_Position_t)(ctrl) == (ST25FTM_LAST_PACKET))
/*! Helper to decide if a transfer is completed */
#define ST25FTM_CTRL_IS_COMMAND_COMPLETE(msg) (ST25FTM_CTRL_IS_SINGLE_PACKET(((ST25FTM_Ctrl_Byte_t*)msg)->b.position) ||\
                                              ST25FTM_CTRL_IS_LAST_PACKET(((FTM_Ctrl_Byte_t*)msg)->b.position))


/*! Helper to decide if a packet has a CRC and is the only packet of the transfer */
#define ST25FTM_CTRL_IS_ACK_SINGLE(ackCtrl) ((ackCtrl) == (uint8_t)(ST25FTM_ACK_SINGLE_PKT))
/*! Helper to decide if a packet has a CRC and is part of a bigger transfer */
#define ST25FTM_CTRL_IS_SEGMENT_END(ackCtrl) ((ackCtrl) == (uint8_t)(ST25FTM_SEGMENT_END))
/*! Helper to decide if a packet has a CRC */
#define ST25FTM_CTRL_HAS_CRC(msg)       (ST25FTM_CTRL_IS_ACK_SINGLE((msg).b.ackCtrl) || \
                                      ST25FTM_CTRL_IS_SEGMENT_END((msg).b.ackCtrl))


/*! Helper to get packet length byre */
#define ST25FTM_GET_PKT_LEN(msg)        (((ST25FTM_Header_t*)msg)->with_len.length)
/*! Helper to get transfer length when packet length is present */
#define ST25FTM_GET_TOTAL_LEN_WITH_LEN(msg)     (((ST25FTM_Header_t*)msg)->with_len.totalLength)
/*! Helper to get transfer length when packet length is not present */
#define ST25FTM_GET_TOTAL_LEN_WITHOUT_LEN(msg)  (((ST25FTM_Header_t*)msg)->without_len.totalLength)

/*! Defines packet positions in the transfer */
typedef enum {
  ST25FTM_SINGLE_PACKET = 0U,   /*!< This is the only packet for the transfer */
  ST25FTM_FIRST_PACKET = 1U,    /*!< This is the first packet of the transfer */
  ST25FTM_MIDDLE_PACKET = 2U,   /*!< This is a middle packet of the transfer */
  ST25FTM_LAST_PACKET = 3U,     /*!< This is the last packet of the transfer */

  NUMBER_OF_ST25FTM_PACKET_POSITIONS
} ST25FTM_Packet_Position_t;

/*! Defines the policy for packets acknowledgment */
typedef enum {
  ST25FTM_NO_ACK_PACKET = 0U,   /*!< This packet is not part of a segment: no CRC to be computed on this data */
  ST25FTM_SEGMENT_START = 1U,   /*!< This packet is the beginning of a segment: CRC to be initialized with this data */
  ST25FTM_SEGMENT_END = 2U,     /*!< This packet is the end of a segment: a CRC is present and a ACK is expected in response */
  ST25FTM_ACK_SINGLE_PKT = 3U,  /*!< This is the only packet for the segment: a CRC is present and a ACK is expected in response */

  NUMBER_OF_ST25FTM_PACKET_ACKS
} ST25FTM_Packet_Acknowledge_t;

/*! Defines ACK possible status */
typedef enum {
  ST25FTM_SEGMENT_OK = 0,       /*!< Segment data has been successfully validated */
  ST25FTM_CRC_ERROR = 1,        /*!< Segment data has not been successfully validated */
  ST25FTM_ACK_RFU = 2,          /*!< Reserved for Futur Use */
  ST25FTM_ABORT_TRANSFER = 3,   /*!< An unrecoverable error occured */
  ST25FTM_ACK_BUSY=-1,          /*!< The ACK has not been read/written, retry */
  ST25FTM_ACK_ERROR=-2          /*!< An error has occured */
} ST25FTM_Acknowledge_Status_t;

/*! Defines protocol bytes */
typedef enum {
  ST25FTM_CTRL_BYTE = 0x0,      /*!< This is a control byte: b7 = 0 */
  ST25FTM_STATUS_BYTE = 0x80    /*!< This is a status byte: b7 = 1 */
} ST25FTM_Packet_Identifier_t;

/*! Defines control byte content */
typedef union {
 struct {
  uint8_t inSegment:1;      /*!< b0: this packet is part of a segment */
  uint8_t segId:1;          /*!< b1: segment id, allow desync issue detection */
  uint8_t position:2;       /*!< b2-b3: position of the packet in the transfer */
  uint8_t ackCtrl:2;        /*!< b4-b5: position of the packet in the segment */
  uint8_t pktLen:1;         /*!< b6: length byte is present (the packet doesn't use the full length of the buffer) */
  uint8_t type:1;           /*!< b7: control byte = 0 */
  } b;                      /*!< Bitwise access to the control byte */
  uint8_t byte;             /*!< Bytewise access to the control byte */
} ST25FTM_Ctrl_Byte_t;

/*! This struct is used to decode the received ST25FTM headers, it must be packed */
typedef  ST25FTM_PACKED(union)  {
    ST25FTM_PACKED(struct)  {
  uint8_t ctrl;                     /*!< control byte */
  ST25FTM_Packet_Length_t length;   /*!< length byte */
  uint32_t totalLength;             /*!< total transfer length word */
  }  with_len;                      /*!< when length byte is present */
  ST25FTM_PACKED(struct)  {
  uint8_t ctrl;                     /*!< control byte */
  uint32_t totalLength;             /*!< totaltransfer length word */
  }  without_len;                   /*!< when length byte is not present */
}  ST25FTM_Header_t;

/*! Packet information structure */
typedef struct {
  ST25FTM_Ctrl_Byte_t ctrl;     /*!< Control byte */
  uint32_t length;              /*!< Packet length */
  uint32_t totalLength;         /*!< Total transfer length */
  ST25FTM_Crc_t crc;            /*!< CRC value */
  uint8_t* data;                /*!< Payload pointer */
  uint8_t has_crc;              /*!< Packet has CRC */
} ST25FTM_Packet_t;

/*! Defines status byte content */
typedef union {
  struct {
    uint8_t status:4;       /*!< Segment status */
    uint8_t rfu:3;          /*!< Reserved for futur use */
    uint8_t type:1;         /*!< Status byte = 1 */
  } bit;                    /*!< Bitwise access to the status byte */
  uint8_t byte;             /*!< Bytewise access to the status byte */
} ST25FTM_Status_Byte_t;

#endif  // __FTM_PACKETS_H__
