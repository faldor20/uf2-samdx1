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

/*! Format and print a buffer of bytes (only beginning and end for long frames)
 *  @param buf Buffer of bytes to be printed
 *  @param len Number of bytes to be printed
 */
void logHexBuf(uint8_t* buf, uint32_t len)
{
#if (ST25FTM_ENABLE_LOG != 0)
    if(len > 32)
    {
        char data[40];
        memset(data,0,sizeof(data));
        memcpy(data, ST25FTM_HEX2STR((uint8_t*)(buf + len - 16),16),32);
        ST25FTM_LOG("%s...%s\r\n",ST25FTM_HEX2STR(buf,16),data);
    } else {
        ST25FTM_LOG("%s\r\n",ST25FTM_HEX2STR(buf,len));
    }
#else
    UNUSED(buf);
    UNUSED(len);
#endif
}

void ST25FTM_CallEventCb(ST25FTM_Protocol_Event_t event, uint8_t  *bufferPtr, uint8_t *sourcePtr, uint32_t bufferLength)
{
    if (gFtmState.events[event].cb != NULL) {
        ST25FTM_EventData_t  eventData;
        eventData.bufferPtr = bufferPtr;
        eventData.sourcePtr = sourcePtr;
        eventData.bufferLength = bufferLength;
        gFtmState.events[event].cb(event, &eventData, gFtmState.events[event].userData);
    }
}

/*! Compute time tick substraction
 * @param a Current time tick
 * @param b Previous time tick
 * @return (a - b) considering tick roll-over */
uint32_t ST25FTM_CompareTime(uint32_t a, uint32_t b)
{
    return (a >= b) ? (a - b) : ((0xFFFFFFFF - b) + 1 + a);
}

