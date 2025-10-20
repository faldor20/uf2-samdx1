/* ST25 FTM HAL configuration - define I2C address and mailbox registers.
 * Adjust these values to match your ST25DV variant.
 */

#ifndef ST25FTM_CONFIG_HAL_H
#define ST25FTM_CONFIG_HAL_H

#include <stdint.h>

/* Packing helper for ST25 macros */
#ifndef ST25FTM_PACKED
#define ST25FTM_PACKED(t) t __attribute__((__packed__))
#endif

/* Default FTM internal packet buffer length (bytes) */
#ifndef ST25FTM_BUFFER_LENGTH
#define ST25FTM_BUFFER_LENGTH 256u
#endif

/* Platform logging - disable by default */
#ifndef ST25FTM_LOG
#define ST25FTM_LOG(...)
#endif

/* Platform timebase and delay hooks (implemented in port file) */
uint32_t ST25FTM_GetMsTick(void);
void ST25FTM_PlatformDelay(uint32_t ms);

#define ST25FTM_TICK()   ST25FTM_GetMsTick()
#define ST25FTM_DELAY(ms) ST25FTM_PlatformDelay(ms)

/* Timeout and segment length defaults */
#ifndef ST25FTM_WAIT_TIMEOUT_IN_MS
#define ST25FTM_WAIT_TIMEOUT_IN_MS 100u
#endif

#ifndef ST25FTM_MAX_NBR_OF_REGISTER_ERRORS
#define ST25FTM_MAX_NBR_OF_REGISTER_ERRORS 5u
#endif

#ifndef ST25FTM_SEGMENT_LEN
#define ST25FTM_SEGMENT_LEN ST25_FTM_MAX_FRAME
#endif




/* Maximum single frame length to read/write (tuned at runtime too). */
#ifndef ST25_FTM_MAX_FRAME
#define ST25_FTM_MAX_FRAME 256u
#endif

#endif /* ST25FTM_CONFIG_HAL_H */


