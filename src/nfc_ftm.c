/* NFC FTM runner glue with UF2 block receiver */

#include <string.h>
#include "uf2.h"
#include "nfc_ftm.h"
#include "lib/ST25FTM/Inc/st25ftm.h"
/* ST25DV driver is used in the port file; not needed here */

/* Platform must provide initialized ST25DV handle; weak default returns NULL */
extern void *ST25FTM_GetSt25dvHandle(void);

typedef struct {
    uint8_t buf[512];
    uint32_t filled;
    WriteState ws;
    bool active;
    uint8_t cmdId;
    bool cmdSeen;
    bool transferring;
    bool block_ready;
    bool argSeen;
    uint8_t cmdArg;
    bool isBinary;
    bool error;
    uint32_t binAddr;
} NfcUf2Rx;

static NfcUf2Rx g_rx;

// mailbox is only ever 256 bytes, so we need to buffer the data in a larger buffer
static uint8_t g_ftm_rx_buf[512];
static uint32_t g_ftm_rx_len;

static bool g_wait_after_response = false;

static void ftm_arm_rx(void) {
    /* Reset local RX state and arm the library reception */
    g_rx.active = true;
    g_rx.cmdId = 0xFF;
    g_rx.cmdSeen = false;
    g_rx.binAddr = APP_START_ADDRESS;

    g_ftm_rx_len = sizeof(g_ftm_rx_buf);
    (void)ST25FTM_ReceiveStart(g_ftm_rx_buf, sizeof(g_ftm_rx_buf), &g_ftm_rx_len);
}

/* Normal FTM mode */

/* Command protocol constants (aligned with ST's examples) */
#define FTM_RESP_CMD 0x80
#define FTM_STATUS_VALID 0x81
#define FTM_STATUS_ERROR 0x82
#define FTM_STATUS_UNKNOWN 0x84

#define FTM_CMD_FW_UPGRADE 0x04
#define FTM_CMD_FW_UPGRADE_BIN 0x05
/* Test command: reply with basic VALID status; first response byte will be 0x99 */
#define FTM_CMD_TEST 0x66

static void ftm_send_basic_response(uint8_t cmd, uint8_t status) {
    uint8_t resp[2];
    resp[0] = (uint8_t)(FTM_RESP_CMD | cmd);
    resp[1] = status;
    /* Use ACKed TX for reliability */
    ST25FTM_SendStart(resp, 2, ST25FTM_SEND_WITH_ACK);
}

static bool ftm_flush_binary_rows(void) {
    while (g_rx.filled >= FLASH_ROW_SIZE) {
        if (g_rx.binAddr < APP_START_ADDRESS || (g_rx.binAddr + FLASH_ROW_SIZE) > FLASH_SIZE) {
            return false;
        }

        flash_write_row((uint32_t *)g_rx.binAddr, (uint32_t *)g_rx.buf);
        g_rx.binAddr += FLASH_ROW_SIZE;

        g_rx.filled -= FLASH_ROW_SIZE;
        if (g_rx.filled)
            memmove(g_rx.buf, g_rx.buf + FLASH_ROW_SIZE, g_rx.filled);
    }
    return true;
}

static void ftm_event_cb(ST25FTM_Protocol_Event_t event, ST25FTM_EventData_t *eventData,
                         void *userData) {
    (void)userData;
    switch (event) {
    case EVENT_FTM_RX_NEW_FRAME:
        g_rx.active = true;
        /* If we're in the middle of a firmware transfer, preserve state across frames. */
        break;
    case EVENT_FTM_RX_NEW_PKT:
        g_wait_after_response = true;
        break;
    case EVENT_FTM_RX_NEW_SEGMENT:

        if (eventData && eventData->bufferLength) {
            // uint32_t remaining = eventData->bufferLength;
            uint32_t remaining = ST25FTM_GetAvailableDataLength();
            /* Only the first segment of a NEW transfer carries a 1-byte command header. */
            if (!g_rx.transferring && !g_rx.cmdSeen && g_rx.filled == 0 && remaining) {
                uint8_t b = 0;
                if (ST25FTM_ReadBuffer(&b, 1) == 0) {
                    g_rx.cmdSeen = true;
                    g_rx.cmdId = b;
                    remaining--;
                    if (g_rx.cmdId == FTM_CMD_FW_UPGRADE || g_rx.cmdId == FTM_CMD_FW_UPGRADE_BIN) {
                        g_rx.transferring = true;
                        g_rx.argSeen = false;
                        g_rx.isBinary = (g_rx.cmdId == FTM_CMD_FW_UPGRADE_BIN);
                        g_rx.binAddr = APP_START_ADDRESS;
                        g_rx.error = false;
                    }
                }
            }
            if (remaining) {
                if (g_rx.cmdId == FTM_CMD_FW_UPGRADE || g_rx.cmdId == FTM_CMD_FW_UPGRADE_BIN) {
                    /* Read directly into UF2 block buffer */
                    while (remaining) {
                        uint32_t space = sizeof(g_rx.buf) - g_rx.filled;
                        uint32_t chunk = remaining < space ? remaining : space;
                        if (chunk == 0)
                            break;
                        if (ST25FTM_ReadBuffer(&g_rx.buf[g_rx.filled], chunk) != 0)
                            break;
                        g_rx.filled += chunk;
                        remaining -= chunk;
                        if (!g_rx.error) {
                            if (g_rx.filled == 512) {
                                if (g_rx.cmdId == FTM_CMD_FW_UPGRADE_BIN) {
                                    if (!write_block(0, g_rx.buf, false, &g_rx.ws)) {
                                        g_rx.error = true;
                                    }
                                    g_rx.filled = 0;
                                } else {
                                    if (!ftm_flush_binary_rows()) {
                                        g_rx.error = true;
                                    }
                                }
                            }
                        }
                        if (g_rx.error) {
                            g_rx.filled = 0;
                        }
                    }
                } else {
                    /* Drain and drop non-UF2 command payload to free buffer */
                    while (remaining) {
                        uint8_t b = 0;
                        if (ST25FTM_ReadBuffer(&b, 1) != 0)
                            break;
                        remaining--;
                    }
                }
            }
        }
        break;
    case EVENT_FTM_RX_DONE:
        if (g_rx.cmdId == FTM_CMD_FW_UPGRADE_BIN || g_rx.cmdId == FTM_CMD_FW_UPGRADE) {
            if (g_rx.filled && !g_rx.error && (g_rx.cmdId == FTM_CMD_FW_UPGRADE_BIN)) {
                memset(&g_rx.buf[g_rx.filled], 0xff, FLASH_ROW_SIZE - g_rx.filled);
                g_rx.filled = FLASH_ROW_SIZE;
                if (!ftm_flush_binary_rows()) {
                    g_rx.error = true;
                }
            }

            ftm_send_basic_response(g_rx.cmdId, g_rx.error ? FTM_STATUS_ERROR : FTM_STATUS_VALID);

            if (!g_rx.error) {
                resetHorizon = timerHigh + 30;
            }

            g_rx.filled = 0;
        } else if (g_rx.cmdId == FTM_CMD_TEST) {

            /* Test command 0x99: acknowledge (response[0] will be 0x99) */
            ftm_send_basic_response(g_rx.cmdId, FTM_STATUS_VALID);

        } else {
            /* Unknown command: respond with unknown status if a command was seen */
            if (g_rx.cmdSeen) {
                ftm_send_basic_response(g_rx.cmdId, FTM_STATUS_UNKNOWN);
            }
        }
        g_rx.active = false;
        g_rx.transferring = false;
        break;
    default:
        break;
    }
}

bool nfc_ftm_start(void) {
    /* Normal FTM mode */
    memset(&g_rx, 0, sizeof(g_rx));
    if (ST25FTM_Initialize(ST25FTM_GetSt25dvHandle()) != ST25FTM_OK) {
        return false;
    }
    ST25FTM_SetRunnerPolicy(ST25FTM_RUNNER_LOOP_WITHIN_APPLICATION);
    ST25FTM_RegisterEvent(EVENT_FTM_RX_NEW_SEGMENT, ftm_event_cb, NULL);
    ST25FTM_RegisterEvent(EVENT_FTM_RX_DONE, ftm_event_cb, NULL);
    ST25FTM_RegisterEvent(EVENT_FTM_RX_NEW_PKT, ftm_event_cb, NULL);
    ST25FTM_RegisterEvent(EVENT_FTM_RX_NEW_FRAME, ftm_event_cb, NULL);

    ftm_arm_rx();

    led_init_flash(3, false, 100);
    /* Signal that device is ready for FTM by writing sequence to NFC tag memory */
    // ST25FTM_SetReadySequence();
    return true;
}

void nfc_ftm_poll(void) {
    /* Normal FTM mode - poll as fast as possible to avoid mailbox overflow */
    uint8_t remaining = 1;
    while (remaining) {
        remaining = ST25FTM_Runner();
    }

    /* Write any ready blocks */

    if (!g_rx.transferring) {
        delay(100);
    } else {
        if (g_wait_after_response) {
            // we wait for at least 100ms after the response is sent then sleep for a bit
            delay(150);
            g_wait_after_response = false;
        } else {
            delay(20);
        }
    }
}
