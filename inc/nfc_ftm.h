#ifndef NFC_FTM_H
#define NFC_FTM_H

#include <stdbool.h>

bool nfc_ftm_start(void);
uint8_t nfc_ftm_poll(void);

/* Ready sequence management for NFC tag signaling */
void ST25FTM_SetReadySequence(void);
void ST25FTM_ClearReadySequence(void);

#endif



