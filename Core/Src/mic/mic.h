/*
 * mic.h — LM393 sound module digital output driver (PA0)
 */

#ifndef MIC_H
#define MIC_H

#include <stdint.h>

/* Configure PA0 as a digital input with pull-up. */
void mic_init(void);

/* Returns 1 if sound is detected (DO pin pulled low), 0 otherwise. */
uint8_t mic_detected(void);

#endif /* MIC_H */
