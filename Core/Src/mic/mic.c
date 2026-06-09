/*
 * mic.c — LM393 sound module digital output driver (PA0)
 *
 * Note: the analog output (AO) pin of the LM393 module is used for ADC input.
 * This driver reads only the digital output (DO) pin, which reflects the
 * onboard comparator threshold set by the potentiometer.
 */

#include "mic.h"

#define GPIOA_BASE  0x48000000UL
#define RCC_BASE    0x40021000UL

#define RCC_AHB2ENR (*(volatile uint32_t *)(RCC_BASE  + 0x4C))
#define GPIOA_MODER (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_PUPDR (*(volatile uint32_t *)(GPIOA_BASE + 0x0C))
#define GPIOA_IDR   (*(volatile uint32_t *)(GPIOA_BASE + 0x10))

void mic_init(void)
{
    RCC_AHB2ENR |= (1u);
    GPIOA_MODER  = (GPIOA_MODER & ~0x3u) | 0x0u;   /* PA0 input */
    GPIOA_PUPDR  = (GPIOA_PUPDR & ~0x3u) | 0x1u;   /* pull-up   */
}

uint8_t mic_detected(void)
{
    return (GPIOA_IDR & 1u) ? 0 : 1;   /* DO is active-low */
}
