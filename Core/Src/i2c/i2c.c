/*
 * i2c.c — Bare-metal I2C1 driver, single-byte blocking transmit.
 */

#include "i2c.h"

static void i2c_reset(void)
{
    I2C1_CR1 &= ~(1u << 0);
    for (volatile int i = 0; i < 1000; i++);
    I2C1_ICR  = 0xFFFFFFFFu;
    I2C1_CR1 |=  (1u << 0);
    for (volatile int i = 0; i < 1000; i++);
}

void i2c_send_byte(uint8_t data, uint8_t address)
{
    uint32_t timeout;

    /* Wait for bus idle (BUSY flag, ISR bit 15) */
    timeout = 500000u;
    while ((I2C1_ISR & (1u << 15)) && --timeout);
    if (!timeout) { i2c_reset(); return; }

    /* Clear NACKF and STOPF from any previous transaction */
    I2C1_ICR = (1u << 4) | (1u << 5);

    /* 7-bit address, 1 byte, AUTOEND, START */
    I2C1_CR2 = ((address & 0x7Fu) << 1)
             | (1u << 16)    /* NBYTES = 1 */
             | (1u << 25)    /* AUTOEND   */
             | (1u << 13);   /* START     */

    /* Wait for TXIS (data register empty) or NACKF */
    timeout = 500000u;
    while (!(I2C1_ISR & (1u << 1)) && !(I2C1_ISR & (1u << 4)) && --timeout);

    if ((I2C1_ISR & (1u << 4)) || !timeout) {
        I2C1_ICR = (1u << 4) | (1u << 5);
        if (!timeout) i2c_reset();
        return;
    }

    I2C1_TXDR = data;

    /* Wait for STOPF — AUTOEND issues the STOP condition automatically */
    timeout = 500000u;
    while (!(I2C1_ISR & (1u << 5)) && --timeout);
    I2C1_ICR = (1u << 5);
    if (!timeout) i2c_reset();
}
