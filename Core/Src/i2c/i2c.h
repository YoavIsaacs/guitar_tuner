/*
 * i2c.h — Bare-metal I2C1 driver for STM32L432KC
 *
 * Covers single-byte blocking transmit on PB6 (SCL) / PB7 (SDA), AF4,
 * open-drain, with NACK detection and timeout-based recovery.
 *
 * The caller is responsible for peripheral initialisation before first use:
 *   1. Enable GPIOB and I2C1 clocks in RCC.
 *   2. Configure PB6/PB7 as AF4, open-drain (see main.c).
 *   3. Write I2C1_TIMINGR = 0x10909CEC  (100 kHz @ 32 MHz PCLK1).
 *   4. Set I2C1_CR1 bit 0 (PE).
 */

#ifndef I2C_H
#define I2C_H

#include <stdint.h>

/* I2C1 register map (RM0394 §37.7) */
#define I2C1_BASE    0x40005400UL
#define I2C1_CR1     (*(volatile uint32_t *)(I2C1_BASE + 0x00))
#define I2C1_CR2     (*(volatile uint32_t *)(I2C1_BASE + 0x04))
#define I2C1_TIMINGR (*(volatile uint32_t *)(I2C1_BASE + 0x10))
#define I2C1_ISR     (*(volatile uint32_t *)(I2C1_BASE + 0x18))
#define I2C1_ICR     (*(volatile uint32_t *)(I2C1_BASE + 0x1C))
#define I2C1_TXDR    (*(volatile uint32_t *)(I2C1_BASE + 0x28))

/*
 * Send a single byte to a 7-bit I2C device.
 * Blocks until transmission completes, a NACK is received, or a timeout
 * expires. On failure the peripheral is reset automatically.
 */
void i2c_send_byte(uint8_t data, uint8_t address);

#endif /* I2C_H */
