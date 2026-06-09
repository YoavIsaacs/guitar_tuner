/*
 * lcd.c — HD44780 LCD driver via PCF8574 I2C backpack.
 */

#include "lcd.h"
#include "../i2c/i2c.h"

/*
 * Send one 4-bit nibble to the LCD.
 *
 * PCF8574 bit mapping:
 *   P7 P6 P5 P4 = DB7 DB6 DB5 DB4
 *   P3          = backlight (always on via 0x08)
 *   P2          = EN
 *   P1          = RW (always 0, write-only)
 *   P0          = RS
 */
static void lcd_send_nibble(uint8_t nibble, uint8_t rs)
{
    uint8_t base = (nibble & 0xF0u) | 0x08u | (rs & 0x01u);
    i2c_send_byte(base | 0x04u, LCD_ADDR);   /* EN high */
    for (volatile int i = 0; i < 2000; i++);
    i2c_send_byte(base & ~0x04u, LCD_ADDR);  /* EN low  */
    for (volatile int i = 0; i < 2000; i++);
}

static void lcd_send_byte(uint8_t data, uint8_t rs)
{
    lcd_send_nibble( data & 0xF0u,        rs);
    lcd_send_nibble((data << 4) & 0xF0u,  rs);
}

void lcd_init(void)
{
    for (volatile int i = 0; i < 3000000; i++);   /* >40 ms power-on delay */

    /* HD44780 initialisation-by-instruction sequence (datasheet §4.4) */
    lcd_send_nibble(0x30, 0); for (volatile int i = 0; i < 300000; i++);
    lcd_send_nibble(0x30, 0); for (volatile int i = 0; i < 50000; i++);
    lcd_send_nibble(0x30, 0); for (volatile int i = 0; i < 50000; i++);
    lcd_send_nibble(0x20, 0); for (volatile int i = 0; i < 50000; i++);   /* switch to 4-bit */

    lcd_send_byte(0x28, 0); for (volatile int i = 0; i < 50000; i++);    /* 4-bit, 2 lines, 5x8 */
    lcd_send_byte(0x08, 0); for (volatile int i = 0; i < 50000; i++);    /* display off */
    lcd_send_byte(0x01, 0); for (volatile int i = 0; i < 300000; i++);   /* clear */
    lcd_send_byte(0x06, 0); for (volatile int i = 0; i < 50000; i++);    /* entry mode: increment, no shift */
    lcd_send_byte(0x0C, 0); for (volatile int i = 0; i < 50000; i++);    /* display on, cursor off */
}

void lcd_clear(void)
{
    lcd_send_byte(0x01, 0);
    for (volatile int i = 0; i < 300000; i++);
}

void lcd_set_cursor(uint8_t row, uint8_t col)
{
    uint8_t addr = (row == 0 ? 0x00u : 0x40u) + col;
    lcd_send_byte(0x80u | addr, 0);
    for (volatile int i = 0; i < 50000; i++);
}

void lcd_print_char(char c)
{
    lcd_send_byte((uint8_t)c, 1);
}

void lcd_print(const char *str)
{
    for (int i = 0; str[i] != '\0'; i++)
        lcd_print_char(str[i]);
}

void lcd_create_char(uint8_t location, const uint8_t *pattern)
{
    location &= 0x07u;
    lcd_send_byte(0x40u | (location << 3), 0);
    for (int i = 0; i < 8; i++)
        lcd_send_byte(pattern[i] & 0x1Fu, 1);
}
