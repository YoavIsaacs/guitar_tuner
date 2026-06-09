/*
 * lcd.h — HD44780 LCD driver via PCF8574 I2C backpack
 *
 * Hardware assumptions:
 *   - 16x2 HD44780-compatible LCD with PCF8574 I2C backpack (address 0x3F)
 *   - PCF8574 pin mapping: P7-P4 = DB7-DB4, P3 = backlight, P2 = EN, P1 = RW, P0 = RS
 *   - I2C1 must be initialised before calling lcd_init()
 */

#ifndef LCD_H
#define LCD_H

#include <stdint.h>

#define LCD_ADDR  0x3F

/* Initialise the LCD. Must be called once after I2C is ready. */
void lcd_init(void);

/* Print a null-terminated string at the current cursor position.
 * Does not clear the display — call lcd_clear() explicitly when needed. */
void lcd_print(const char *str);

/* Print a single character at the current cursor position.
 * Custom glyph codes 1-7 and the built-in solid block (0xFF) are valid. */
void lcd_print_char(char c);

/* Move the cursor. row: 0 or 1. col: 0-based. */
void lcd_set_cursor(uint8_t row, uint8_t col);

/* Clear the display and return the cursor to home. */
void lcd_clear(void);

/*
 * Load a 5x8 custom glyph into CGRAM slot 'location' (0-7).
 * pattern: 8 bytes, low 5 bits of each byte are the pixel rows (top to bottom).
 * After loading, call lcd_set_cursor() before printing — the address pointer
 * is left in CGRAM space.
 * Avoid slot 0: the glyph code (0x00) terminates C strings.
 */
void lcd_create_char(uint8_t location, const uint8_t *pattern);

#endif /* LCD_H */
