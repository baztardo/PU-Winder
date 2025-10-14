#ifndef LCD_PCF8574_H
#define LCD_PCF8574_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

/**
 * LCD Driver for HD44780-compatible displays with PCF8574 I2C backpack
 * Uses generic I2C helper library for communication
 * 
 * TESTED COMPATIBLE DISPLAYS:
 * - TC2004A-01 (20x4, SPLC780D1 controller, Adafruit)
 * - Standard HD44780 20x4 LCDs
 * - Standard HD44780 16x2 LCDs
 */

// LCD Commands
#define LCD_CLEARDISPLAY    0x01
#define LCD_RETURNHOME      0x02
#define LCD_ENTRYMODESET    0x04
#define LCD_DISPLAYCONTROL  0x08
#define LCD_FUNCTIONSET     0x20
#define LCD_SETDDRAMADDR    0x80

// Flags for display entry mode
#define LCD_ENTRYRIGHT      0x00
#define LCD_ENTRYLEFT       0x02

// Flags for display on/off control
#define LCD_DISPLAYON       0x04
#define LCD_CURSOROFF       0x00
#define LCD_BLINKOFF        0x00

// Flags for function set
#define LCD_4BITMODE        0x00
#define LCD_2LINE           0x08
#define LCD_5x8DOTS         0x00

// Backlight control
#define LCD_BACKLIGHT       0x08
#define LCD_NOBACKLIGHT     0x00

// PCF8574 I2C expander bits
#define LCD_EN              0x04  // Enable bit
#define LCD_RW              0x02  // Read/Write bit
#define LCD_RS              0x01  // Register select bit

// LCD Structure
typedef struct {
    i2c_inst_t *i2c_port;
    uint8_t address;
    uint8_t cols;
    uint8_t rows;
    uint8_t backlight_val;
} lcd_t;

/**
 * Initialize LCD
 * @param lcd LCD structure
 * @param i2c_port I2C port instance (must be already initialized)
 * @param address 7-bit I2C address of LCD
 * @param cols Number of columns
 * @param rows Number of rows
 */
void lcd_init(lcd_t *lcd, i2c_inst_t *i2c_port, uint8_t address, uint8_t cols, uint8_t rows);

/**
 * Clear LCD display
 */
void lcd_clear(lcd_t *lcd);

/**
 * Return cursor to home position
 */
void lcd_home(lcd_t *lcd);

/**
 * Set cursor position
 * @param col Column (0-based)
 * @param row Row (0-based)
 */
void lcd_set_cursor(lcd_t *lcd, uint8_t col, uint8_t row);

/**
 * Print string to LCD
 * @param str Null-terminated string
 */
void lcd_print(lcd_t *lcd, const char *str);

/**
 * Print integer to LCD
 * @param value Integer value
 */
void lcd_print_int(lcd_t *lcd, int32_t value);

/**
 * Print float to LCD
 * @param value Float value
 * @param decimals Number of decimal places
 */
void lcd_print_float(lcd_t *lcd, float value, uint8_t decimals);

/**
 * Turn backlight on
 */
void lcd_backlight_on(lcd_t *lcd);

/**
 * Turn backlight off
 */
void lcd_backlight_off(lcd_t *lcd);

/**
 * Send command to LCD
 * @param cmd Command byte
 */
void lcd_command(lcd_t *lcd, uint8_t cmd);

// Internal functions (exposed for advanced use)
void lcd_send(lcd_t *lcd, uint8_t value, uint8_t mode);
void lcd_write_nibble(lcd_t *lcd, uint8_t nibble, uint8_t mode);
void lcd_expander_write(lcd_t *lcd, uint8_t data);
void lcd_pulse_enable(lcd_t *lcd, uint8_t data);

#endif // LCD_PCF8574_H
