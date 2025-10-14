#include "lcd_pcf8574.h"
#include "i2c_helper.h"
#include <stdio.h>
#include <string.h>

void lcd_expander_write(lcd_t *lcd, uint8_t data) {
    uint8_t buffer = data | lcd->backlight_val;
    i2c_helper_write_byte(lcd->i2c_port, lcd->address, buffer);
}

void lcd_pulse_enable(lcd_t *lcd, uint8_t data) {
    lcd_expander_write(lcd, data | LCD_EN);
    sleep_us(1);
    lcd_expander_write(lcd, data & ~LCD_EN);
    sleep_us(50);
}

void lcd_write_nibble(lcd_t *lcd, uint8_t nibble, uint8_t mode) {
    uint8_t data = (nibble & 0xF0) | mode;
    lcd_expander_write(lcd, data);
    lcd_pulse_enable(lcd, data);
}

void lcd_send(lcd_t *lcd, uint8_t value, uint8_t mode) {
    uint8_t high_nibble = value & 0xF0;
    uint8_t low_nibble = (value << 4) & 0xF0;
    
    lcd_write_nibble(lcd, high_nibble, mode);
    lcd_write_nibble(lcd, low_nibble, mode);
}

void lcd_command(lcd_t *lcd, uint8_t cmd) {
    lcd_send(lcd, cmd, 0);
}

void lcd_init(lcd_t *lcd, i2c_inst_t *i2c_port, uint8_t address, uint8_t cols, uint8_t rows) {
    lcd->i2c_port = i2c_port;
    lcd->address = address;
    lcd->cols = cols;
    lcd->rows = rows;
    lcd->backlight_val = LCD_BACKLIGHT;
    
    // Wait for LCD to power up
    sleep_ms(50);
    
    // Check if device is present
    if (!i2c_helper_device_present(i2c_port, address)) {
        printf("WARNING: LCD not found at address 0x%02X\n", address);
    }
    
    lcd_expander_write(lcd, lcd->backlight_val);
    sleep_ms(1000);
    
    // Initialize LCD in 4-bit mode
    lcd_write_nibble(lcd, 0x30, 0);
    sleep_ms(5);
    
    lcd_write_nibble(lcd, 0x30, 0);
    sleep_us(150);
    
    lcd_write_nibble(lcd, 0x30, 0);
    sleep_us(150);
    
    lcd_write_nibble(lcd, 0x20, 0);  // Set to 4-bit mode
    sleep_us(150);
    
    // Configure LCD
    lcd_command(lcd, LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
    lcd_command(lcd, LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
    lcd_clear(lcd);
    lcd_command(lcd, LCD_ENTRYMODESET | LCD_ENTRYLEFT);
    lcd_home(lcd);
}

void lcd_clear(lcd_t *lcd) {
    lcd_command(lcd, LCD_CLEARDISPLAY);
    sleep_ms(2);
}

void lcd_home(lcd_t *lcd) {
    lcd_command(lcd, LCD_RETURNHOME);
    sleep_ms(2);
}

void lcd_set_cursor(lcd_t *lcd, uint8_t col, uint8_t row) {
    uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
    if (row >= lcd->rows) {
        row = lcd->rows - 1;
    }
    lcd_command(lcd, LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void lcd_print(lcd_t *lcd, const char *str) {
    while (*str) {
        lcd_send(lcd, *str++, LCD_RS);
    }
}

void lcd_print_int(lcd_t *lcd, int32_t value) {
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%ld", value);
    lcd_print(lcd, buffer);
}

void lcd_print_float(lcd_t *lcd, float value, uint8_t decimals) {
    char buffer[16];
    char format[8];
    snprintf(format, sizeof(format), "%%.%df", decimals);
    snprintf(buffer, sizeof(buffer), format, value);
    lcd_print(lcd, buffer);
}

void lcd_backlight_on(lcd_t *lcd) {
    lcd->backlight_val = LCD_BACKLIGHT;
    lcd_expander_write(lcd, 0);
}

void lcd_backlight_off(lcd_t *lcd) {
    lcd->backlight_val = LCD_NOBACKLIGHT;
    lcd_expander_write(lcd, 0);
}
