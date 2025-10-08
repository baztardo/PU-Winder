// =============================================================================
// lcd_display.cpp - I2C LCD Display Implementation
// =============================================================================

#include "lcd_display.h"
#include "pico/stdlib.h"
#include <cstring>
#include <cstdarg>
#include <cstdio>

LCDDisplay::LCDDisplay(i2c_inst_t* i2c_inst, uint8_t addr, uint8_t cols, uint8_t rows)
    : i2c(i2c_inst)
    , i2c_addr(addr)
    , num_cols(cols)
    , num_rows(rows)
    , backlight_val(LCD_BACKLIGHT)
    , display_control(0)
    , display_mode(0) {
}

bool LCDDisplay::init() {
    sleep_ms(50);  // Wait for LCD to power up
    
    // Initialize to 4-bit mode
    write_4bits(0x03 << 4);
    sleep_ms(5);
    write_4bits(0x03 << 4);
    sleep_ms(5);
    write_4bits(0x03 << 4);
    sleep_us(150);
    write_4bits(0x02 << 4);  // Set to 4-bit mode
    
    // Function set: 4-bit mode, 2 lines, 5x8 dots
    send_command(LCD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE | LCD_5x8_DOTS);
    
    // Display control: display on, cursor off, blink off
    display_control = LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF;
    send_command(LCD_DISPLAY_CONTROL | display_control);
    
    // Clear display
    clear();
    
    // Entry mode: left to right, no shift
    display_mode = LCD_ENTRY_LEFT | LCD_ENTRY_SHIFT_DECREMENT;
    send_command(LCD_ENTRY_MODE_SET | display_mode);
    
    home();
    
    return true;
}

void LCDDisplay::clear() {
    send_command(LCD_CLEAR_DISPLAY);
    sleep_ms(2);
}

void LCDDisplay::home() {
    send_command(LCD_RETURN_HOME);
    sleep_ms(2);
}

void LCDDisplay::set_cursor(uint8_t col, uint8_t row) {
    // Row offsets for 20x4 LCD
    static const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    
    if (row >= num_rows) row = num_rows - 1;
    if (col >= num_cols) col = num_cols - 1;
    
    send_command(LCD_SET_DDRAM_ADDR | (col + row_offsets[row]));
}

void LCDDisplay::print(const char* str) {
    while (*str) {
        send_data(*str++);
    }
}

void LCDDisplay::print_at(uint8_t col, uint8_t row, const char* str) {
    set_cursor(col, row);
    
    // Clear the rest of the line and print
    char buffer[21];
    snprintf(buffer, sizeof(buffer), "%-20s", str);
    buffer[num_cols] = '\0';
    print(buffer);
}

void LCDDisplay::printf_at(uint8_t col, uint8_t row, const char* format, ...) {
    char buffer[21];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    print_at(col, row, buffer);
}

void LCDDisplay::display(bool on) {
    if (on) {
        display_control |= LCD_DISPLAY_ON;
    } else {
        display_control &= ~LCD_DISPLAY_ON;
    }
    send_command(LCD_DISPLAY_CONTROL | display_control);
}

void LCDDisplay::cursor(bool on) {
    if (on) {
        display_control |= LCD_CURSOR_ON;
    } else {
        display_control &= ~LCD_CURSOR_ON;
    }
    send_command(LCD_DISPLAY_CONTROL | display_control);
}

void LCDDisplay::blink(bool on) {
    if (on) {
        display_control |= LCD_BLINK_ON;
    } else {
        display_control &= ~LCD_BLINK_ON;
    }
    send_command(LCD_DISPLAY_CONTROL | display_control);
}

void LCDDisplay::backlight(bool on) {
    backlight_val = on ? LCD_BACKLIGHT : LCD_NO_BACKLIGHT;
    expander_write(0);
}

void LCDDisplay::create_char(uint8_t location, const uint8_t charmap[8]) {
    location &= 0x7;  // Only 8 locations (0-7)
    send_command(LCD_SET_CGRAM_ADDR | (location << 3));
    for (int i = 0; i < 8; i++) {
        send_data(charmap[i]);
    }
}

void LCDDisplay::send_command(uint8_t cmd) {
    uint8_t high = cmd & 0xF0;
    uint8_t low = (cmd << 4) & 0xF0;
    write_4bits(high);
    write_4bits(low);
}

void LCDDisplay::send_data(uint8_t data) {
    uint8_t high = data & 0xF0;
    uint8_t low = (data << 4) & 0xF0;
    write_4bits(high | 0x01);  // RS=1 for data
    write_4bits(low | 0x01);
}

void LCDDisplay::write_4bits(uint8_t value) {
    expander_write(value);
    pulse_enable(value);
}

void LCDDisplay::expander_write(uint8_t data) {
    uint8_t byte = data | backlight_val;
    i2c_write_blocking(i2c, i2c_addr, &byte, 1, false);
}

void LCDDisplay::pulse_enable(uint8_t data) {
    expander_write(data | ENABLE_BIT);
    sleep_us(1);
    expander_write(data & ~ENABLE_BIT);
    sleep_us(50);
}