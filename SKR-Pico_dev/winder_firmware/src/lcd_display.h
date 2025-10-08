// =============================================================================
// lcd_display.h - I2C LCD Display Driver (HD44780 compatible)
// Purpose: 4x20 I2C LCD for status display and debugging
// =============================================================================

#pragma once

#include "hardware/i2c.h"
#include <cstdint>
#include <string>

// =============================================================================
// LCD Commands
// =============================================================================
#define LCD_CLEAR_DISPLAY   0x01
#define LCD_RETURN_HOME     0x02
#define LCD_ENTRY_MODE_SET  0x04
#define LCD_DISPLAY_CONTROL 0x08
#define LCD_CURSOR_SHIFT    0x10
#define LCD_FUNCTION_SET    0x20
#define LCD_SET_CGRAM_ADDR  0x40
#define LCD_SET_DDRAM_ADDR  0x80

// Flags for display entry mode
#define LCD_ENTRY_RIGHT          0x00
#define LCD_ENTRY_LEFT           0x02
#define LCD_ENTRY_SHIFT_INCREMENT 0x01
#define LCD_ENTRY_SHIFT_DECREMENT 0x00

// Flags for display on/off control
#define LCD_DISPLAY_ON  0x04
#define LCD_DISPLAY_OFF 0x00
#define LCD_CURSOR_ON   0x02
#define LCD_CURSOR_OFF  0x00
#define LCD_BLINK_ON    0x01
#define LCD_BLINK_OFF   0x00

// Flags for function set
#define LCD_8BIT_MODE 0x10
#define LCD_4BIT_MODE 0x00
#define LCD_2LINE     0x08
#define LCD_1LINE     0x00
#define LCD_5x10_DOTS 0x04
#define LCD_5x8_DOTS  0x00

// Backlight control
#define LCD_BACKLIGHT   0x08
#define LCD_NO_BACKLIGHT 0x00

#define ENABLE_BIT 0x04  // Enable bit

// =============================================================================
// LCDDisplay Class
// =============================================================================
class LCDDisplay {
public:
    /**
     * @brief Constructor
     * @param i2c_inst I2C instance (i2c0 or i2c1)
     * @param addr I2C address (typically 0x27 or 0x3F)
     * @param cols Number of columns (default 20)
     * @param rows Number of rows (default 4)
     */
    LCDDisplay(i2c_inst_t* i2c_inst, uint8_t addr = 0x27, 
               uint8_t cols = 20, uint8_t rows = 4);
    
    /**
     * @brief Initialize the LCD display
     * @return true if successful
     */
    bool init();
    
    /**
     * @brief Clear the display
     */
    void clear();
    
    /**
     * @brief Move cursor to home position (0, 0)
     */
    void home();
    
    /**
     * @brief Set cursor position
     * @param col Column (0-19 for 20 char display)
     * @param row Row (0-3 for 4 line display)
     */
    void set_cursor(uint8_t col, uint8_t row);
    
    /**
     * @brief Print a string at current cursor position
     * @param str String to print
     */
    void print(const char* str);
    
    /**
     * @brief Print a string at specific position
     * @param col Column
     * @param row Row
     * @param str String to print
     */
    void print_at(uint8_t col, uint8_t row, const char* str);
    
    /**
     * @brief Print formatted text (printf style)
     * @param col Column
     * @param row Row
     * @param format Format string
     * @param ... Arguments
     */
    void printf_at(uint8_t col, uint8_t row, const char* format, ...);
    
    /**
     * @brief Turn display on/off
     * @param on true to turn on, false to turn off
     */
    void display(bool on);
    
    /**
     * @brief Turn cursor on/off
     * @param on true to show cursor
     */
    void cursor(bool on);
    
    /**
     * @brief Turn cursor blinking on/off
     * @param on true to blink cursor
     */
    void blink(bool on);
    
    /**
     * @brief Turn backlight on/off
     * @param on true to turn on backlight
     */
    void backlight(bool on);
    
    /**
     * @brief Create custom character
     * @param location Character location (0-7)
     * @param charmap 8-byte array defining character pattern
     */
    void create_char(uint8_t location, const uint8_t charmap[8]);

private:
    i2c_inst_t* i2c;
    uint8_t i2c_addr;
    uint8_t num_cols;
    uint8_t num_rows;
    uint8_t backlight_val;
    uint8_t display_control;
    uint8_t display_mode;
    
    /**
     * @brief Send command to LCD
     * @param cmd Command byte
     */
    void send_command(uint8_t cmd);
    
    /**
     * @brief Send data to LCD
     * @param data Data byte
     */
    void send_data(uint8_t data);
    
    /**
     * @brief Write 4 bits to LCD
     * @param value 4-bit value
     */
    void write_4bits(uint8_t value);
    
    /**
     * @brief Expand I2C expander byte and send
     * @param data Data to send
     */
    void expander_write(uint8_t data);
    
    /**
     * @brief Pulse enable pin
     * @param data Data with enable bit
     */
    void pulse_enable(uint8_t data);
};