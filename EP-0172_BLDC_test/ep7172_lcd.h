// =============================================================================
// ep7172_lcd.h - Simple LCD display driver for EP-0172
// Uses SPI for TFT display communication
// =============================================================================

#pragma once

#include <cstdint>
#include "hardware/spi.h"

class EP7172Display {
public:
    EP7172Display();
    
    void init();
    void clear(uint16_t color = 0x0000);  // Black
    void print_at(uint16_t x, uint16_t y, const char* text, uint16_t fg_color = 0xFFFF, uint16_t bg_color = 0x0000);
    void printf_at(uint16_t x, uint16_t y, uint16_t fg_color, const char* format, ...);
    
    void draw_rectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
    void draw_filled_rectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
    void draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
    
    // Color definitions (RGB565)
    static constexpr uint16_t COLOR_BLACK   = 0x0000;
    static constexpr uint16_t COLOR_WHITE   = 0xFFFF;
    static constexpr uint16_t COLOR_RED     = 0xF800;
    static constexpr uint16_t COLOR_GREEN   = 0x07E0;
    static constexpr uint16_t COLOR_BLUE    = 0x001F;
    static constexpr uint16_t COLOR_YELLOW  = 0xFFE0;
    static constexpr uint16_t COLOR_CYAN    = 0x07FF;
    static constexpr uint16_t COLOR_MAGENTA = 0xF81F;

private:
    uint16_t width;
    uint16_t height;
    
    void write_command(uint8_t cmd);
    void write_data(uint8_t data);
    void write_data_multi(const uint8_t* data, size_t len);
    void set_address_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
    void write_pixel(uint16_t color);
};