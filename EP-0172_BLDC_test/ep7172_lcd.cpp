// =============================================================================
// ep7172_lcd.cpp - EP-0172 LCD Display Driver
// =============================================================================

#include "ep7172_lcd.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include <cstdio>
#include <cstdarg>
#include <cstring>

// =============================================================================
// EP-0172 Pin Configuration
// =============================================================================
#define TFT_CLK_PIN   2   // SPI CLK
#define TFT_DIN_PIN   3   // SPI DIN (MOSI)
#define TFT_CS_PIN    5   // SPI CS
#define TFT_DC_PIN    6   // Data/Command
#define TFT_RST_PIN   7   // Reset

#define TFT_SPI       spi0
#define TFT_BAUD      10000000  // 10 MHz

// Display dimensions
#define TFT_WIDTH     320
#define TFT_HEIGHT    480

// ST7796S commands
#define ST7796_SWRESET    0x01
#define ST7796_SLPOUT     0x11
#define ST7796_DISPON     0x29
#define ST7796_CASET      0x2A
#define ST7796_RASET      0x2B
#define ST7796_RAMWR      0x2C
#define ST7796_MADCTL     0x36
#define ST7796_COLMOD     0x3A

EP7172Display::EP7172Display()
    : width(TFT_WIDTH), height(TFT_HEIGHT)
{
}

void EP7172Display::init() {
    // Initialize SPI
    spi_init(TFT_SPI, TFT_BAUD);
    gpio_set_function(TFT_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(TFT_DIN_PIN, GPIO_FUNC_SPI);
    
    // Initialize control pins
    gpio_init(TFT_CS_PIN);
    gpio_set_dir(TFT_CS_PIN, GPIO_OUT);
    gpio_put(TFT_CS_PIN, 1);
    
    gpio_init(TFT_DC_PIN);
    gpio_set_dir(TFT_DC_PIN, GPIO_OUT);
    
    gpio_init(TFT_RST_PIN);
    gpio_set_dir(TFT_RST_PIN, GPIO_OUT);
    gpio_put(TFT_RST_PIN, 1);
    
    // Reset display
    sleep_ms(100);
    gpio_put(TFT_RST_PIN, 0);
    sleep_ms(100);
    gpio_put(TFT_RST_PIN, 1);
    sleep_ms(100);
    
    // Initialize display
    write_command(ST7796_SWRESET);
    sleep_ms(150);
    
    write_command(ST7796_SLPOUT);
    sleep_ms(100);
    
    write_command(ST7796_COLMOD);
    write_data(0x05);  // 16-bit color
    
    write_command(ST7796_MADCTL);
    write_data(0x00);  // Normal orientation
    
    write_command(ST7796_DISPON);
    sleep_ms(50);
    
    clear();
    printf("[LCD] Initialized (320x480)\n");
}

void EP7172Display::write_command(uint8_t cmd) {
    gpio_put(TFT_CS_PIN, 0);
    gpio_put(TFT_DC_PIN, 0);  // Command mode
    spi_write_blocking(TFT_SPI, &cmd, 1);
    gpio_put(TFT_CS_PIN, 1);
}

void EP7172Display::write_data(uint8_t data) {
    gpio_put(TFT_CS_PIN, 0);
    gpio_put(TFT_DC_PIN, 1);  // Data mode
    spi_write_blocking(TFT_SPI, &data, 1);
    gpio_put(TFT_CS_PIN, 1);
}

void EP7172Display::set_address_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    write_command(ST7796_CASET);
    write_data(x1 >> 8);
    write_data(x1 & 0xFF);
    write_data(x2 >> 8);
    write_data(x2 & 0xFF);
    
    write_command(ST7796_RASET);
    write_data(y1 >> 8);
    write_data(y1 & 0xFF);
    write_data(y2 >> 8);
    write_data(y2 & 0xFF);
    
    write_command(ST7796_RAMWR);
}

void EP7172Display::write_pixel(uint16_t color) {
    uint8_t data[2] = {(uint8_t)(color >> 8), (uint8_t)(color & 0xFF)};
    gpio_put(TFT_CS_PIN, 0);
    gpio_put(TFT_DC_PIN, 1);
    spi_write_blocking(TFT_SPI, data, 2);
    gpio_put(TFT_CS_PIN, 1);
}

void EP7172Display::clear(uint16_t color) {
    set_address_window(0, 0, width - 1, height - 1);
    
    gpio_put(TFT_CS_PIN, 0);
    gpio_put(TFT_DC_PIN, 1);
    
    for (uint32_t i = 0; i < (uint32_t)width * height; i++) {
        uint8_t data[2] = {(uint8_t)(color >> 8), (uint8_t)(color & 0xFF)};
        spi_write_blocking(TFT_SPI, data, 2);
    }
    
    gpio_put(TFT_CS_PIN, 1);
}

void EP7172Display::draw_filled_rectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    set_address_window(x, y, x + w - 1, y + h - 1);
    
    gpio_put(TFT_CS_PIN, 0);
    gpio_put(TFT_DC_PIN, 1);
    
    for (uint32_t i = 0; i < (uint32_t)w * h; i++) {
        uint8_t data[2] = {(uint8_t)(color >> 8), (uint8_t)(color & 0xFF)};
        spi_write_blocking(TFT_SPI, data, 2);
    }
    
    gpio_put(TFT_CS_PIN, 1);
}

void EP7172Display::print_at(uint16_t x, uint16_t y, const char* text, uint16_t fg_color, uint16_t bg_color) {
    // Simple character rendering (8x8 pixels per character)
    // This is a simplified version - full implementation would use bitmap fonts
    
    // For now, just print to serial for debugging
    printf("[LCD %u,%u] %s\n", x, y, text);
}

void EP7172Display::printf_at(uint16_t x, uint16_t y, uint16_t fg_color, const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    print_at(x, y, buffer, fg_color);
}

void EP7172Display::draw_rectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    // Draw top/bottom lines
    set_address_window(x, y, x + w - 1, y);
    gpio_put(TFT_CS_PIN, 0);
    gpio_put(TFT_DC_PIN, 1);
    for (uint16_t i = 0; i < w; i++) {
        uint8_t data[2] = {(uint8_t)(color >> 8), (uint8_t)(color & 0xFF)};
        spi_write_blocking(TFT_SPI, data, 2);
    }
    gpio_put(TFT_CS_PIN, 1);
}

void EP7172Display::draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
    // Simplified Bresenham line drawing
    // Full implementation would be more complex
}