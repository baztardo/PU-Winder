// =============================================================================
// ep7172_lcd.cpp - ST7796S Driver for Pico SDK (FROM ADAFRUIT LIBRARY)
// =============================================================================

#include "ep7172_lcd.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include <cstdio>
#include <cstdarg>
#include <cstring>

// =============================================================================
// Pin Configuration
// =============================================================================
#define TFT_CLK_PIN   2
#define TFT_DIN_PIN   3
#define TFT_MISO_PIN  4
#define TFT_CS_PIN    5
#define TFT_DC_PIN    6
#define TFT_RST_PIN   7

#define TFT_SPI       spi0
#define TFT_BAUD      10000000  // 10 MHz

#define TFT_WIDTH     320
#define TFT_HEIGHT    480

// ST7796S Commands
#define ST77XX_SWRESET   0x01
#define ST77XX_SLPOUT    0x11
#define ST77XX_DISPON    0x29
#define ST77XX_CASET     0x2A
#define ST77XX_RASET     0x2B
#define ST77XX_RAMWR     0x2C
#define ST77XX_MADCTL    0x36
#define ST77XX_COLMOD    0x3A

EP7172Display::EP7172Display()
    : width(TFT_WIDTH), height(TFT_HEIGHT)
{
}

void EP7172Display::init() {
    printf("[LCD] Initializing ST7796S with Adafruit sequence...\n");
    
    // Initialize SPI0
    printf("[LCD] Configuring SPI...\n");
    spi_init(TFT_SPI, TFT_BAUD);
    gpio_set_function(TFT_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(TFT_DIN_PIN, GPIO_FUNC_SPI);
    gpio_set_function(TFT_MISO_PIN, GPIO_FUNC_SPI);
    
    // GPIO for control pins
    gpio_init(TFT_CS_PIN);
    gpio_set_dir(TFT_CS_PIN, GPIO_OUT);
    gpio_put(TFT_CS_PIN, 1);
    
    gpio_init(TFT_DC_PIN);
    gpio_set_dir(TFT_DC_PIN, GPIO_OUT);
    
    gpio_init(TFT_RST_PIN);
    gpio_set_dir(TFT_RST_PIN, GPIO_OUT);
    gpio_put(TFT_RST_PIN, 1);
    
    // Reset sequence
    printf("[LCD] Reset...\n");
    sleep_ms(100);
    gpio_put(TFT_RST_PIN, 0);
    sleep_ms(100);
    gpio_put(TFT_RST_PIN, 1);
    sleep_ms(150);
    
    // ADAFRUIT INITIALIZATION SEQUENCE
    printf("[LCD] Running Adafruit init sequence...\n");
    
    // 1. Software Reset
    write_command(ST77XX_SWRESET);
    sleep_ms(150);
    
    // 2. Unlock manufacturer commands
    write_command(0xF0);
    write_data(0xC3);
    
    write_command(0xF0);
    write_data(0x96);
    
    // 3. VCOM Control
    write_command(0xC5);
    write_data(0x1C);
    
    // 4. Memory Access Control (MADCTL) - CRITICAL!
    write_command(ST77XX_MADCTL);
    write_data(0x48);  // Important: NOT 0x00!
    
    // 5. Color Mode - 16-bit
    write_command(ST77XX_COLMOD);
    write_data(0x55);  // Important: 0x55 not 0x05!
    
    // 6. Interface Control
    write_command(0xB0);
    write_data(0x80);
    
    // 7. Inversion Control
    write_command(0xB4);
    write_data(0x00);
    
    // 8. Display Function Control
    write_command(0xB6);
    write_data(0x80);
    write_data(0x02);
    write_data(0x3B);
    
    // 9. Entry Mode
    write_command(0xB7);
    write_data(0xC6);
    
    // 10. Lock manufacturer commands
    write_command(0xF0);
    write_data(0x69);
    
    write_command(0xF0);
    write_data(0x3C);
    
    // 11. Sleep Out
    write_command(ST77XX_SLPOUT);
    sleep_ms(150);
    
    // 12. Display ON
    write_command(ST77XX_DISPON);
    sleep_ms(150);
    
    printf("[LCD] Init complete!\n");
    
    // Clear screen to black
    clear(0x0000);
}

void EP7172Display::write_command(uint8_t cmd) {
    gpio_put(TFT_CS_PIN, 0);
    gpio_put(TFT_DC_PIN, 0);  // Command
    spi_write_blocking(TFT_SPI, &cmd, 1);
    gpio_put(TFT_CS_PIN, 1);
}

void EP7172Display::write_data(uint8_t data) {
    gpio_put(TFT_CS_PIN, 0);
    gpio_put(TFT_DC_PIN, 1);  // Data
    spi_write_blocking(TFT_SPI, &data, 1);
    gpio_put(TFT_CS_PIN, 1);
}

void EP7172Display::write_data_multi(const uint8_t* data, size_t len) {
    gpio_put(TFT_CS_PIN, 0);
    gpio_put(TFT_DC_PIN, 1);  // Data
    spi_write_blocking(TFT_SPI, (uint8_t*)data, len);
    gpio_put(TFT_CS_PIN, 1);
}

void EP7172Display::set_address_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    // Column address
    write_command(ST77XX_CASET);
    write_data(x1 >> 8);
    write_data(x1 & 0xFF);
    write_data(x2 >> 8);
    write_data(x2 & 0xFF);
    
    // Row address
    write_command(ST77XX_RASET);
    write_data(y1 >> 8);
    write_data(y1 & 0xFF);
    write_data(y2 >> 8);
    write_data(y2 & 0xFF);
    
    // RAM write - keep CS low for pixels
    gpio_put(TFT_CS_PIN, 0);
    gpio_put(TFT_DC_PIN, 0);  // Command
    uint8_t cmd = ST77XX_RAMWR;
    spi_write_blocking(TFT_SPI, &cmd, 1);
    gpio_put(TFT_DC_PIN, 1);  // Data mode
}

void EP7172Display::write_pixel(uint16_t color) {
    uint8_t data[2] = {(uint8_t)(color >> 8), (uint8_t)(color & 0xFF)};
    spi_write_blocking(TFT_SPI, data, 2);
}

void EP7172Display::clear(uint16_t color) {
    set_address_window(0, 0, width - 1, height - 1);
    
    uint32_t pixel_count = (uint32_t)width * height;
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;
    
    for (uint32_t i = 0; i < pixel_count; i++) {
        spi_write_blocking(TFT_SPI, &hi, 1);
        spi_write_blocking(TFT_SPI, &lo, 1);
    }
    
    gpio_put(TFT_CS_PIN, 1);
    printf("[LCD] Screen cleared\n");
}

void EP7172Display::draw_filled_rectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    set_address_window(x, y, x + w - 1, y + h - 1);
    
    uint32_t pixel_count = (uint32_t)w * h;
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;
    
    for (uint32_t i = 0; i < pixel_count; i++) {
        spi_write_blocking(TFT_SPI, &hi, 1);
        spi_write_blocking(TFT_SPI, &lo, 1);
    }
    
    gpio_put(TFT_CS_PIN, 1);
}

void EP7172Display::print_at(uint16_t x, uint16_t y, const char* text, uint16_t fg_color, uint16_t bg_color) {
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
    set_address_window(x, y, x + w - 1, y);
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;
    for (uint16_t i = 0; i < w; i++) {
        spi_write_blocking(TFT_SPI, &hi, 1);
        spi_write_blocking(TFT_SPI, &lo, 1);
    }
    gpio_put(TFT_CS_PIN, 1);
}

void EP7172Display::draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
    // Simplified line drawing
}