// =============================================================================
// st7789_pico.cpp - ST7789 TFT Display Driver Implementation
// =============================================================================

#include "st7789_pico.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include <cmath>
#include <cstring>
#include <cstdio>

// =============================================================================
// Constructor
// =============================================================================
ST7789::ST7789(spi_inst_t* spi_inst,
               int sck_pin, int mosi_pin, int miso_pin,
               int cs_pin, int dc_pin, int rst_pin,
               uint16_t w, uint16_t h)
    : spi(spi_inst)
    , sck(sck_pin)
    , mosi(mosi_pin)
    , miso(miso_pin)
    , cs(cs_pin)
    , dc(dc_pin)
    , rst(rst_pin)
    , width(w)
    , height(h)
    , rotation(0) {
}

// =============================================================================
// Initialization
// =============================================================================
bool ST7789::init(uint32_t spi_freq) {
    // Initialize SPI
    spi_init(spi, spi_freq);
    
    // Set GPIO function for SPI pins
    gpio_set_function(sck, GPIO_FUNC_SPI);
    gpio_set_function(mosi, GPIO_FUNC_SPI);
    if (miso >= 0) {
        gpio_set_function(miso, GPIO_FUNC_SPI);
    }
    
    // Initialize control pins (GPIO output)
    gpio_init(cs);
    gpio_set_dir(cs, GPIO_OUT);
    gpio_put(cs, 1);  // Chip select inactive (high)
    
    gpio_init(dc);
    gpio_set_dir(dc, GPIO_OUT);
    gpio_put(dc, 0);
    
    if (rst >= 0) {
        gpio_init(rst);
        gpio_set_dir(rst, GPIO_OUT);
        gpio_put(rst, 1);
    }
    
    // Hardware reset
    if (rst >= 0) {
        reset();
    }
    
    sleep_ms(120);
    
    // Software reset
    write_command(ST7789_CMD_SWRESET);
    sleep_ms(150);
    
    // Exit sleep
    write_command(ST7789_CMD_SLPOUT);
    sleep_ms(10);
    
    // Set color mode to 16-bit RGB565
    write_command(ST7789_CMD_COLMOD);
    write_data_byte(ST7789_COLMOD_16BIT);
    sleep_ms(10);
    
    // Set memory data access control (MADCTL)
    // MV=0, MX=0, MY=0, ML=0, RGB mode
    write_command(ST7789_CMD_MADCTL);
    write_data_byte(0x00);
    
    // Set frame rate
    write_command(ST7789_CMD_FRMCTR1);
    write_data_byte(0x0E);  // 60 Hz
    
    // Display on
    write_command(ST7789_CMD_DISPON);
    sleep_ms(10);
    
    printf("[ST7789] Initialized: %u x %u\n", width, height);
    
    return true;
}

// =============================================================================
// Low-level SPI Communication
// =============================================================================
void ST7789::write_command(uint8_t cmd) {
    gpio_put(cs, 0);        // CS low (select)
    gpio_put(dc, 0);        // DC low (command)
    
    spi_write_blocking(spi, &cmd, 1);
    
    gpio_put(cs, 1);        // CS high (deselect)
}

void ST7789::write_data_byte(uint8_t data) {
    gpio_put(cs, 0);        // CS low
    gpio_put(dc, 1);        // DC high (data)
    
    spi_write_blocking(spi, &data, 1);
    
    gpio_put(cs, 1);        // CS high
}

void ST7789::write_data_word(uint16_t data) {
    uint8_t buf[2] = {
        (uint8_t)(data >> 8),
        (uint8_t)(data & 0xFF)
    };
    
    gpio_put(cs, 0);        // CS low
    gpio_put(dc, 1);        // DC high (data)
    
    spi_write_blocking(spi, buf, 2);
    
    gpio_put(cs, 1);        // CS high
}

void ST7789::write_data(const uint8_t* data, size_t len) {
    gpio_put(cs, 0);        // CS low
    gpio_put(dc, 1);        // DC high (data)
    
    spi_write_blocking(spi, (uint8_t*)data, len);
    
    gpio_put(cs, 1);        // CS high
}

uint8_t ST7789::read_data() {
    uint8_t data = 0;
    
    gpio_put(cs, 0);        // CS low
    gpio_put(dc, 1);        // DC high (data)
    
    spi_read_blocking(spi, 0x00, &data, 1);
    
    gpio_put(cs, 1);        // CS high
    
    return data;
}

// =============================================================================
// Display Control
// =============================================================================
void ST7789::reset() {
    gpio_put(rst, 0);
    sleep_ms(10);
    gpio_put(rst, 1);
    sleep_ms(10);
}

void ST7789::set_address_window(int16_t x1, int16_t y1, int16_t x2, int16_t y2) {
    // Column address set (CASET)
    write_command(ST7789_CMD_CASET);
    uint8_t caset_data[4] = {
        (uint8_t)(x1 >> 8), (uint8_t)x1,
        (uint8_t)(x2 >> 8), (uint8_t)x2
    };
    write_data(caset_data, 4);
    
    // Row address set (RASET)
    write_command(ST7789_CMD_RASET);
    uint8_t raset_data[4] = {
        (uint8_t)(y1 >> 8), (uint8_t)y1,
        (uint8_t)(y2 >> 8), (uint8_t)y2
    };
    write_data(raset_data, 4);
}

void ST7789::fill(uint16_t color) {
    fill_rect(0, 0, width - 1, height - 1, color);
}

void ST7789::fill_rect(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
    if (x1 > x2) { int16_t t = x1; x1 = x2; x2 = t; }
    if (y1 > y2) { int16_t t = y1; y1 = y2; y2 = t; }
    
    if (x1 >= width || y1 >= height || x2 < 0 || y2 < 0) return;
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 >= width) x2 = width - 1;
    if (y2 >= height) y2 = height - 1;
    
    uint16_t w = x2 - x1 + 1;
    uint16_t h = y2 - y1 + 1;
    
    set_address_window(x1, y1, x2, y2);
    
    // Write RAM command
    write_command(ST7789_CMD_RAMWR);
    
    // Fill with color
    uint8_t high = color >> 8;
    uint8_t low = color & 0xFF;
    
    gpio_put(cs, 0);
    gpio_put(dc, 1);
    
    for (uint16_t i = 0; i < (w * h); i++) {
        spi_write_blocking(spi, &high, 1);
        spi_write_blocking(spi, &low, 1);
    }
    
    gpio_put(cs, 1);
}

void ST7789::draw_pixel(int16_t x, int16_t y, uint16_t color) {
    if (x < 0 || x >= width || y < 0 || y >= height) return;
    
    set_address_window(x, y, x, y);
    write_command(ST7789_CMD_RAMWR);
    write_data_word(color);
}

void ST7789::draw_hline(int16_t x, int16_t y, int16_t length, uint16_t color) {
    if (y < 0 || y >= height) return;
    if (x < 0) { length += x; x = 0; }
    if (x + length > width) length = width - x;
    if (length <= 0) return;
    
    set_address_window(x, y, x + length - 1, y);
    write_command(ST7789_CMD_RAMWR);
    
    uint8_t high = color >> 8;
    uint8_t low = color & 0xFF;
    
    gpio_put(cs, 0);
    gpio_put(dc, 1);
    
    for (int16_t i = 0; i < length; i++) {
        spi_write_blocking(spi, &high, 1);
        spi_write_blocking(spi, &low, 1);
    }
    
    gpio_put(cs, 1);
}

void ST7789::draw_vline(int16_t x, int16_t y, int16_t length, uint16_t color) {
    if (x < 0 || x >= width) return;
    if (y < 0) { length += y; y = 0; }
    if (y + length > height) length = height - y;
    if (length <= 0) return;
    
    set_address_window(x, y, x, y + length - 1);
    write_command(ST7789_CMD_RAMWR);
    
    uint8_t high = color >> 8;
    uint8_t low = color & 0xFF;
    
    gpio_put(cs, 0);
    gpio_put(dc, 1);
    
    for (int16_t i = 0; i < length; i++) {
        spi_write_blocking(spi, &high, 1);
        spi_write_blocking(spi, &low, 1);
    }
    
    gpio_put(cs, 1);
}

void ST7789::draw_rect(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
    draw_hline(x1, y1, x2 - x1 + 1, color);
    draw_hline(x1, y2, x2 - x1 + 1, color);
    draw_vline(x1, y1, y2 - y1 + 1, color);
    draw_vline(x2, y1, y2 - y1 + 1, color);
}

void ST7789::draw_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    int16_t x = r;
    int16_t y = 0;
    int16_t d = 3 - 2 * r;
    
    while (x >= y) {
        draw_pixel(x0 + x, y0 + y, color);
        draw_pixel(x0 - x, y0 + y, color);
        draw_pixel(x0 + x, y0 - y, color);
        draw_pixel(x0 - x, y0 - y, color);
        draw_pixel(x0 + y, y0 + x, color);
        draw_pixel(x0 - y, y0 + x, color);
        draw_pixel(x0 + y, y0 - x, color);
        draw_pixel(x0 - y, y0 - x, color);
        
        if (d < 0) {
            d = d + 4 * y + 6;
        } else {
            d = d + 4 * (y - x) + 10;
            x--;
        }
        y++;
    }
}

void ST7789::fill_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    for (int16_t y = -r; y <= r; y++) {
        for (int16_t x = -r; x <= r; x++) {
            if (x * x + y * y <= r * r) {
                draw_pixel(x0 + x, y0 + y, color);
            }
        }
    }
}

void ST7789::display_on(bool on) {
    write_command(on ? ST7789_CMD_DISPON : ST7789_CMD_DISPOFF);
}

void ST7789::invert(bool invert) {
    write_command(invert ? ST7789_CMD_INVON : ST7789_CMD_INVOFF);
}

void ST7789::set_rotation(uint8_t rot) {
    rotation = rot % 4;
    
    uint8_t madctl = 0;
    
    switch (rotation) {
        case 0:  // 0°
            madctl = 0x00;
            width = 240;
            height = 320;
            break;
        case 1:  // 90°
            madctl = ST7789_MADCTL_MV | ST7789_MADCTL_MX;
            width = 320;
            height = 240;
            break;
        case 2:  // 180°
            madctl = ST7789_MADCTL_MY | ST7789_MADCTL_MX;
            width = 240;
            height = 320;
            break;
        case 3:  // 270°
            madctl = ST7789_MADCTL_MV | ST7789_MADCTL_MY;
            width = 320;
            height = 240;
            break;
    }
    
    write_command(ST7789_CMD_MADCTL);
    write_data_byte(madctl);
}