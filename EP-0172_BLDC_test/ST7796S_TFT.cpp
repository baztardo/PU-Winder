// ST7796S_TFT.cpp
// ST7796S Display Driver for Raspberry Pi Pico SDK

#include "ST7796S_TFT.hpp"
#include <cstdio>
#include <cmath>

// Constructor
ST7796_TFT::ST7796_TFT(spi_inst_t *spi_port, uint8_t clk_pin, uint8_t mosi_pin, 
                       uint8_t miso_pin, uint8_t cs_pin, uint8_t dc_pin, uint8_t rst_pin,
                       uint32_t spi_speed_hz)
    : _spi(spi_port), _clk_pin(clk_pin), _mosi_pin(mosi_pin), _miso_pin(miso_pin),
      _cs_pin(cs_pin), _dc_pin(dc_pin), _rst_pin(rst_pin), _spi_speed(spi_speed_hz),
      _width(ST7796_WIDTH), _height(ST7796_HEIGHT), _rotation(0)
{
}

void ST7796_TFT::init() {
    printf("[ST7796S] Initializing display...\n");
    
    // Initialize SPI
    printf("[ST7796S] Configuring SPI...\n");
    spi_init(_spi, _spi_speed);
    gpio_set_function(_clk_pin, GPIO_FUNC_SPI);
    gpio_set_function(_mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(_miso_pin, GPIO_FUNC_SPI);
    
    // GPIO control pins
    gpio_init(_cs_pin);
    gpio_set_dir(_cs_pin, GPIO_OUT);
    gpio_put(_cs_pin, 1);
    
    gpio_init(_dc_pin);
    gpio_set_dir(_dc_pin, GPIO_OUT);
    gpio_put(_dc_pin, 0);
    
    gpio_init(_rst_pin);
    gpio_set_dir(_rst_pin, GPIO_OUT);
    gpio_put(_rst_pin, 1);
    
    // Hardware reset
    hardwareReset();
    
    // ST7796S INITIALIZATION SEQUENCE (from Adafruit)
    printf("[ST7796S] Running initialization sequence...\n");
    
    // Software Reset
    writeCommand(ST77XX_SWRESET);
    sleep_ms(150);
    
    // Unlock manufacturer commands
    writeCommand(0xF0);
    writeData(0xC3);
    
    writeCommand(0xF0);
    writeData(0x96);
    
    // VCOM Control
    writeCommand(0xC5);
    writeData(0x1C);
    
    // Memory Access Control (MADCTL)
    writeCommand(ST77XX_MADCTL);
    writeData(0x48);  // RGB mode
    
    // Color Mode - 16-bit RGB565
    writeCommand(ST77XX_COLMOD);
    writeData(0x55);  // 16-bit
    
    // Interface Control
    writeCommand(0xB0);
    writeData(0x80);
    
    // Inversion Control
    writeCommand(0xB4);
    writeData(0x00);
    
    // Display Function Control
    writeCommand(0xB6);
    writeData(0x80);
    writeData(0x02);
    writeData(0x3B);
    
    // Entry Mode
    writeCommand(0xB7);
    writeData(0xC6);
    
    // Lock manufacturer commands
    writeCommand(0xF0);
    writeData(0x69);
    
    writeCommand(0xF0);
    writeData(0x3C);
    
    // Sleep Out
    writeCommand(ST77XX_SLPOUT);
    sleep_ms(150);
    
    // Display ON
    writeCommand(ST77XX_DISPON);
    sleep_ms(150);
    
    printf("[ST7796S] Initialization complete!\n");
    
    // Clear screen
    fillScreen(COLOR_BLACK);
}

void ST7796_TFT::hardwareReset() {
    printf("[ST7796S] Hardware reset...\n");
    sleep_ms(100);
    gpio_put(_rst_pin, 0);
    sleep_ms(100);
    gpio_put(_rst_pin, 1);
    sleep_ms(150);
}

void ST7796_TFT::writeCommand(uint8_t cmd) {
    gpio_put(_cs_pin, 0);
    gpio_put(_dc_pin, 0);  // Command mode
    spi_write_blocking(_spi, &cmd, 1);
    gpio_put(_cs_pin, 1);
}

void ST7796_TFT::writeData(uint8_t data) {
    gpio_put(_cs_pin, 0);
    gpio_put(_dc_pin, 1);  // Data mode
    spi_write_blocking(_spi, &data, 1);
    gpio_put(_cs_pin, 1);
}

void ST7796_TFT::writeDataBlock(const uint8_t *data, size_t len) {
    gpio_put(_cs_pin, 0);
    gpio_put(_dc_pin, 1);  // Data mode
    spi_write_blocking(_spi, (uint8_t *)data, len);
    gpio_put(_cs_pin, 1);
}

void ST7796_TFT::setAddressWindow(int16_t x1, int16_t y1, int16_t x2, int16_t y2) {
    // Column Address Set
    writeCommand(ST77XX_CASET);
    writeData(x1 >> 8);
    writeData(x1 & 0xFF);
    writeData(x2 >> 8);
    writeData(x2 & 0xFF);
    
    // Row Address Set
    writeCommand(ST77XX_RASET);
    writeData(y1 >> 8);
    writeData(y1 & 0xFF);
    writeData(y2 >> 8);
    writeData(y2 & 0xFF);
    
    // RAM Write - keep CS low for pixel data
    gpio_put(_cs_pin, 0);
    gpio_put(_dc_pin, 0);  // Command
    uint8_t cmd = ST77XX_RAMWR;
    spi_write_blocking(_spi, &cmd, 1);
    gpio_put(_dc_pin, 1);  // Data mode
}

void ST7796_TFT::pushPixel(uint16_t color) {
    uint8_t data[2] = {(uint8_t)(color >> 8), (uint8_t)(color & 0xFF)};
    spi_write_blocking(_spi, data, 2);
}

void ST7796_TFT::pushPixels(const uint16_t *colors, uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
        pushPixel(colors[i]);
    }
}

void ST7796_TFT::fillScreen(uint16_t color) {
    printf("[ST7796S] Filling screen with color 0x%04X\n", color);
    fillRect(0, 0, _width, _height, color);
}

void ST7796_TFT::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    if (x < 0 || y < 0 || x + w > _width || y + h > _height) return;
    
    setAddressWindow(x, y, x + w - 1, y + h - 1);
    
    uint32_t pixel_count = (uint32_t)w * h;
    for (uint32_t i = 0; i < pixel_count; i++) {
        pushPixel(color);
    }
    
    gpio_put(_cs_pin, 1);  // End transmission
}

void ST7796_TFT::drawPixel(int16_t x, int16_t y, uint16_t color) {
    if (x < 0 || y < 0 || x >= _width || y >= _height) return;
    
    setAddressWindow(x, y, x, y);
    pushPixel(color);
    gpio_put(_cs_pin, 1);
}

void ST7796_TFT::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    // Top and bottom lines
    fillRect(x, y, w, 1, color);
    fillRect(x, y + h - 1, w, 1, color);
    
    // Left and right lines
    fillRect(x, y, 1, h, color);
    fillRect(x + w - 1, y, 1, h, color);
}

void ST7796_TFT::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
    // Bresenham line algorithm
    int16_t dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int16_t dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx - dy;
    
    while (true) {
        drawPixel(x0, y0, color);
        
        if (x0 == x1 && y0 == y1) break;
        
        int16_t e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void ST7796_TFT::drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    int16_t x = 0;
    int16_t y = r;
    int16_t d = 3 - 2 * r;
    
    while (x <= y) {
        drawPixel(x0 + x, y0 + y, color);
        drawPixel(x0 - x, y0 + y, color);
        drawPixel(x0 + x, y0 - y, color);
        drawPixel(x0 - x, y0 - y, color);
        drawPixel(x0 + y, y0 + x, color);
        drawPixel(x0 - y, y0 + x, color);
        drawPixel(x0 + y, y0 - x, color);
        drawPixel(x0 - y, y0 - x, color);
        
        if (d < 0) {
            d = d + 4 * x + 6;
        } else {
            d = d + 4 * (x - y) + 10;
            y--;
        }
        x++;
    }
}

void ST7796_TFT::fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    drawCircle(x0, y0, r, color);
    for (int16_t i = 1; i < r; i++) {
        drawCircle(x0, y0, i, color);
    }
}

void ST7796_TFT::setRotation(uint8_t rotation) {
    uint8_t madctl = 0;
    _rotation = rotation & 3;
    
    switch (_rotation) {
    case 0:
        madctl = 0x48;
        _width = ST7796_WIDTH;
        _height = ST7796_HEIGHT;
        break;
    case 1:
        madctl = 0x28;
        _width = ST7796_HEIGHT;
        _height = ST7796_WIDTH;
        break;
    case 2:
        madctl = 0x88;
        _width = ST7796_WIDTH;
        _height = ST7796_HEIGHT;
        break;
    case 3:
        madctl = 0xE8;
        _width = ST7796_HEIGHT;
        _height = ST7796_WIDTH;
        break;
    }
    
    writeCommand(ST77XX_MADCTL);
    writeData(madctl);
}

void ST7796_TFT::invertDisplay(bool invert) {
    writeCommand(invert ? ST77XX_INVON : ST77XX_INVOFF);
}

void ST7796_TFT::sleepDisplay(bool sleep) {
    writeCommand(sleep ? 0x10 : ST77XX_SLPOUT);
    sleep_ms(120);
}

void ST7796_TFT::enableDisplay(bool enable) {
    writeCommand(enable ? ST77XX_DISPON : 0x28);
}