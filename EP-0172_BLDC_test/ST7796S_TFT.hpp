// ST7796S_TFT.hpp
// ST7796S Display Driver for Raspberry Pi Pico SDK
// Based on Sitronix ST7796S controller architecture

#ifndef ST7796S_TFT_HPP
#define ST7796S_TFT_HPP

#include <cstdint>
#include <cstring>
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

// Display dimensions
#define ST7796_WIDTH  320
#define ST7796_HEIGHT 480

// ST7796S Command Set
#define ST77XX_NOP         0x00
#define ST77XX_SWRESET     0x01
#define ST77XX_RDDID       0x04
#define ST77XX_RDDST       0x09
#define ST77XX_SLPIN       0x10
#define ST77XX_SLPOUT      0x11
#define ST77XX_PTLON       0x12
#define ST77XX_NORON       0x13
#define ST77XX_INVOFF      0x20
#define ST77XX_INVON       0x21
#define ST77XX_GAMSET      0x26
#define ST77XX_DISPOFF     0x28
#define ST77XX_DISPON      0x29
#define ST77XX_CASET       0x2A
#define ST77XX_RASET       0x2B
#define ST77XX_RAMWR       0x2C
#define ST77XX_RAMRD       0x2E
#define ST77XX_PTLAR       0x30
#define ST77XX_MADCTL      0x36
#define ST77XX_COLMOD      0x3A
#define ST77XX_FRMCTR1     0xB1
#define ST77XX_FRMCTR2     0xB2
#define ST77XX_FRMCTR3     0xB3
#define ST77XX_INVCTR      0xB4
#define ST77XX_DISSET5     0xB6
#define ST77XX_GPIDR       0xB7
#define ST77XX_PWCTR1      0xC0
#define ST77XX_PWCTR2      0xC1
#define ST77XX_PWCTR3      0xC2
#define ST77XX_PWCTR4      0xC3
#define ST77XX_PWCTR5      0xC4
#define ST77XX_VMCTR1      0xC5
#define ST77XX_VMCTR2      0xC7
#define ST77XX_WRID2       0xD1
#define ST77XX_WRID3       0xD2
#define ST77XX_NVCTR0      0xD9
#define ST77XX_RDID1       0xDA
#define ST77XX_RDID2       0xDB
#define ST77XX_RDID3       0xDC
#define ST77XX_RDID4       0xDD
#define ST77XX_GMCTRP1     0xE0
#define ST77XX_GMCTRN1     0xE1
#define ST77XX_EXTCTRL     0xF0

// Color definitions (RGB565)
#define COLOR_BLACK        0x0000
#define COLOR_WHITE        0xFFFF
#define COLOR_RED          0xF800
#define COLOR_GREEN        0x07E0
#define COLOR_BLUE         0x001F
#define COLOR_CYAN         0x07FF
#define COLOR_MAGENTA      0xF81F
#define COLOR_YELLOW       0xFFE0

class ST7796_TFT {
public:
    // Constructor
    ST7796_TFT(spi_inst_t *spi_port, uint8_t clk_pin, uint8_t mosi_pin, 
               uint8_t miso_pin, uint8_t cs_pin, uint8_t dc_pin, uint8_t rst_pin,
               uint32_t spi_speed_hz = 10000000);
    
    // Initialization
    void init();
    void setRotation(uint8_t rotation);
    void invertDisplay(bool invert);
    void sleepDisplay(bool sleep);
    void enableDisplay(bool enable);
    
    // Drawing functions
    void fillScreen(uint16_t color);
    void drawPixel(int16_t x, int16_t y, uint16_t color);
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
    void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
    void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
    void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
    void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
    
    // Properties
    uint16_t width() const { return _width; }
    uint16_t height() const { return _height; }
    
private:
    // Hardware configuration
    spi_inst_t *_spi;
    uint8_t _clk_pin;
    uint8_t _mosi_pin;
    uint8_t _miso_pin;
    uint8_t _cs_pin;
    uint8_t _dc_pin;
    uint8_t _rst_pin;
    uint32_t _spi_speed;
    
    // Display state
    uint16_t _width;
    uint16_t _height;
    uint8_t _rotation;
    
    // Internal functions
    void hardwareReset();
    void writeCommand(uint8_t cmd);
    void writeData(uint8_t data);
    void writeDataBlock(const uint8_t *data, size_t len);
    void setAddressWindow(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
    void pushPixel(uint16_t color);
    void pushPixels(const uint16_t *colors, uint32_t count);
    
    // Helper
    void writeCommand16(uint16_t cmd);
    void writeData16(uint16_t data);
};

#endif // ST7796S_TFT_HPP