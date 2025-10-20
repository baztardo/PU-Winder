// =============================================================================
// st7789_pico.h - ST7789 TFT Display Driver for Raspberry Pi Pico (Pico SDK)
// =============================================================================
// Native implementation for SKR-Pico board
// No Arduino dependencies - pure Pico SDK
// SPI Interface (4-wire or 3-wire mode)
// =============================================================================

#pragma once

#include <cstdint>
#include <cstddef>
#include "hardware/spi.h"
#include "hardware/gpio.h"

// =============================================================================
// ST7789 Register Definitions
// =============================================================================

// Command opcodes
#define ST7789_CMD_NOP          0x00
#define ST7789_CMD_SWRESET      0x01
#define ST7789_CMD_SLPIN        0x10
#define ST7789_CMD_SLPOUT       0x11
#define ST7789_CMD_PTLON        0x12
#define ST7789_CMD_NORON        0x13
#define ST7789_CMD_INVOFF       0x20
#define ST7789_CMD_INVON        0x21
#define ST7789_CMD_DISPOFF      0x28
#define ST7789_CMD_DISPON       0x29
#define ST7789_CMD_CASET        0x2A  // Column address set
#define ST7789_CMD_RASET        0x2B  // Row address set
#define ST7789_CMD_RAMWR        0x2C  // RAM write
#define ST7789_CMD_RAMRD        0x2E  // RAM read
#define ST7789_CMD_COLMOD       0x3A  // Color mode
#define ST7789_CMD_MADCTL       0x36  // Memory data access control
#define ST7789_CMD_VSCSAD       0x37  // Vertical scroll address
#define ST7789_CMD_VCMOFFSETA   0x40  // VCOMS offset set A
#define ST7789_CMD_VCMOFFSETB   0x41  // VCOMS offset set B
#define ST7789_CMD_FRMCTR1      0xB1  // Frame rate control 1
#define ST7789_CMD_FRMCTR2      0xB2  // Frame rate control 2
#define ST7789_CMD_FRMCTR3      0xB3  // Frame rate control 3
#define ST7789_CMD_GCTRL        0xB7  // Gate control
#define ST7789_CMD_VCOMS        0xBB  // VCOM setting
#define ST7789_CMD_LCMCTRL      0xC0  // LCM control
#define ST7789_CMD_IDSET        0xC1  // ID set (read)
#define ST7789_CMD_VDVS         0xC2  // VDV and VRH command enable
#define ST7789_CMD_VRHS         0xC3  // VRH set
#define ST7789_CMD_VDVSET       0xC4  // VDV set
#define ST7789_CMD_PWCTRL1      0xD0  // Power control 1
#define ST7789_CMD_PWCTRL2      0xE0  // Power control 2 (positive gamma)
#define ST7789_CMD_PWCTRL3      0xE1  // Power control 3 (negative gamma)

// Memory data access control bits
#define ST7789_MADCTL_MY        0x80  // Row address order
#define ST7789_MADCTL_MX        0x40  // Column address order
#define ST7789_MADCTL_MV        0x20  // Row/column exchange
#define ST7789_MADCTL_ML        0x10  // Vertical refresh
#define ST7789_MADCTL_RGB       0x00  // RGB color order
#define ST7789_MADCTL_BGR       0x08  // BGR color order

// Color modes
#define ST7789_COLMOD_12BIT     0x03
#define ST7789_COLMOD_16BIT     0x05
#define ST7789_COLMOD_18BIT     0x06

// =============================================================================
// Color Definitions (16-bit RGB565)
// =============================================================================
#define ST7789_BLACK            0x0000
#define ST7789_WHITE            0xFFFF
#define ST7789_RED              0xF800
#define ST7789_GREEN            0x07E0
#define ST7789_BLUE             0x001F
#define ST7789_CYAN             0x07FF
#define ST7789_MAGENTA          0xF81F
#define ST7789_YELLOW           0xFFE0

// =============================================================================
// ST7789 Display Class
// =============================================================================
class ST7789 {
public:
    /**
     * @brief Constructor
     * @param spi_inst SPI instance (spi0 or spi1)
     * @param sck_pin SPI clock pin
     * @param mosi_pin SPI MOSI pin
     * @param miso_pin SPI MISO pin (optional, use -1 if not used)
     * @param cs_pin Chip select pin
     * @param dc_pin Data/Command pin
     * @param rst_pin Reset pin (optional, use -1 if not used)
     * @param width Display width in pixels
     * @param height Display height in pixels
     */
    ST7789(spi_inst_t* spi_inst,
           int sck_pin, int mosi_pin, int miso_pin,
           int cs_pin, int dc_pin, int rst_pin = -1,
           uint16_t width = 240, uint16_t height = 320);
    
    /**
     * @brief Initialize display
     * @param spi_freq SPI frequency in Hz (default 62.5 MHz)
     * @return true if successful
     */
    bool init(uint32_t spi_freq = 62500000);
    
    /**
     * @brief Fill entire display with color
     * @param color 16-bit RGB565 color
     */
    void fill(uint16_t color);
    
    /**
     * @brief Draw a pixel at coordinates
     * @param x X coordinate
     * @param y Y coordinate
     * @param color 16-bit RGB565 color
     */
    void draw_pixel(int16_t x, int16_t y, uint16_t color);
    
    /**
     * @brief Draw a horizontal line
     * @param x Start X coordinate
     * @param y Y coordinate
     * @param length Line length
     * @param color 16-bit RGB565 color
     */
    void draw_hline(int16_t x, int16_t y, int16_t length, uint16_t color);
    
    /**
     * @brief Draw a vertical line
     * @param x X coordinate
     * @param y Start Y coordinate
     * @param length Line length
     * @param color 16-bit RGB565 color
     */
    void draw_vline(int16_t x, int16_t y, int16_t length, uint16_t color);
    
    /**
     * @brief Draw a filled rectangle
     * @param x1 Top-left X coordinate
     * @param y1 Top-left Y coordinate
     * @param x2 Bottom-right X coordinate
     * @param y2 Bottom-right Y coordinate
     * @param color 16-bit RGB565 color
     */
    void fill_rect(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
    
    /**
     * @brief Draw a rectangle outline
     * @param x1 Top-left X coordinate
     * @param y1 Top-left Y coordinate
     * @param x2 Bottom-right X coordinate
     * @param y2 Bottom-right Y coordinate
     * @param color 16-bit RGB565 color
     */
    void draw_rect(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
    
    /**
     * @brief Draw a circle
     * @param x Center X coordinate
     * @param y Center Y coordinate
     * @param r Radius
     * @param color 16-bit RGB565 color
     */
    void draw_circle(int16_t x, int16_t y, int16_t r, uint16_t color);
    
    /**
     * @brief Fill a circle
     * @param x Center X coordinate
     * @param y Center Y coordinate
     * @param r Radius
     * @param color 16-bit RGB565 color
     */
    void fill_circle(int16_t x, int16_t y, int16_t r, uint16_t color);
    
    /**
     * @brief Display power on/off
     * @param on true to turn on, false to turn off
     */
    void display_on(bool on);
    
    /**
     * @brief Invert display colors
     * @param invert true to invert, false for normal
     */
    void invert(bool invert);
    
    /**
     * @brief Set display rotation
     * @param rotation 0=0°, 1=90°, 2=180°, 3=270°
     */
    void set_rotation(uint8_t rotation);
    
    /**
     * @brief Get display width
     * @return Width in pixels
     */
    uint16_t get_width() const { return width; }
    
    /**
     * @brief Get display height
     * @return Height in pixels
     */
    uint16_t get_height() const { return height; }

private:
    spi_inst_t* spi;
    int sck, mosi, miso;
    int cs, dc, rst;
    uint16_t width, height;
    uint8_t rotation;
    
    // Low-level SPI functions
    void write_command(uint8_t cmd);
    void write_data_byte(uint8_t data);
    void write_data_word(uint16_t data);
    void write_data(const uint8_t* data, size_t len);
    uint8_t read_data();
    
    // Display control
    void set_address_window(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
    void reset();
};

// =============================================================================
// Inline helper for RGB565 color conversion
// =============================================================================
static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}