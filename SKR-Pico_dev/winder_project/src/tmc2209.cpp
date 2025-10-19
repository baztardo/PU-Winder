#include "tmc2209.h"
#include "config.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <algorithm>

// =============================================================================
// Configuration constants
// =============================================================================
#define TMC_BAUDRATE 115200
#define BIT_DELAY_US (1000000 / TMC_BAUDRATE)

// =============================================================================
// CRC helper
// =============================================================================
static uint8_t tmc_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t c = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ c) & 0x80;
            crc <<= 1;
            if (mix) crc ^= 0x07;
            c <<= 1;
        }
    }
    return crc;
}

// =============================================================================
// Software UART (bit-banged, single-wire) helpers
// =============================================================================
static inline void sw_uart_tx_byte(uint8_t pin, uint8_t b) {
    gpio_put(pin, 0); sleep_us(BIT_DELAY_US);
    for (int i = 0; i < 8; i++) {
        gpio_put(pin, (b >> i) & 1);
        sleep_us(BIT_DELAY_US);
    }
    gpio_put(pin, 1); sleep_us(BIT_DELAY_US);
}

static inline uint8_t sw_uart_rx_byte(uint8_t pin) {
    uint8_t v = 0;
    while (gpio_get(pin)) tight_loop_contents();  // wait start bit
    sleep_us(BIT_DELAY_US + BIT_DELAY_US / 2);
    for (int i = 0; i < 8; i++) {
        v |= (gpio_get(pin) << i);
        sleep_us(BIT_DELAY_US);
    }
    sleep_us(BIT_DELAY_US);  // stop bit
    return v;
}

// =============================================================================
// Class implementation
// =============================================================================
TMC2209_UART::TMC2209_UART(uart_inst_t* uart_port, int tx_pin, int rx_pin, uint8_t slave_addr)
    : uart_inst(uart_port),
      tx(tx_pin), rx(rx_pin),
      pin_mode(false), slave_addr(slave_addr)
{
    if (uart_port) {
        // Hardware UART mode
        uart_init(uart_port, TMC_BAUDRATE);
        gpio_set_function(tx, GPIO_FUNC_UART);
        gpio_set_function(rx, GPIO_FUNC_UART);
    } else {
        // Software UART (single wire)
        pin_mode = true;
        gpio_init(tx);
        gpio_set_dir(tx, GPIO_OUT);
        gpio_put(tx, 1);
    }
}

// For SKR Pico: single-wire convenience constructor
TMC2209_UART::TMC2209_UART(uint8_t gpio_pin, uint8_t slave_addr)
    : uart_inst(nullptr), tx(gpio_pin), rx(gpio_pin),
      pin_mode(false), slave_addr(slave_addr) 
{
    gpio_init(tx);
    gpio_set_dir(tx, GPIO_OUT);
    gpio_put(tx, 1);
}

// =============================================================================
bool TMC2209_UART::writeRegister(uint8_t reg, uint32_t value) {
    uint8_t tx[8] = {
        0x05,                     // sync
        (uint8_t)(slave_addr | 0x80), // write bit set
        reg,
        (uint8_t)((value >> 24) & 0xFF),
        (uint8_t)((value >> 16) & 0xFF),
        (uint8_t)((value >> 8) & 0xFF),
        (uint8_t)(value & 0xFF),
        0
    };
    tx[7] = crc8(tx, 7);

    uart_write_blocking(uart_inst, tx, 8);

    // Optional read-back or wait delay
    sleep_ms(5);
    return true;
}


// =============================================================================
bool TMC2209_UART::readRegister(uint8_t reg, uint32_t &value) {
    uint8_t tx[4] = {0x05, (uint8_t)(slave_addr & 0x7F), reg, 0x00};
    tx[3] = crc8(tx, 3);   // ✅ correct line

    uart_write_blocking(uart_inst, tx, 4);

    uint8_t rx[8] = {0};
    int got = 0;
    absolute_time_t end_time = make_timeout_time_ms(200); // 200 ms timeout

    while (got < 8 && absolute_time_diff_us(get_absolute_time(), end_time) > 0) {
        if (uart_is_readable(uart_inst)) {
            rx[got++] = uart_getc(uart_inst);
        }
    }

    if (got < 8) {
        printf("⚠️ TMC2209 read timeout on reg 0x%02X\n", reg);
        return false;
    }

    value = ((uint32_t)rx[4] << 24) | ((uint32_t)rx[5] << 16) |
            ((uint32_t)rx[6] << 8) | rx[7];
    return true;
}



// =============================================================================
// Simple diagnostic UART read test
void TMC2209_UART::testRead() {
    // Transmit a simple read command (example: IFCNT register)
    uint8_t tx[4] = {0x05, 0x00, 0x00, 0x00};
    uart_write_blocking(uart_inst, tx, 4);

    // Receive buffer for response
    uint8_t rx[5] = {0};  // receive buffer

    // Non-blocking UART read with timeout
    int got = 0;
    absolute_time_t end_time = make_timeout_time_ms(1000); // 1 second timeout
    while (got < 5 && absolute_time_diff_us(get_absolute_time(), end_time) > 0) {
        if (uart_is_readable(uart_inst)) {
            rx[got++] = uart_getc(uart_inst);
        }
    }

    // Timeout indicator (LED on FAN3)
    gpio_init(DEBUG_PIN);
    gpio_set_dir(DEBUG_PIN, GPIO_OUT);

    if (got < 5) {
        gpio_put(DEBUG_PIN, 1);
        sleep_ms(500);
        gpio_put(DEBUG_PIN, 0);
        printf("UART timeout (no response)\n");
        return; // Timeout
    }

    // If it got here, data was received
    printf("TMC2209 UART read %d bytes: ", got);
    for (int i = 0; i < got; i++) {
        printf("%02X ", rx[i]);
    }
    printf("\n");

    // Blink short success signal
    for (int i = 0; i < 3; i++) {
        gpio_put(DEBUG_PIN, 1);
        sleep_ms(100);
        gpio_put(DEBUG_PIN, 0);
        sleep_ms(100);
    }
}



// =============================================================================
// Helper functions to match your existing code
// =============================================================================
bool TMC2209_UART::set_rms_current(float rms_ma, float r_sense) {
    const float vsense_high = 0.325f;
    const float vsense_low = 0.180f;
    const float sqrt2 = 1.41421356f;
    
    float vref = vsense_high;
    float cs_float = (32.0f * sqrt2 * (rms_ma / 1000.0f) * r_sense / vref) - 1.0f;
    
    bool use_vsense = true;
    
    if (cs_float > 31.0f) {
        vref = vsense_low;
        cs_float = (32.0f * sqrt2 * (rms_ma / 1000.0f) * r_sense / vref) - 1.0f;
        use_vsense = false;
    }
    
    int irun = (int)(cs_float + 0.5f);
    if (irun < 0) irun = 0;
    if (irun > 31) irun = 31;
    
    int ihold = (int)(irun * 0.3f);
    if (ihold > 31) ihold = 31;
    
    uint32_t chopconf;
    if (!readRegister(TMC_REG_CHOPCONF, chopconf)) {
        chopconf = 0x10000053;
    }
    
    if (use_vsense) {
        chopconf |= (1 << 17);
    } else {
        chopconf &= ~(1 << 17);
    }
    
    writeRegister(TMC_REG_CHOPCONF, chopconf);
    sleep_ms(10);
    
    return set_ihold_irun(ihold, irun, 10);
}

bool TMC2209_UART::set_ihold_irun(uint8_t ihold, uint8_t irun, uint8_t ihold_delay) {
    uint32_t reg = ((uint32_t)ihold & 0x1F) | 
                   (((uint32_t)irun & 0x1F) << 8) | 
                   (((uint32_t)ihold_delay & 0x0F) << 16);
    
    writeRegister(TMC_REG_IHOLD_IRUN, reg);
    return true;
}

bool TMC2209_UART::set_microsteps(uint8_t microsteps) {
    uint8_t mres = 8;
    
    switch(microsteps) {
        case 128: mres = 1; break;
        case 64:  mres = 2; break;
        case 32:  mres = 3; break;
        case 16:  mres = 4; break;
        case 8:   mres = 5; break;
        case 4:   mres = 6; break;
        case 2:   mres = 7; break;
        case 1:   mres = 8; break;
        default:  mres = 4; break;
    }
    
    uint32_t chopconf = 0x10000053;
    chopconf &= ~(0x0F << 24);
    chopconf |= ((uint32_t)mres << 24);
    
    writeRegister(TMC_REG_CHOPCONF, chopconf);
    return true;
}

bool TMC2209_UART::enable_stealthchop(bool enable) {
    uint32_t gconf = 0;
    if (enable) {
        gconf |= (1 << 2);  // StealthChop: quiet but less torque
    } else {
        gconf &= ~(1 << 2); // SpreadCycle: noisy but MORE TORQUE!
    }
    writeRegister(TMC_REG_GCONF, gconf);
    return true;
}

bool TMC2209_UART::init_driver(float current_ma, uint8_t microsteps) {
    writeRegister(TMC_REG_GCONF, 0x00000000);
    sleep_ms(10);
    
    if (!set_rms_current(current_ma, R_SENSE)) {
        return false;
    }
    
    if (!set_microsteps(microsteps)) {
        return false;
    }
    
    // BEAST MODE: SpreadCycle for HIGH TORQUE at high speeds!
    if (!enable_stealthchop(false)) {  // false = SpreadCycle mode! 🔥
        return false;
    }
    
    writeRegister(TMC_REG_TPOWERDOWN, 20);
    
    // SpreadCycle tuned CHOPCONF (more aggressive for high speed!)
    writeRegister(TMC_REG_CHOPCONF, 0x000100C3);  // Optimized for torque!
    
    writeRegister(TMC_REG_PWMCONF, 0xC10D0024);
    
    return true;
}

bool TMC2209_UART::get_driver_status(uint32_t* status) {
    return readRegister(TMC_REG_DRV_STATUS, *status);
}

bool TMC2209_UART::is_stalled() {
    uint32_t status = 0;
    if (!get_driver_status(&status)) {
        return false;
    }
    return (status & (1 << 24)) != 0;
}

bool TMC2209_UART::is_overtemp() {
    uint32_t status = 0;
    if (!get_driver_status(&status)) {
        return false;
    }
    return ((status & (1 << 26)) != 0) || ((status & (1 << 27)) != 0);
}