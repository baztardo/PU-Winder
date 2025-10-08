#include "tmc2209.h"
#include "config.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <algorithm>

// Constructor
TMC2209_UART::TMC2209_UART(uart_inst_t* uart, uint8_t slave_addr, uint tx_pin, uint rx_pin)
    : uart_instance(uart), slave_addr(slave_addr), tx_pin(tx_pin), rx_pin(rx_pin) {
}

// CRC8 calculation
uint8_t TMC2209_UART::crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// Initialize UART
bool TMC2209_UART::begin(uint32_t baud) {
    uart_init(uart_instance, baud);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    
    uart_set_format(uart_instance, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart_instance, true);
    
    sleep_ms(100);
    
    // Flush RX buffer
    while (uart_is_readable(uart_instance)) {
        uart_getc(uart_instance);
    }
    
    return true;
}

// Write register
bool TMC2209_UART::write_reg(uint8_t reg, uint32_t value) {
    // Clear RX buffer
    while (uart_is_readable(uart_instance)) {
        uart_getc(uart_instance);
    }
    
    uint8_t pkt[8];
    pkt[0] = 0x05;
    pkt[1] = slave_addr;
    pkt[2] = 0x80 | (reg & 0x7F);
    pkt[3] = (value >> 24) & 0xFF;
    pkt[4] = (value >> 16) & 0xFF;
    pkt[5] = (value >> 8) & 0xFF;
    pkt[6] = value & 0xFF;
    pkt[7] = crc8(pkt, 7);
    
    uart_write_blocking(uart_instance, pkt, 8);
    uart_tx_wait_blocking(uart_instance);
    sleep_us(100);
    
    return true;
}

// Read register
bool TMC2209_UART::read_reg(uint8_t reg, uint32_t* value, uint32_t timeout_us) {
    // Clear RX buffer
    while (uart_is_readable(uart_instance)) {
        uart_getc(uart_instance);
    }
    
    // Send read request
    uint8_t req[4];
    req[0] = 0x05;
    req[1] = slave_addr;
    req[2] = reg & 0x7F;
    req[3] = crc8(req, 3);
    
    uart_write_blocking(uart_instance, req, 4);
    uart_tx_wait_blocking(uart_instance);
    
    // Wait longer for response
    sleep_ms(2);
    
    // Read response - try JUST 8 bytes (no echo)
    uint8_t resp[8];
    uint32_t start = time_us_32();
    int bytes_read = 0;
    
    while (bytes_read < 8 && (time_us_32() - start) < timeout_us) {
        if (uart_is_readable(uart_instance)) {
            resp[bytes_read++] = uart_getc(uart_instance);
        }
    }
    
    if (bytes_read < 8) {
        return false;  // Didn't get 8 bytes
    }
    
    // Verify CRC on bytes 0-6, check byte 7
    uint8_t calc_crc = crc8(resp, 7);
    if (calc_crc != resp[7]) {
        return false;  // CRC mismatch
    }
    
    // Extract value from bytes 3-6
    if (value) {
        *value = ((uint32_t)resp[3] << 24) |
                 ((uint32_t)resp[4] << 16) |
                 ((uint32_t)resp[5] << 8) |
                 ((uint32_t)resp[6]);
    }
    
    return true;
}

// Set RMS current (CORRECTED FORMULA)
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
    if (!read_reg(TMC_REG_CHOPCONF, &chopconf, 2000)) {
        chopconf = 0x10000053;
    }
    
    if (use_vsense) {
        chopconf |= (1 << 17);
    } else {
        chopconf &= ~(1 << 17);
    }
    
    write_reg(TMC_REG_CHOPCONF, chopconf);
    sleep_ms(10);
    
    return set_ihold_irun(ihold, irun, 10);
}

// Set IHOLD/IRUN
bool TMC2209_UART::set_ihold_irun(uint8_t ihold, uint8_t irun, uint8_t ihold_delay) {
    uint32_t reg = ((uint32_t)ihold & 0x1F) | 
                   (((uint32_t)irun & 0x1F) << 8) | 
                   (((uint32_t)ihold_delay & 0x0F) << 16);
    
    return write_reg(TMC_REG_IHOLD_IRUN, reg);
}

// Set microsteps
bool TMC2209_UART::set_microsteps(uint8_t microsteps) {
    uint8_t mres = 8;
    
    // Remove 256 case - uint8_t max is 255
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
    
    return write_reg(TMC_REG_CHOPCONF, chopconf);
}

// Enable stealthchop
bool TMC2209_UART::enable_stealthchop(bool enable) {
    uint32_t gconf = 0;
    if (enable) {
        gconf |= (1 << 2);
    }
    return write_reg(TMC_REG_GCONF, gconf);
}

// Initialize driver
bool TMC2209_UART::init_driver(float current_ma, uint16_t microsteps) {
    write_reg(TMC_REG_GCONF, 0x00000000);
    sleep_ms(10);
    
    if (!set_rms_current(current_ma, R_SENSE)) {
        return false;
    }
    
    if (!set_microsteps(microsteps)) {
        return false;
    }
    
    if (!enable_stealthchop(true)) {
        return false;
    }
    
    write_reg(TMC_REG_TPOWERDOWN, 20);
    write_reg(TMC_REG_PWMCONF, 0xC10D0024);
    
    return true;
}

// Get driver status
bool TMC2209_UART::get_driver_status(uint32_t* status) {
    return read_reg(TMC_REG_DRV_STATUS, status, 2000);
}

// Check if stalled
bool TMC2209_UART::is_stalled() {
    uint32_t status = 0;
    if (!get_driver_status(&status)) {
        return false;
    }
    return (status & (1 << 24)) != 0;
}

// Check if overtemp
bool TMC2209_UART::is_overtemp() {
    uint32_t status = 0;
    if (!get_driver_status(&status)) {
        return false;
    }
    return ((status & (1 << 26)) != 0) || ((status & (1 << 27)) != 0);
}