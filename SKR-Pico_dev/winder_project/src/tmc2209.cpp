// =============================================================================
// tmc2209.cpp - TMC2209 UART Driver Implementation
// =============================================================================

#include "tmc2209.h"
#include "config.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <cstring>
#include <cmath>
#include <algorithm>

TMC2209_UART::TMC2209_UART(uart_inst_t* uart, uint8_t slave_addr)
    : uart_inst(uart), slave_addr(slave_addr) {
}

bool TMC2209_UART::begin(uint32_t baud) {
    uart_init(uart_instance, baud);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    
    // Critical: Set UART data format
    uart_set_format(uart_instance, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart_instance, true);
    
    // Give UART time to stabilize
    sleep_ms(100);
    
    // Flush any garbage in RX buffer
    while (uart_is_readable(uart_instance)) {
        uart_getc(uart_instance);
    }
    
    return true;
}

bool TMC2209_UART::write_reg(uint8_t reg, uint32_t value) {
    // Clear RX buffer first
    while (uart_is_readable(uart_instance)) {
        uart_getc(uart_instance);
    }
    
    uint8_t pkt[8];
    pkt[0] = 0x05;                        // Sync
    pkt[1] = slave_addr;                  // Address
    pkt[2] = 0x80 | (reg & 0x7F);        // Write bit + register
    pkt[3] = (value >> 24) & 0xFF;
    pkt[4] = (value >> 16) & 0xFF;
    pkt[5] = (value >> 8) & 0xFF;
    pkt[6] = value & 0xFF;
    pkt[7] = crc8(pkt, 7);
    
    // Send packet
    uart_write_blocking(uart_instance, pkt, 8);
    
    // CRITICAL: Wait for TX to complete before continuing
    uart_tx_wait_blocking(uart_instance);
    sleep_us(100);  // Small delay for driver to process
    
    return true;
}

bool TMC2209_UART::read_reg(uint8_t reg, uint32_t* value, uint32_t timeout_us) {
    // Clear RX buffer
    while (uart_is_readable(uart_instance)) {
        uart_getc(uart_instance);
    }
    
    // Send read request
    uint8_t req[4];
    req[0] = 0x05;                   // Sync
    req[1] = slave_addr;             // Address  
    req[2] = reg & 0x7F;             // Read (no write bit)
    req[3] = crc8(req, 3);
    
    uart_write_blocking(uart_instance, req, 4);
    uart_tx_wait_blocking(uart_instance);
    
    // Wait a bit for response
    sleep_us(500);
    
    // Read response (12 bytes total: 4 echo + 8 response)
    uint8_t resp[12];
    uint32_t start = time_us_32();
    int bytes_read = 0;
    
    while (bytes_read < 12 && (time_us_32() - start) < timeout_us) {
        if (uart_is_readable(uart_instance)) {
            resp[bytes_read++] = uart_getc(uart_instance);
        }
    }
    
    if (bytes_read < 12) {
        return false;  // Timeout or incomplete
    }
    
    // Skip first 4 bytes (our echo), use last 8 bytes (response)
    uint8_t* response = &resp[4];
    
    // Verify CRC
    uint8_t calc_crc = crc8(response, 7);
    if (calc_crc != response[7]) {
        return false;  // CRC error
    }
    
    // Extract value
    if (value) {
        *value = ((uint32_t)response[3] << 24) |
                 ((uint32_t)response[4] << 16) |
                 ((uint32_t)response[5] << 8) |
                 ((uint32_t)response[6]);
    }
    
    return true;
}

uint8_t TMC2209_UART::crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void TMC2209_UART::send_bytes(const uint8_t* data, size_t len) {
    uart_write_blocking(uart_inst, data, len);
}

int TMC2209_UART::read_bytes(uint8_t* buf, size_t len, uint32_t timeout_us) {
    uint32_t start = time_us_32();
    size_t read = 0;
    
    while (read < len) {
        if (uart_is_readable(uart_inst)) {
            buf[read++] = uart_getc(uart_inst);
        } else {
            if ((time_us_32() - start) > timeout_us) {
                break;
            }
        }
    }
    
    return read;
}

bool TMC2209_UART::write_reg(uint8_t reg, uint32_t value) {
    uint8_t pkt[8];
    
    pkt[0] = 0x05;                      // Sync byte
    pkt[1] = slave_addr & 0x0F;        // Slave address
    pkt[2] = 0x80 | (reg & 0x7F);      // Register + write bit
    pkt[3] = (value >> 24) & 0xFF;
    pkt[4] = (value >> 16) & 0xFF;
    pkt[5] = (value >> 8) & 0xFF;
    pkt[6] = value & 0xFF;
    pkt[7] = crc8(pkt, 7);
    
    send_bytes(pkt, 8);
    return true;
}

bool TMC2209_UART::read_reg(uint8_t reg, uint32_t* value, uint32_t timeout_us) {
    // Send read request
    uint8_t req[4];
    req[0] = 0x05;                  // Sync byte
    req[1] = slave_addr & 0x0F;    // Slave address
    req[2] = reg & 0x7F;           // Register (no write bit)
    req[3] = crc8(req, 3);
    
    send_bytes(req, 4);
    
    // Read response
    uint8_t resp[8];
    int r = read_bytes(resp, 8, timeout_us);
    
    if (r < 8) return false;
    
    // Verify CRC
    uint8_t crc = crc8(resp, 7);
    if (crc != resp[7]) return false;
    
    // Extract value
    if (value) {
        *value = ((uint32_t)resp[3] << 24) |
                 ((uint32_t)resp[4] << 16) |
                 ((uint32_t)resp[5] << 8) |
                 ((uint32_t)resp[6]);
    }
    
    return true;
}

// From Klipper TMC2208/2209 current calculation
// Formula: I_rms = (CS+1)/32 * Vref/(sqrt(2)*Rsense)
// Where Vref = 0.325V (vsense=1) or 0.180V (vsense=0)

bool TMC2209_UART::set_rms_current(float rms_ma, float r_sense) {
    const float vsense_high = 0.325f;  // High sensitivity (vsense=1)
    const float vsense_low = 0.180f;   // Low sensitivity (vsense=0)
    const float sqrt2 = 1.41421356f;
    
    // Try high sensitivity first (better for low currents)
    float vref = vsense_high;
    float cs_float = (32.0f * sqrt2 * (rms_ma / 1000.0f) * r_sense / vref) - 1.0f;
    
    bool use_vsense = true;
    
    // If CS > 31, switch to low sensitivity
    if (cs_float > 31.0f) {
        vref = vsense_low;
        cs_float = (32.0f * sqrt2 * (rms_ma / 1000.0f) * r_sense / vref) - 1.0f;
        use_vsense = false;
    }
    
    // Clamp and round
    int irun = (int)(cs_float + 0.5f);
    if (irun < 0) irun = 0;
    if (irun > 31) irun = 31;
    
    // Hold = 30% of run
    int ihold = (int)(irun * 0.3f);
    if (ihold > 31) ihold = 31;
    
    // Update CHOPCONF with vsense bit
    uint32_t chopconf;
    if (!read_reg(TMC_REG_CHOPCONF, &chopconf, 2000)) {
        chopconf = 0x10000053;  // Default if read fails
    }
    
    if (use_vsense) {
        chopconf |= (1 << 17);   // Set vsense=1
    } else {
        chopconf &= ~(1 << 17);  // Clear vsense=0  
    }
    
    write_reg(TMC_REG_CHOPCONF, chopconf);
    sleep_ms(10);
    
    return set_ihold_irun(ihold, irun, 10);
}

bool TMC2209_UART::set_ihold_irun(uint8_t ihold, uint8_t irun, uint8_t ihold_delay) {
    uint32_t reg = ((uint32_t)ihold & 0x1F) | 
                   (((uint32_t)irun & 0x1F) << 8) | 
                   (((uint32_t)ihold_delay & 0x0F) << 16);
    
    return write_reg(TMC_REG_IHOLD_IRUN, reg);
}

bool TMC2209_UART::set_microsteps(uint8_t microsteps) {
    // Calculate MRES value from microsteps
    uint8_t mres = 8;  // 256 microsteps
    
    switch(microsteps) {
        case 256: mres = 0; break;
        case 128: mres = 1; break;
        case 64:  mres = 2; break;
        case 32:  mres = 3; break;
        case 16:  mres = 4; break;
        case 8:   mres = 5; break;
        case 4:   mres = 6; break;
        case 2:   mres = 7; break;
        case 1:   mres = 8; break;
        default:  mres = 4; break;  // Default to 16
    }
    
    // Read current CHOPCONF
    uint32_t chopconf = 0x10000053;  // Default value with good settings
    
    // Set MRES field
    chopconf &= ~(0x0F << 24);  // Clear MRES bits
    chopconf |= ((uint32_t)mres << 24);
    
    return write_reg(TMC_REG_CHOPCONF, chopconf);
}

bool TMC2209_UART::enable_stealthchop(bool enable) {
    uint32_t gconf = 0;
    
    if (enable) {
        gconf |= (1 << 2);  // en_pwm_mode = 1
    }
    
    return write_reg(TMC_REG_GCONF, gconf);
}

bool TMC2209_UART::init_driver(float current_ma, uint16_t microsteps) {
    // Reset and basic configuration
    write_reg(TMC_REG_GCONF, 0x00000000);
    sleep_ms(10);
    
    // Set current
    if (!set_rms_current(current_ma, R_SENSE)) {
        return false;
    }
    
    // Set microstepping
    if (!set_microsteps(microsteps)) {
        return false;
    }
    
    // Enable StealthChop for quiet operation
    if (!enable_stealthchop(true)) {
        return false;
    }
    
    // Set power down delay - reduce current after 2 seconds of no movement
    write_reg(TMC_REG_TPOWERDOWN, 20);  // 20 * 0.1s = 2 seconds
    
    // PWM configuration for StealthChop - optimized for low heat
    // PWM_AUTOSCALE=1, PWM_AUTOGRAD=1 for automatic tuning
    write_reg(TMC_REG_PWMCONF, 0xC10D0024);
    
    // CHOPCONF with optimized settings for smooth, cool operation
    uint32_t chopconf = 0x10000053;  // Good default settings
    
    // Set MRES based on microsteps
    uint8_t mres = 4;  // Default 16 microsteps
    switch(microsteps) {
        case 256: mres = 0; break;
        case 128: mres = 1; break;
        case 64:  mres = 2; break;
        case 32:  mres = 3; break;
        case 16:  mres = 4; break;
        case 8:   mres = 5; break;
        case 4:   mres = 6; break;
        case 2:   mres = 7; break;
        case 1:   mres = 8; break;
    }
    
    chopconf &= ~(0x0F << 24);
    chopconf |= ((uint32_t)mres << 24);
    
    // Add TOFF=5 for good performance and heat management
    chopconf &= ~0x0F;
    chopconf |= 0x05;
    
    write_reg(TMC_REG_CHOPCONF, chopconf);
    
    return true;
}

bool TMC2209_UART::get_driver_status(uint32_t* status) {
    return read_reg(TMC_REG_DRV_STATUS, status, 2000);
}

bool TMC2209_UART::is_stalled() {
    uint32_t status = 0;
    if (!get_driver_status(&status)) {
        return false;
    }
    
    // Check stallGuard bit
    return (status & (1 << 24)) != 0;
}

bool TMC2209_UART::is_overtemp() {
    uint32_t status = 0;
    if (!get_driver_status(&status)) {
        return false;
    }
    
    // Check overtemperature warning and shutdown bits
    return ((status & (1 << 26)) != 0) || ((status & (1 << 27)) != 0);
}