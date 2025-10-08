// =============================================================================
// tmc2209.h - TMC2209 Stepper Driver (UART Interface)
// Purpose: Handle all TMC2209 register reads/writes via UART protocol
// =============================================================================

#pragma once

#include "hardware/uart.h"
#include <cstdint>

// =============================================================================
// TMC2209 Register Addresses
// =============================================================================
#define TMC_REG_GCONF       0x00
#define TMC_REG_GSTAT       0x01
#define TMC_REG_IFCNT       0x02
#define TMC_REG_IOIN        0x06
#define TMC_REG_IHOLD_IRUN  0x10
#define TMC_REG_TPOWERDOWN  0x11
#define TMC_REG_TSTEP       0x12
#define TMC_REG_TPWMTHRS    0x13
#define TMC_REG_VACTUAL     0x22
#define TMC_REG_CHOPCONF    0x6C
#define TMC_REG_DRV_STATUS  0x6F
#define TMC_REG_PWMCONF     0x70

// =============================================================================
// TMC2209_UART Class
// =============================================================================
class TMC2209_UART {
public:
    /**
     * @brief Constructor
     * @param uart UART instance (uart0 or uart1)
     * @param slave_addr UART slave address (0-3)
     */
    // Change this line in tmc2209.h:
    TMC2209_UART(uart_inst_t* uart, uint8_t slave_addr, uint tx_pin, uint rx_pin);
    
    /**
     * @brief Initialize UART communication
     * @param baud Baud rate (default 115200)
     * @return true if successful
     */
    bool begin(uint32_t baud = 115200);
    
    /**
     * @brief Write a 32-bit register
     * @param reg Register address
     * @param value 32-bit value to write
     * @return true if successful
     */
    bool write_reg(uint8_t reg, uint32_t value);
    
    /**
     * @brief Read a 32-bit register
     * @param reg Register address
     * @param value Pointer to store result
     * @param timeout_us Timeout in microseconds
     * @return true if successful
     */
    bool read_reg(uint8_t reg, uint32_t* value, uint32_t timeout_us = 2000);
    
    /**
     * @brief Initialize driver with standard settings
     * @param current_ma RMS current in milliamps
     * @param microsteps Microstepping (1,2,4,8,16,32,64,128,256)
     * @return true if successful
     */
    bool init_driver(float current_ma, uint16_t microsteps = 16);
    
    /**
     * @brief Set RMS current
     * @param rms_ma RMS current in milliamps
     * @param r_sense Sense resistor value in ohms
     * @return true if successful
     */
    bool set_rms_current(float rms_ma, float r_sense);
    
    /**
     * @brief Set IHOLD and IRUN values directly
     * @param ihold Hold current (0-31)
     * @param irun Run current (0-31)
     * @param ihold_delay Hold delay (0-15)
     * @return true if successful
     */
    bool set_ihold_irun(uint8_t ihold, uint8_t irun, uint8_t ihold_delay);
    
    /**
     * @brief Set microstepping
     * @param microsteps Must be power of 2, up to 256
     * @return true if successful
     */
    bool set_microsteps(uint8_t microsteps);
    
    /**
     * @brief Enable/disable StealthChop mode
     * @param enable true to enable StealthChop
     * @return true if successful
     */
    bool enable_stealthchop(bool enable);
    
    /**
     * @brief Read driver status register
     * @param status Pointer to store status
     * @return true if successful
     */
    bool get_driver_status(uint32_t* status);
    
    /**
     * @brief Check if driver is in stallGuard condition
     * @return true if stalled
     */
    bool is_stalled();
    
    /**
     * @brief Check if driver has overtemperature warning
     * @return true if overtemp
     */
    bool is_overtemp();

private:
    uart_inst_t* uart_instance;
    uint8_t slave_addr;
    uint tx_pin;
    uint rx_pin;

    /**
     * @brief Calculate CRC8 for TMC UART protocol
     * @param data Data buffer
     * @param len Length of data
     * @return CRC8 value
     */
    uint8_t crc8(const uint8_t* data, size_t len);
    
    /**
     * @brief Send raw bytes via UART
     * @param data Data buffer
     * @param len Length of data
     */
    void send_bytes(const uint8_t* data, size_t len);
    
    /**
     * @brief Read raw bytes from UART with timeout
     * @param buf Buffer to store data
     * @param len Number of bytes to read
     * @param timeout_us Timeout in microseconds
     * @return Number of bytes actually read
     */
    int read_bytes(uint8_t* buf, size_t len, uint32_t timeout_us);
};