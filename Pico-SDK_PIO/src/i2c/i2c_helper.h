#ifndef I2C_HELPER_H
#define I2C_HELPER_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

/**
 * I2C Helper Library
 * Generic I2C communication functions for RP2040
 */

// I2C configuration structure
typedef struct {
    i2c_inst_t *port;
    uint8_t sda_pin;
    uint8_t scl_pin;
    uint32_t baudrate;
} i2c_config_t;

/**
 * Initialize I2C bus
 * @param config I2C configuration structure
 * @return true if successful
 */
bool i2c_helper_init(const i2c_config_t *config);

/**
 * Write data to I2C device
 * @param port I2C port instance
 * @param addr 7-bit I2C device address
 * @param data Pointer to data buffer
 * @param len Number of bytes to write
 * @param nostop If true, master retains control of the bus at the end
 * @return Number of bytes written, or PICO_ERROR_GENERIC on error
 */
int i2c_helper_write(i2c_inst_t *port, uint8_t addr, const uint8_t *data, size_t len, bool nostop);

/**
 * Read data from I2C device
 * @param port I2C port instance
 * @param addr 7-bit I2C device address
 * @param data Pointer to buffer to receive data
 * @param len Number of bytes to read
 * @param nostop If true, master retains control of the bus at the end
 * @return Number of bytes read, or PICO_ERROR_GENERIC on error
 */
int i2c_helper_read(i2c_inst_t *port, uint8_t addr, uint8_t *data, size_t len, bool nostop);

/**
 * Write single byte to I2C device
 * @param port I2C port instance
 * @param addr 7-bit I2C device address
 * @param data Byte to write
 * @return true if successful
 */
bool i2c_helper_write_byte(i2c_inst_t *port, uint8_t addr, uint8_t data);

/**
 * Read single byte from I2C device
 * @param port I2C port instance
 * @param addr 7-bit I2C device address
 * @param data Pointer to receive byte
 * @return true if successful
 */
bool i2c_helper_read_byte(i2c_inst_t *port, uint8_t addr, uint8_t *data);

/**
 * Write to register on I2C device
 * @param port I2C port instance
 * @param addr 7-bit I2C device address
 * @param reg Register address
 * @param data Data to write
 * @return true if successful
 */
bool i2c_helper_write_register(i2c_inst_t *port, uint8_t addr, uint8_t reg, uint8_t data);

/**
 * Read from register on I2C device
 * @param port I2C port instance
 * @param addr 7-bit I2C device address
 * @param reg Register address
 * @param data Pointer to receive data
 * @return true if successful
 */
bool i2c_helper_read_register(i2c_inst_t *port, uint8_t addr, uint8_t reg, uint8_t *data);

/**
 * Scan I2C bus for devices
 * @param port I2C port instance
 * @param found_devices Array to store found device addresses (max 128)
 * @return Number of devices found
 */
int i2c_helper_scan(i2c_inst_t *port, uint8_t *found_devices);

/**
 * Check if device is present on I2C bus
 * @param port I2C port instance
 * @param addr 7-bit I2C device address
 * @return true if device responds
 */
bool i2c_helper_device_present(i2c_inst_t *port, uint8_t addr);

#endif // I2C_HELPER_H
