#ifndef MCP23017_H
#define MCP23017_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

/**
 * MCP23017 16-Bit I2C GPIO Expander Driver
 * Example of another I2C device driver using i2c_helper
 * 
 * This is an EXAMPLE driver to demonstrate the architecture.
 * Include this in your build only if you're using an MCP23017.
 */

// MCP23017 Register Addresses
#define MCP23017_IODIRA    0x00   // I/O direction register A
#define MCP23017_IODIRB    0x01   // I/O direction register B
#define MCP23017_GPIOA     0x12   // Port A register
#define MCP23017_GPIOB     0x13   // Port B register
#define MCP23017_OLATA     0x14   // Output latch A
#define MCP23017_OLATB     0x15   // Output latch B

// Default I2C address (A2,A1,A0 = 0,0,0)
#define MCP23017_ADDRESS   0x20

// Port definitions
typedef enum {
    MCP23017_PORTA = 0,
    MCP23017_PORTB = 1
} mcp23017_port_t;

// Pin mode
typedef enum {
    MCP23017_OUTPUT = 0,
    MCP23017_INPUT = 1
} mcp23017_mode_t;

// Device structure
typedef struct {
    i2c_inst_t *i2c_port;
    uint8_t address;
} mcp23017_t;

/**
 * Initialize MCP23017
 * @param dev Device structure
 * @param i2c_port I2C port (must be already initialized)
 * @param address 7-bit I2C address
 * @return true if successful
 */
bool mcp23017_init(mcp23017_t *dev, i2c_inst_t *i2c_port, uint8_t address);

/**
 * Set pin mode (input/output)
 * @param dev Device structure
 * @param port Port A or B
 * @param pin Pin number (0-7)
 * @param mode Input or output
 * @return true if successful
 */
bool mcp23017_pin_mode(mcp23017_t *dev, mcp23017_port_t port, uint8_t pin, mcp23017_mode_t mode);

/**
 * Write to pin
 * @param dev Device structure
 * @param port Port A or B
 * @param pin Pin number (0-7)
 * @param value 0 or 1
 * @return true if successful
 */
bool mcp23017_digital_write(mcp23017_t *dev, mcp23017_port_t port, uint8_t pin, bool value);

/**
 * Read from pin
 * @param dev Device structure
 * @param port Port A or B
 * @param pin Pin number (0-7)
 * @param value Pointer to store read value
 * @return true if successful
 */
bool mcp23017_digital_read(mcp23017_t *dev, mcp23017_port_t port, uint8_t pin, bool *value);

/**
 * Write entire port (8 bits)
 * @param dev Device structure
 * @param port Port A or B
 * @param value 8-bit value
 * @return true if successful
 */
bool mcp23017_write_port(mcp23017_t *dev, mcp23017_port_t port, uint8_t value);

/**
 * Read entire port (8 bits)
 * @param dev Device structure
 * @param port Port A or B
 * @param value Pointer to store read value
 * @return true if successful
 */
bool mcp23017_read_port(mcp23017_t *dev, mcp23017_port_t port, uint8_t *value);

#endif // MCP23017_H
