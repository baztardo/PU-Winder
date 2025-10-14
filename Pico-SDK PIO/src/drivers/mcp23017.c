#include "mcp23017.h"
#include "../i2c/i2c_helper.h"
#include <stdio.h>

bool mcp23017_init(mcp23017_t *dev, i2c_inst_t *i2c_port, uint8_t address) {
    dev->i2c_port = i2c_port;
    dev->address = address;
    
    // Check if device is present
    if (!i2c_helper_device_present(i2c_port, address)) {
        printf("ERROR: MCP23017 not found at address 0x%02X\n", address);
        return false;
    }
    
    // Set all pins to output by default
    i2c_helper_write_register(i2c_port, address, MCP23017_IODIRA, 0x00);
    i2c_helper_write_register(i2c_port, address, MCP23017_IODIRB, 0x00);
    
    printf("MCP23017 initialized at address 0x%02X\n", address);
    return true;
}

bool mcp23017_pin_mode(mcp23017_t *dev, mcp23017_port_t port, uint8_t pin, mcp23017_mode_t mode) {
    if (pin > 7) return false;
    
    uint8_t reg = (port == MCP23017_PORTA) ? MCP23017_IODIRA : MCP23017_IODIRB;
    uint8_t current_dir;
    
    // Read current direction register
    if (!i2c_helper_read_register(dev->i2c_port, dev->address, reg, &current_dir)) {
        return false;
    }
    
    // Modify bit
    if (mode == MCP23017_INPUT) {
        current_dir |= (1 << pin);   // Set bit for input
    } else {
        current_dir &= ~(1 << pin);  // Clear bit for output
    }
    
    // Write back
    return i2c_helper_write_register(dev->i2c_port, dev->address, reg, current_dir);
}

bool mcp23017_digital_write(mcp23017_t *dev, mcp23017_port_t port, uint8_t pin, bool value) {
    if (pin > 7) return false;
    
    uint8_t reg = (port == MCP23017_PORTA) ? MCP23017_OLATA : MCP23017_OLATB;
    uint8_t current_state;
    
    // Read current output latch
    if (!i2c_helper_read_register(dev->i2c_port, dev->address, reg, &current_state)) {
        return false;
    }
    
    // Modify bit
    if (value) {
        current_state |= (1 << pin);   // Set bit
    } else {
        current_state &= ~(1 << pin);  // Clear bit
    }
    
    // Write back
    return i2c_helper_write_register(dev->i2c_port, dev->address, reg, current_state);
}

bool mcp23017_digital_read(mcp23017_t *dev, mcp23017_port_t port, uint8_t pin, bool *value) {
    if (pin > 7 || !value) return false;
    
    uint8_t reg = (port == MCP23017_PORTA) ? MCP23017_GPIOA : MCP23017_GPIOB;
    uint8_t port_state;
    
    // Read port
    if (!i2c_helper_read_register(dev->i2c_port, dev->address, reg, &port_state)) {
        return false;
    }
    
    // Extract bit
    *value = (port_state & (1 << pin)) != 0;
    return true;
}

bool mcp23017_write_port(mcp23017_t *dev, mcp23017_port_t port, uint8_t value) {
    uint8_t reg = (port == MCP23017_PORTA) ? MCP23017_OLATA : MCP23017_OLATB;
    return i2c_helper_write_register(dev->i2c_port, dev->address, reg, value);
}

bool mcp23017_read_port(mcp23017_t *dev, mcp23017_port_t port, uint8_t *value) {
    if (!value) return false;
    
    uint8_t reg = (port == MCP23017_PORTA) ? MCP23017_GPIOA : MCP23017_GPIOB;
    return i2c_helper_read_register(dev->i2c_port, dev->address, reg, value);
}
