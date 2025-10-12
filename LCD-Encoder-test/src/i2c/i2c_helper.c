#include "i2c_helper.h"
#include <stdio.h>

bool i2c_helper_init(const i2c_config_t *config) {
    if (!config || !config->port) {
        return false;
    }
    
    // Initialize I2C port
    i2c_init(config->port, config->baudrate);
    
    // Set up GPIO pins for I2C
    gpio_set_function(config->sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(config->scl_pin, GPIO_FUNC_I2C);
    
    // Enable pull-ups
    gpio_pull_up(config->sda_pin);
    gpio_pull_up(config->scl_pin);
    
    return true;
}

int i2c_helper_write(i2c_inst_t *port, uint8_t addr, const uint8_t *data, size_t len, bool nostop) {
    if (!port || !data || len == 0) {
        return PICO_ERROR_GENERIC;
    }
    
    return i2c_write_blocking(port, addr, data, len, nostop);
}

int i2c_helper_read(i2c_inst_t *port, uint8_t addr, uint8_t *data, size_t len, bool nostop) {
    if (!port || !data || len == 0) {
        return PICO_ERROR_GENERIC;
    }
    
    return i2c_read_blocking(port, addr, data, len, nostop);
}

bool i2c_helper_write_byte(i2c_inst_t *port, uint8_t addr, uint8_t data) {
    return i2c_helper_write(port, addr, &data, 1, false) == 1;
}

bool i2c_helper_read_byte(i2c_inst_t *port, uint8_t addr, uint8_t *data) {
    return i2c_helper_read(port, addr, data, 1, false) == 1;
}

bool i2c_helper_write_register(i2c_inst_t *port, uint8_t addr, uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = { reg, data };
    return i2c_helper_write(port, addr, buffer, 2, false) == 2;
}

bool i2c_helper_read_register(i2c_inst_t *port, uint8_t addr, uint8_t reg, uint8_t *data) {
    // Write register address
    if (i2c_helper_write(port, addr, &reg, 1, true) != 1) {
        return false;
    }
    
    // Read data from register
    return i2c_helper_read(port, addr, data, 1, false) == 1;
}

int i2c_helper_scan(i2c_inst_t *port, uint8_t *found_devices) {
    int count = 0;
    
    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
    
    for (int addr = 0; addr < (1 << 7); addr++) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }
        
        // Skip reserved addresses
        if ((addr & 0x78) == 0 || (addr & 0x78) == 0x78) {
            printf("   ");
        } else {
            uint8_t data;
            int ret = i2c_read_blocking(port, addr, &data, 1, false);
            
            if (ret >= 0) {
                printf("%02x ", addr);
                if (found_devices) {
                    found_devices[count] = addr;
                }
                count++;
            } else {
                printf("-- ");
            }
        }
        
        if (addr % 16 == 15) {
            printf("\n");
        }
    }
    
    printf("Found %d device(s)\n\n", count);
    return count;
}

bool i2c_helper_device_present(i2c_inst_t *port, uint8_t addr) {
    uint8_t data;
    int ret = i2c_read_blocking(port, addr, &data, 1, false);
    return ret >= 0;
}
