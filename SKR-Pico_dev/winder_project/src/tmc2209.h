#pragma once
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <cstdint>

class TMC2209_UART {
public:
    // Constructors
    TMC2209_UART(uart_inst_t* uart, int tx_pin, int rx_pin, uint8_t slave_addr);
    TMC2209_UART(uint8_t tx_pin, uint8_t rx_pin);

    // Initialization
    bool begin(uint32_t baud = 115200);

    // Core TMC functions
    bool readRegister(uint8_t reg, uint32_t &value);
    bool writeRegister(uint8_t reg, uint32_t value);
    bool set_rms_current(float rms_ma, float r_sense);
    bool set_ihold_irun(uint8_t ihold, uint8_t irun, uint8_t iholddelay);
    void testRead();
    bool set_microsteps(uint8_t microsteps);
    bool enable_stealthchop(bool enable);
    bool init_driver(float current_ma, uint8_t microsteps);
    bool get_driver_status(uint32_t* status);
    bool is_stalled();
    bool is_overtemp();


private:
    // --- Hardware and config ---
    uart_inst_t* uart_inst;
    int tx;
    int rx;
    bool pin_mode;
    uint8_t slave_addr;

    // --- Internal helpers ---
    uint8_t crc8(const uint8_t* data, size_t len) {
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
};


// Register addresses
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