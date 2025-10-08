#pragma once
#include "hardware/uart.h"
#include <cstdint>

class TMC2209_UART {
public:
    // Hardware UART (e.g. SKR 1.3/1.4)
    TMC2209_UART(uart_inst_t* uart_port,
                 int tx_pin, int rx_pin,
                 uint8_t slave_addr);

    // Software single-wire (e.g. SKR Pico v1.0) - USE THIS ONE
    TMC2209_UART(uint8_t gpio_pin, uint8_t slave_addr);

    void writeRegister(uint8_t reg, uint32_t data);
    bool readRegister(uint8_t reg, uint32_t &out);
    void testRead();
    
    // Helper functions you'll need
    bool init_driver(float current_ma, uint8_t microsteps = 16);
    bool set_rms_current(float rms_ma, float r_sense);
    bool set_ihold_irun(uint8_t ihold, uint8_t irun, uint8_t ihold_delay);
    bool set_microsteps(uint8_t microsteps);
    bool enable_stealthchop(bool enable);
    bool get_driver_status(uint32_t* status);
    bool is_stalled();
    bool is_overtemp();

private:
    uart_inst_t* uart_inst;
    int tx, rx;
    bool pin_mode;   // true = bit-banged single-wire mode
    uint8_t addr;
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