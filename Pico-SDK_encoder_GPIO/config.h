#ifndef CONFIG_H
#define CONFIG_H

#include "pico/stdlib.h"

// I2C LCD Configuration
#define I2C_PORT            i2c0
#define I2C_SDA_PIN         0
#define I2C_SCL_PIN         1
#define I2C_BAUDRATE        100000
#define LCD_ADDRESS         0x27
#define LCD_COLS            20
#define LCD_ROWS            4

// Encoder Configuration
#define ENCODER_A_PIN       3       // Make sure these match your wiring!
#define ENCODER_B_PIN       4
#define ENCODER_Z_PIN       25
#define ENCODER_PPR         360     // From your encoder spec
#define ENCODER_CPR         1440    // PPR × 4 for quadrature

// Display Update Configuration
#define UPDATE_INTERVAL_MS  200     // Update LCD every 100ms
#define RPM_SAMPLE_MS       100     // RPM sample rate

#endif // CONFIG_H