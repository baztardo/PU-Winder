#ifndef CONFIG_H
#define CONFIG_H

#include "pico/stdlib.h"

// I2C LCD Configuration
#define I2C_PORT            i2c0
#define I2C_SDA_PIN         0
#define I2C_SCL_PIN         1
#define I2C_BAUDRATE        100000  // 100kHz
#define LCD_ADDRESS         0x27    // Common I2C LCD address (or 0x3F)
#define LCD_COLS            20      // LCD columns (TC2004A-01: 20 chars)
#define LCD_ROWS            4       // LCD rows (TC2004A-01: 4 lines)

// Encoder Configuration
#define ENCODER_A_PIN       4
#define ENCODER_B_PIN       3
#define ENCODER_Z_PIN       25
#define ENCODER_PPR         360     // Pulses per revolution
#define ENCODER_CPR         1440    // Counts per revolution (quadrature)

// Display Update Configuration
#define UPDATE_INTERVAL_MS  100     // Display update interval in ms
#define RPM_SAMPLE_MS       100     // RPM calculation interval in ms

// Global Variables (extern declarations)
extern volatile int32_t encoder_count;
extern volatile int32_t revolution_count;
extern volatile int32_t pulses_this_rev;
extern volatile bool direction_cw;
extern volatile uint32_t last_z_time_us;

extern uint32_t last_display_update;
extern uint32_t last_rpm_update;
extern float current_rpm;
extern volatile int32_t last_count_for_rpm;

#endif // CONFIG_H
