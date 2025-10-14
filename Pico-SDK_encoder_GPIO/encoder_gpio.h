#ifndef ENCODER_GPIO_H
#define ENCODER_GPIO_H

#include "pico/stdlib.h"
#include <stdbool.h>

// Encoder structure
typedef struct {
    uint8_t pin_a;
    uint8_t pin_b;
    uint8_t pin_z;
    int32_t ppr;                        // Pulses per revolution
    int32_t cpr;                        // Counts per revolution (PPR × 4 for quadrature)
    
    volatile int32_t count;             // Total count
    volatile int32_t revolution_count;  // Revolution count from Z pulses
    volatile int32_t pulses_this_rev;   // Pulses since last Z pulse
    volatile bool direction_cw;         // true = CW, false = CCW
    volatile uint8_t last_state;        // Last A/B state (for quadrature decoding)
    
    // Z pulse debouncing
    volatile uint64_t last_z_time;
    uint32_t z_debounce_us;             // Debounce time in microseconds
    
    // RPM calculation
    volatile int32_t last_count_for_rpm;
    uint64_t last_rpm_time;
    float current_rpm;
} encoder_gpio_t;

// Initialize encoder with GPIO interrupts
bool encoder_gpio_init(encoder_gpio_t *enc, 
                       uint8_t pin_a, 
                       uint8_t pin_b, 
                       uint8_t pin_z,
                       int32_t ppr, 
                       int32_t cpr);

// Get current values
int32_t encoder_gpio_get_count(encoder_gpio_t *enc);
int32_t encoder_gpio_get_revolutions(encoder_gpio_t *enc);
int32_t encoder_gpio_get_pulses_this_rev(encoder_gpio_t *enc);
bool encoder_gpio_get_direction(encoder_gpio_t *enc);
float encoder_gpio_get_rpm(encoder_gpio_t *enc);

// Calculate RPM (call this regularly, e.g., every 100ms)
void encoder_gpio_calculate_rpm(encoder_gpio_t *enc);

// Reset all counters
void encoder_gpio_reset(encoder_gpio_t *enc);

#endif // ENCODER_GPIO_H