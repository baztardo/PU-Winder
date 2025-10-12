#ifndef ENCODER_H
#define ENCODER_H

#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Encoder structure
typedef struct {
    uint8_t pin_a;
    uint8_t pin_b;
    uint8_t pin_z;
    int32_t ppr;
    int32_t cpr;
    
    volatile int32_t count;
    volatile int32_t revolution_count;
    volatile int32_t pulses_this_rev;
    volatile bool direction_cw;
    volatile uint64_t last_z_time;
    volatile int32_t last_count_for_rpm;
    
    uint64_t last_rpm_time;
    float current_rpm;
    
    volatile uint8_t last_state;
} encoder_t;

// Function prototypes
void encoder_init(encoder_t *enc, uint8_t pin_a, uint8_t pin_b, uint8_t pin_z, int32_t ppr, int32_t cpr);
void encoder_reset(encoder_t *enc);
int32_t encoder_get_count(encoder_t *enc);
int32_t encoder_get_revolutions(encoder_t *enc);
int32_t encoder_get_pulses_this_rev(encoder_t *enc);
bool encoder_get_direction(encoder_t *enc);
float encoder_get_rpm(encoder_t *enc);
void encoder_calculate_rpm(encoder_t *enc);

// Internal functions
void encoder_update(encoder_t *enc);
void encoder_handle_z_pulse(encoder_t *enc);

// Quadrature state lookup table
extern const int8_t encoder_states[16];

#endif // ENCODER_H
