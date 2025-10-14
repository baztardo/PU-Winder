#ifndef ENCODER_H
#define ENCODER_H

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"

typedef struct {
    uint8_t pin_a;
    uint8_t pin_b;
    uint8_t pin_z;
    int32_t ppr;
    int32_t cpr;
    PIO pio;
    uint sm;
    uint offset;
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

bool encoder_init(encoder_t *enc, PIO pio, uint8_t pin_a, uint8_t pin_b, 
                  uint8_t pin_z, int32_t ppr, int32_t cpr);
void encoder_reset(encoder_t *enc);
int32_t encoder_get_count(encoder_t *enc);
int32_t encoder_get_revolutions(encoder_t *enc);
int32_t encoder_get_pulses_this_rev(encoder_t *enc);
bool encoder_get_direction(encoder_t *enc);
float encoder_get_rpm(encoder_t *enc);
void encoder_calculate_rpm(encoder_t *enc);
void encoder_process(encoder_t *enc);

extern const int8_t encoder_states[16];

#endif