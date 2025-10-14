#pragma once
#include "pico/stdlib.h"
#include "hardware/pio.h"

typedef struct {
    int32_t position;       // pulse count (A/B)
    int32_t revolutions;    // Z count
    int8_t  direction;      // +1 / -1 / 0
    absolute_time_t last_time;
} encoder_t;

void encoder_init(encoder_t *enc, PIO pio, uint sm_ab, uint sm_z,
                  uint pin_a, uint pin_b, uint pin_z, float ab_clkdiv);

void encoder_update(encoder_t *enc, PIO pio, uint sm_ab);
float encoder_get_rpm(encoder_t *enc, uint32_t pulses_per_rev);