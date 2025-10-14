#include "encoder_gpio.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include <stdio.h>
#include <stdlib.h>

// Global encoder pointer for ISRs
static encoder_gpio_t *global_encoder = NULL;

// Quadrature decoding lookup table
static const int8_t quadrature_table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

// Simple, fast interrupt handler
static void encoder_gpio_irq_handler(uint gpio, uint32_t events) {
    if (global_encoder == NULL) return;
    
    // Z PULSE - simplest possible handling
    if (gpio == global_encoder->pin_z) {
        uint64_t now = time_us_64();
        if (now - global_encoder->last_z_time > 10000) {  // 10ms debounce
            global_encoder->pulses_this_rev = 0;
            if (global_encoder->direction_cw) {
                global_encoder->revolution_count++;
            } else {
                global_encoder->revolution_count--;
            }
            global_encoder->last_z_time = now;
        }
        return;
    }
    
    // A/B QUADRATURE - as fast as possible
    uint32_t gpio_state = gpio_get_all();
    uint8_t current_state = ((gpio_state >> global_encoder->pin_a) & 1) << 1 |
                            ((gpio_state >> global_encoder->pin_b) & 1);
    
    uint8_t index = (global_encoder->last_state << 2) | current_state;
    int8_t delta = quadrature_table[index];
    
    if (delta != 0) {
        global_encoder->count += delta;
        global_encoder->direction_cw = (delta > 0);
        global_encoder->pulses_this_rev += delta;
        
        // Simple bounds check
        if (global_encoder->pulses_this_rev < 0) {
            global_encoder->pulses_this_rev += global_encoder->cpr;
        } else if (global_encoder->pulses_this_rev >= global_encoder->cpr) {
            global_encoder->pulses_this_rev -= global_encoder->cpr;
        }
    }
    
    global_encoder->last_state = current_state;
}

bool encoder_gpio_init(encoder_gpio_t *enc, 
                       uint8_t pin_a, uint8_t pin_b, uint8_t pin_z,
                       int32_t ppr, int32_t cpr) {
    enc->pin_a = pin_a;
    enc->pin_b = pin_b;
    enc->pin_z = pin_z;
    enc->ppr = ppr;
    enc->cpr = cpr;
    enc->count = 0;
    enc->revolution_count = 0;
    enc->pulses_this_rev = 0;
    enc->direction_cw = true;
    enc->last_z_time = 0;
    enc->z_debounce_us = 10000;
    enc->last_count_for_rpm = 0;
    enc->last_rpm_time = 0;
    enc->current_rpm = 0.0f;
    
    global_encoder = enc;
    
    gpio_init(pin_a);
    gpio_init(pin_b);
    gpio_init(pin_z);
    gpio_set_dir(pin_a, GPIO_IN);
    gpio_set_dir(pin_b, GPIO_IN);
    gpio_set_dir(pin_z, GPIO_IN);
    gpio_pull_up(pin_a);
    gpio_pull_up(pin_b);
    gpio_pull_up(pin_z);
    
    enc->last_state = ((gpio_get(pin_a) ? 1 : 0) << 1) | (gpio_get(pin_b) ? 1 : 0);
    
    gpio_set_irq_enabled_with_callback(pin_a, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, 
                                       true, &encoder_gpio_irq_handler);
    gpio_set_irq_enabled(pin_b, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(pin_z, GPIO_IRQ_EDGE_RISE, true);
    
    printf("GPIO encoder initialized: A=GPIO%d B=GPIO%d Z=GPIO%d\n", pin_a, pin_b, pin_z);
    
    return true;
}

void encoder_gpio_calculate_rpm(encoder_gpio_t *enc) {
    uint64_t current_time = time_us_64();
    uint64_t delta_time = current_time - enc->last_rpm_time;
    
    if (delta_time >= 100000) {
        int32_t current_count = enc->count;
        int32_t delta_count = current_count - enc->last_count_for_rpm;
        enc->current_rpm = (float)delta_count / enc->cpr * (60000000.0f / delta_time);
        enc->last_count_for_rpm = current_count;
        enc->last_rpm_time = current_time;
    }
}

void encoder_gpio_reset(encoder_gpio_t *enc) {
    enc->count = 0;
    enc->revolution_count = 0;
    enc->pulses_this_rev = 0;
    enc->last_count_for_rpm = 0;
    enc->current_rpm = 0.0f;
}

int32_t encoder_gpio_get_count(encoder_gpio_t *enc) { return enc->count; }
int32_t encoder_gpio_get_revolutions(encoder_gpio_t *enc) { return enc->revolution_count; }
int32_t encoder_gpio_get_pulses_this_rev(encoder_gpio_t *enc) { return enc->pulses_this_rev; }
bool encoder_gpio_get_direction(encoder_gpio_t *enc) { return enc->direction_cw; }
float encoder_gpio_get_rpm(encoder_gpio_t *enc) { return enc->current_rpm; }