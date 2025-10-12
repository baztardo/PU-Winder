#include "encoder.h"
#include "pico/time.h"
#include <stdio.h>

// Global encoder instance for interrupt handling
static encoder_t *global_encoder = NULL;

// Quadrature encoding lookup table
const int8_t encoder_states[16] = {
    0,  // 0000: No change
   -1,  // 0001: CCW
    1,  // 0010: CW
    0,  // 0011: Invalid
    1,  // 0100: CW
    0,  // 0101: No change
    0,  // 0110: Invalid
   -1,  // 0111: CCW
   -1,  // 1000: CCW
    0,  // 1001: Invalid
    0,  // 1010: No change
    1,  // 1011: CW
    0,  // 1100: Invalid
    1,  // 1101: CW
   -1,  // 1110: CCW
    0   // 1111: No change
};

void encoder_update(encoder_t *enc) {
    // Read current state of both pins
    uint8_t a_state = gpio_get(enc->pin_a) ? 1 : 0;
    uint8_t b_state = gpio_get(enc->pin_b) ? 1 : 0;
    uint8_t current_state = (a_state << 1) | b_state;
    
    // Combine last and current state to form lookup index
    uint8_t index = (enc->last_state << 2) | current_state;
    
    // Get count change from lookup table
    int8_t change = encoder_states[index];
    
    // Update count
    enc->count += change;
    
    // Update direction and pulse counter
    if (change > 0) {
        enc->direction_cw = true;
        enc->pulses_this_rev++;
    } else if (change < 0) {
        enc->direction_cw = false;
        enc->pulses_this_rev++;
    }
    
    // Store current state for next iteration
    enc->last_state = current_state;
}

void encoder_handle_z_pulse(encoder_t *enc) {
    // Reset pulse counter on Z pulse
    enc->pulses_this_rev = 0;
    
    // Update revolution count based on direction
    if (enc->direction_cw) {
        enc->revolution_count++;
    } else {
        enc->revolution_count--;
    }
    
    enc->last_z_time = time_us_64();
}

// GPIO interrupt callback
void encoder_gpio_callback(uint gpio, uint32_t events) {
    if (global_encoder == NULL) return;
    
    if (gpio == global_encoder->pin_z) {
        // Z pulse (index) interrupt
        if (events & GPIO_IRQ_EDGE_RISE) {
            encoder_handle_z_pulse(global_encoder);
        }
    } else if (gpio == global_encoder->pin_a || gpio == global_encoder->pin_b) {
        // A or B channel interrupt
        encoder_update(global_encoder);
    }
}

void encoder_init(encoder_t *enc, uint8_t pin_a, uint8_t pin_b, uint8_t pin_z, int32_t ppr, int32_t cpr) {
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
    enc->last_count_for_rpm = 0;
    enc->last_rpm_time = 0;
    enc->current_rpm = 0.0f;
    
    // Set global instance
    global_encoder = enc;
    
    // Initialize GPIO pins
    gpio_init(pin_a);
    gpio_init(pin_b);
    gpio_init(pin_z);
    
    gpio_set_dir(pin_a, GPIO_IN);
    gpio_set_dir(pin_b, GPIO_IN);
    gpio_set_dir(pin_z, GPIO_IN);
    
    gpio_pull_up(pin_a);
    gpio_pull_up(pin_b);
    gpio_pull_up(pin_z);
    
    // Read initial state
    uint8_t a_state = gpio_get(pin_a) ? 1 : 0;
    uint8_t b_state = gpio_get(pin_b) ? 1 : 0;
    enc->last_state = (a_state << 1) | b_state;
    
    // Set up interrupts
    gpio_set_irq_enabled_with_callback(pin_a, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_gpio_callback);
    gpio_set_irq_enabled(pin_b, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(pin_z, GPIO_IRQ_EDGE_RISE, true);
}

void encoder_reset(encoder_t *enc) {
    enc->count = 0;
    enc->revolution_count = 0;
    enc->pulses_this_rev = 0;
    enc->last_count_for_rpm = 0;
    enc->current_rpm = 0.0f;
}

int32_t encoder_get_count(encoder_t *enc) {
    return enc->count;
}

int32_t encoder_get_revolutions(encoder_t *enc) {
    return enc->revolution_count;
}

int32_t encoder_get_pulses_this_rev(encoder_t *enc) {
    return enc->pulses_this_rev;
}

bool encoder_get_direction(encoder_t *enc) {
    return enc->direction_cw;
}

float encoder_get_rpm(encoder_t *enc) {
    return enc->current_rpm;
}

void encoder_calculate_rpm(encoder_t *enc) {
    uint64_t current_time = time_us_64();
    uint64_t delta_time = current_time - enc->last_rpm_time;
    
    // Update every 100ms (100000 microseconds)
    if (delta_time >= 100000) {
        int32_t current_count = enc->count;
        int32_t delta_count = current_count - enc->last_count_for_rpm;
        
        // Calculate RPM: (deltaCount / CPR) * (60000000 / deltaTime_us)
        enc->current_rpm = (float)delta_count / enc->cpr * (60000000.0f / delta_time);
        
        // Handle negative RPM for CCW rotation
        if (!enc->direction_cw && enc->current_rpm > 0) {
            enc->current_rpm = -enc->current_rpm;
        }
        
        enc->last_count_for_rpm = current_count;
        enc->last_rpm_time = current_time;
    }
}
