// =============================================================================
// encoder.cpp - Quadrature Encoder Implementation
// =============================================================================

#include "encoder.h"
#include "config.h"
#include "hardware/gpio.h"

Encoder::Encoder()
    : position(0)
    , last_velocity_position(0)
    , last_a(false)
    , last_b(false)
    , last_z(false)
    , z_pulse_detected(false) {
}

void Encoder::init() {
    // Initialize GPIO pins as inputs with pull-ups
    gpio_init(ENCODER_A_PIN);
    gpio_set_dir(ENCODER_A_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_A_PIN);
    
    gpio_init(ENCODER_B_PIN);
    gpio_set_dir(ENCODER_B_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_B_PIN);
    
    gpio_init(ENCODER_Z_PIN);
    gpio_set_dir(ENCODER_Z_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_Z_PIN);
    
    // Read initial state
    last_a = gpio_get(ENCODER_A_PIN);
    last_b = gpio_get(ENCODER_B_PIN);
    last_z = gpio_get(ENCODER_Z_PIN);
}

void Encoder::update() {
    // Read current state
    bool a = gpio_get(ENCODER_A_PIN);
    bool b = gpio_get(ENCODER_B_PIN);
    bool z = gpio_get(ENCODER_Z_PIN);
    
    // Quadrature decoding
    // A changes: if A==B, we're moving forward, else backward
    if (a != last_a) {
        if (a == b) {
            position--;
        } else {
            position++;
        }
    }
    
    // B changes: if A!=B, we're moving forward, else backward
    if (b != last_b) {
        if (a != b) {
            position--;
        } else {
            position++;
        }
    }
    
    // Z index pulse detection (rising edge)
    if (z && !last_z) {
        z_pulse_detected = true;
    }
    
    // Update last state
    last_a = a;
    last_b = b;
    last_z = z;
}

int32_t Encoder::get_position() const {
    return position;
}

void Encoder::set_position(int32_t pos) {
    position = pos;
}

void Encoder::reset() {
    position = 0;
    last_velocity_position = 0;
}

float Encoder::get_revolutions() const {
    return (float)position / (float)ENCODER_CPR;
}

bool Encoder::check_z_pulse() {
    bool detected = z_pulse_detected;
    z_pulse_detected = false;  // Clear flag
    return detected;
}

float Encoder::get_velocity(float dt_seconds) {
    if (dt_seconds <= 0.0f) return 0.0f;
    
    int32_t delta = position - last_velocity_position;
    last_velocity_position = position;
    
    return (float)delta / dt_seconds;
}