// =============================================================================
// encoder.h - Quadrature Encoder Decoder
// Purpose: Track rotary encoder position for closed-loop spindle control
// =============================================================================

#pragma once

#include <cstdint>
#include "hardware/pio.h"

// =============================================================================
// Encoder Class
// =============================================================================
class Encoder {
public:
    /**
     * @brief Constructor
     */
    Encoder();
    
    /**
     * @brief Initialize encoder GPIO pins
     */
    void init();
    
    /**
     * @brief Update encoder state (call from ISR or fast loop)
     * This performs quadrature decoding
     */
    void update();
    
    /**
     * @brief Get current encoder position
     * @return Position in counts
     */
    int32_t get_position() const;
    
    /**
     * @brief Set encoder position to a specific value
     * @param position New position value
     */
    void set_position(int32_t position);
    
    /**
     * @brief Reset encoder position to zero
     */
    void reset();
    
    /**
     * @brief Get position in revolutions
     * @return Position in revolutions (float)
     */
    float get_revolutions() const;
    
    /**
     * @brief Check if Z index pulse was detected
     * @return true if Z pulse detected since last check
     */
    bool check_z_pulse();
    
    /**
     * @brief Get velocity estimate (counts per second)
     * @param dt_seconds Time since last velocity update
     * @return Velocity in counts/second
     */
    float get_velocity(float dt_seconds);

    uint32_t get_isr_hits() const;

    void debug_status() const;

private:
    // Position and diagnostics
    volatile uint32_t isr_hits = 0;
    volatile int32_t position;
    int32_t last_velocity_position;
    bool last_a;
    bool last_b;
    bool last_z;
    bool z_pulse_detected;

    // PIO-based quadrature sampling
    bool pio_initialized = false;
    PIO pio = pio1; // prefer PIO1 to avoid conflicts
    uint sm = 0;
    uint offset = 0;
    uint8_t pio_base_pin = 0;   // min(ENCODER_A_PIN, ENCODER_B_PIN)
    uint8_t a_bit_index = 1;    // bit index within RX sample (0 or 1) for channel A
    uint8_t b_bit_index = 0;    // bit index within RX sample (0 or 1) for channel B
    uint8_t last_state_bits = 0;// 2-bit state encoded as (A<<1)|B
};