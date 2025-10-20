// =============================================================================
// bldc_speed_pulse.h - BLDC Speed Pulse ISR Handler
// Purpose: Count speed pulses from SC output and calculate RPM
// =============================================================================

#pragma once

#include <cstdint>
#include "hardware/gpio.h"

// =============================================================================
// BLDCSpeedPulse Class
// =============================================================================
class BLDCSpeedPulse {
public:
    /**
     * @brief Constructor
     * @param pulse_pin GPIO pin connected to SC speed pulse output
     */
    BLDCSpeedPulse(uint pulse_pin);
    
    /**
     * @brief Initialize GPIO and ISR
     */
    void init();
    
    /**
     * @brief Get current measured RPM
     * @return RPM value (updated on every complete revolution)
     */
    float get_rpm() const;
    
    /**
     * @brief Get pulse frequency in Hz
     * @return Frequency in Hz
     */
    float get_frequency() const;
    
    /**
     * @brief Get total pulse count since init
     * @return Total pulses received
     */
    uint32_t get_pulse_count() const;
    
    /**
     * @brief Get number of complete revolutions
     * @return Revolutions (calculated from pulse count)
     */
    float get_revolutions() const;
    
    /**
     * @brief Set pulses per revolution (for different motor types)
     * @param ppr Pulses per revolution (e.g., 6 for 3-phase BLDC)
     */
    void set_pulses_per_revolution(uint ppr);
    
    /**
     * @brief Reset pulse counters
     */
    void reset();
    
    /**
     * @brief Get smoothed RPM with low-pass filter
     * Reduces jitter in RPM reading
     * @param alpha Filter factor (0.0-1.0, typical 0.1)
     * @return Smoothed RPM value
     */
    float get_smoothed_rpm(float alpha = 0.1f);
    
    /**
     * @brief Print debug information to serial
     */
    void debug_status() const;

private:
    uint pulse_pin;
    volatile uint32_t edge_count;
    volatile uint32_t last_edge_time;
    volatile float measured_rpm;
    volatile float pulse_frequency;
    uint32_t last_rpm_calculation_time;
    uint pulses_per_revolution;
    
    /**
     * @brief Static ISR wrapper (required for Pico SDK)
     */
    static void isr_wrapper(uint gpio, uint32_t events);
    
    /**
     * @brief Instance ISR handler
     */
    void handle_pulse();
};