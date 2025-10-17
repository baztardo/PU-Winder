// =============================================================================
// config.h - EP-0172 BLDC Configuration
// =============================================================================

#pragma once

// =============================================================================
// BLDC Motor Pins (EP-0172)
// =============================================================================
#define BLDC_PWM_PIN          10    // PWM speed control
#define BLDC_DIR_PIN          11    // Direction control
#define BLDC_BRAKE_PIN        18    // Brake control
#define BLDC_SPEED_PULSE_PIN  19    // Speed pulse input (SC output)

// =============================================================================
// PWM Configuration
// =============================================================================
#define BLDC_PWM_FREQ_HZ      16000  // 16 kHz PWM frequency
#define BLDC_PWM_RESOLUTION   1000   // 1000 steps (0-1000 = 0-100%)

// =============================================================================
// Speed Pulse Configuration
// =============================================================================
#define BLDC_PULSES_PER_REV   6      // 6 edges per full rotation (3 phase BLDC)
                                     // Adjust if your motor is different

// =============================================================================
// BLDC Control Parameters
// =============================================================================
#define BLDC_MIN_PWM_DUTY     50     // Minimum 5% (dead band)
#define BLDC_MAX_PWM_DUTY     950    // Maximum 95% (safety limit)

// =============================================================================
// Available GPIO Pins (NOT USED)
// =============================================================================
// GP0, GP1, GP4, GP20, GP21, GP22, GP23, GP24, GP25, GP28
// Use these for future expansions

// =============================================================================
// Allocated Pins (DO NOT USE)
// =============================================================================
// Display (SPI): GP2, GP3, GP5, GP6, GP7
// Touch (I2C):   GP8, GP9
// Components:    GP12 (RGB), GP13 (Beeper), GP14 (BTN2), GP15 (BTN1)
//                GP16 (LED1), GP17 (LED2), GP26 (Joystick X), GP27 (Joystick Y)