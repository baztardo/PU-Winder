// =============================================================================
// config.h - Hardware Configuration and Constants
// Purpose: Central configuration for all hardware pins and system parameters
// =============================================================================

#pragma once

#include <cstdint>


// NOTE: Version info moved to src/version.h
// Don't add FIRMWARE_VERSION, VERSION_DATE, or VERSION_DESC here!
// They're in version.h now so they don't conflict when you edit config.h


// =============================================================================
// PIN DEFINITIONS (SKR Pico)
// =============================================================================

// Spindle Stepper Motor
#define SPINDLE_STEP_PIN    11
#define SPINDLE_DIR_PIN     10
#define SPINDLE_ENA_PIN     12

// Traverse Stepper Motor
#define TRAVERSE_STEP_PIN   6
#define TRAVERSE_DIR_PIN    5
#define TRAVERSE_ENA_PIN    7
#define TRAVERSE_HOME_PIN   16

// --- Direction invert flags (set to 1 to invert that axis' DIR) ---
#define SPINDLE_DIR_INVERT   1   // FIXED: Spindle DIR pin is inverted!
#define TRAVERSE_DIR_INVERT  0   // set 1 if traverse moves the wrong way
#define ENCODER_INVERT       0   // 1 flips encoder A/B sense (if deltas are "negative" in forward)

// Use PIO for encoder sampling (1) or GPIO polling fallback (0)
#define ENCODER_USE_PIO       1

// Encoder (360 PPR, 1:1 with spindle)
#define ENCODER_A_PIN       3
#define ENCODER_B_PIN       4
#define ENCODER_Z_PIN       25

// TMC2209 UART (Shared bus)
#define TMC_UART_TX_PIN     8
#define TMC_UART_RX_PIN     9
#define TMC_UART_BAUD       115200

// I2C Bus
#define I2C_SDA_PIN         0
#define I2C_SCL_PIN         1
#define I2C_FREQ_HZ         100000 // 400000

// =============================================================================
// ENCODER SPECIFICATIONS
// =============================================================================
#define ENCODER_PPR         360     // Pulses per revolution
#define ENCODER_CPR         1440     // Counts per revolution (quadrature)

// =============================================================================
// MECHANICAL SPECIFICATIONS
// =============================================================================
#define TRAVERSE_PITCH_MM   5.0f    // Lead screw pitch in mm/revolution
#define R_SENSE             0.11f   // TMC2209 sense resistor value

// =============================================================================
// MOTOR CURRENT SETTINGS
// =============================================================================
#define SPINDLE_CURRENT_MA  2800    // Spindle motor RMS current (mA)
#define TRAVERSE_CURRENT_MA 250     // Traverse motor RMS current (mA)

// TMC2209 Hold Current (percentage of run current when stationary)
#define HOLD_CURRENT_PERCENT 30     // 30% of run current when holding
#define POWER_DOWN_DELAY     20      // Delay before reducing to hold current (x 0.1s)

// =============================================================================
// SPEED LIMITS
// =============================================================================
// Maximum steps per second for spindle motor
// Target: 1500 RPM spindle = 750 RPM stepper = 10,000 sps
// Note: Encoder will freeze above ~4000 sps until DMA is implemented
// Motor will run fine - encoder just won't count at high speed (OK for now!)
#define MAX_SPINDLE_SPS  12000.0f    // For 1500 RPM spindle target!

// =============================================================================
// MECHANICAL SETUP (CRITICAL!)
// =============================================================================
// Gear ratios:
//   Stepper (40T) → Spindle (20T) = Speed-up! Spindle goes 2× faster than stepper
//   Encoder (20T) → Spindle (20T) = 1:1 (encoder reads spindle directly)
// Therefore: Stepper RPM = Spindle RPM ÷ 2 (stepper goes SLOWER!)
#define SPINDLE_GEAR_RATIO      0.5f    // Multiply to get stepper RPM from spindle RPM

// =============================================================================
// WINDING PARAMETERS (Easy configuration!)
// =============================================================================
#define WINDING_TARGET_TURNS    1000    // Total turns to wind
#define WINDING_SPINDLE_RPM     1800.0f  // **SPINDLE** speed (RPM) - Start conservative!
#define WINDING_WIRE_DIA_MM     0.064f  // Wire diameter (mm) - 43 AWG
#define WINDING_WIDTH_MM        50.0f   // Winding width (mm)
#define WINDING_START_POS_MM    20.0f   // Start position from home (mm)
#define WINDING_RAMP_TIME_SEC   10.0f   // Ramp up/down time (was 5s - too fast!)

// Speed recommendations (SPINDLE RPM with 2:1 gearing, 4x microstepping):
//   60 RPM   = Safe, tested (stepper @ 120 RPM, 1600 sps) ✅
//  120 RPM   = Good (stepper @ 240 RPM, 3200 sps) ✅
//  150 RPM   = Recommended max for now (stepper @ 300 RPM, 4000 sps)
//  200 RPM   = Pushing it (stepper @ 400 RPM, 5333 sps)
//  240 RPM   = Motor stalls! (stepper @ 480 RPM, 6400 sps) ❌
// Note: Higher speeds require tuning TMC2209 (voltage, current, chopper)

// =============================================================================
// TMC2209 MICROSTEPPING CONFIGURATION
// =============================================================================
// CRITICAL: Different microstepping for each axis!
// Lower microstepping = faster capable, less resolution
// Higher microstepping = slower max speed, more resolution

#define SPINDLE_MICROSTEPS  4       // 4x for spindle (Hardware-configured via MS pins!)
#define TRAVERSE_MICROSTEPS 16      // 16x for traverse (PRECISION - slower but accurate)
#define MOTOR_MICROSTEPS    16      // Legacy/default (kept for compatibility)

// TMC2209 Microstepping values:
// 0 = Full step (1x)
// 1 = Half step (2x)
// 2 = 4x    ← Spindle ACTUALLY uses this (SKR Pico hardware config)
// 3 = 8x
// 4 = 16x   ← Traverse uses this
// 5 = 32x
// 6 = 64x
// 7 = 128x
// 8 = 256x

// =============================================================================
// TIMING PARAMETERS
// =============================================================================
#define HEARTBEAT_US         50     // ISR frequency: 50μs = 20 kHz
#define HEARTBEAT_ENABLE      0     // 0: disable heartbeat toggle, 1: enable
#define STEP_PULSE_US       2       // Step pulse width in microseconds

// =============================================================================
// MOVE QUEUE CONFIGURATION
// =============================================================================
#define MOVE_CHUNKS_CAPACITY 128    // Maximum chunks per axis
#define MAX_ERROR_US        20.0    // Step compression error tolerance

// =============================================================================
// AXIS IDENTIFIERS
// =============================================================================
#define AXIS_SPINDLE        0
#define AXIS_TRAVERSE       1
#define NUM_AXES            2

// =============================================================================
// SAFETY LIMITS
// =============================================================================
#define MAX_TRAVERSE_POS_MM     200.0f  // Maximum traverse position
#define MIN_TRAVERSE_POS_MM     0.0f    // Minimum traverse position
#define HOMING_SPEED_MM_PER_SEC 5.0f    // Homing speed

// =============================================================================
// MOTION PARAMETERS (Defaults)
// =============================================================================
#define DEFAULT_MAX_VELOCITY    1000.0  // steps/sec
#define DEFAULT_ACCELERATION    2000.0  // steps/sec²
#define DEFAULT_JERK            5000.0  // steps/sec³ (future use)

// Traverse Motion Speeds
#define TRAVERSE_HOMING_SPEED   1500    // steps/sec for homing
#define TRAVERSE_RAPID_SPEED    3000    // steps/sec for rapid moves
#define TRAVERSE_RAPID_ACCEL    5000    // steps/sec² for rapid moves
#define TRAVERSE_MIN_WINDING_SPEED 1000 // Minimum speed during winding (steps/sec)

// =============================================================================
// DEBUG OPTIONS
// =============================================================================
#define DEBUG_ENABLE_SERIAL     1       // Enable serial debug output
#define DEBUG_PRINT_INTERVAL_MS 1000    // Status print interval

// =============================================================================
// DIAGNOSTIC LED PINS (FAN outputs with indicator LEDs)
// =============================================================================
#define LED1_PIN    17   // FAN1
#define LED2_PIN    18   // FAN2
#define LED3_PIN    20   // FAN3

// =============================================================================
// Scheduler heartbeat diagnostic
// =============================================================================
#define SCHED_HEARTBEAT_PIN  20    // FAN3 LED
#define SCHED_HEARTBEAT_INTERVAL_MS  500   // Blink every 100 ms
// =============================================================================
// Debug pin (used for single flash pulse diagnostics)
// =============================================================================
#define DEBUG_PIN  17   // FAN1 or any spare pin you prefer
