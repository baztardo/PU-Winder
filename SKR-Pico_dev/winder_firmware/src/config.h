// =============================================================================
// config.h - Hardware Configuration and Constants
// Purpose: Central configuration for all hardware pins and system parameters
// =============================================================================

#pragma once

#include <cstdint>

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
#define TRAVERSE_HOME_PIN   14

// Encoder (360 PPR, 1:1 with spindle)
#define ENCODER_A_PIN       4
#define ENCODER_B_PIN       3
#define ENCODER_Z_PIN       25

// TMC2209 UART (Shared bus)
#define TMC_UART_TX_PIN     8
#define TMC_UART_RX_PIN     9
#define TMC_UART_BAUD       115200

// I2C Bus
#define I2C_SDA_PIN         0
#define I2C_SCL_PIN         1
#define I2C_FREQ_HZ         400000

// =============================================================================
// ENCODER SPECIFICATIONS
// =============================================================================
#define ENCODER_PPR         360     // Pulses per revolution
#define ENCODER_CPR         720     // Counts per revolution (quadrature)

// =============================================================================
// MECHANICAL SPECIFICATIONS
// =============================================================================
#define TRAVERSE_PITCH_MM   5.0f    // Lead screw pitch in mm/revolution
#define R_SENSE             0.11f   // TMC2209 sense resistor value

// =============================================================================
// MOTOR CURRENT SETTINGS
// =============================================================================
#define SPINDLE_CURRENT_MA  2800    // Spindle motor RMS current
#define TRAVERSE_CURRENT_MA 500     // Traverse motor RMS current
#define MOTOR_MICROSTEPS    16      // Microstepping setting

// =============================================================================
// TIMING PARAMETERS
// =============================================================================
#define HEARTBEAT_US        100     // ISR frequency: 100μs = 10 kHz
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

// =============================================================================
// DEBUG OPTIONS
// =============================================================================
#define DEBUG_ENABLE_SERIAL     1       // Enable serial debug output
#define DEBUG_PRINT_INTERVAL_MS 1000    // Status print interval