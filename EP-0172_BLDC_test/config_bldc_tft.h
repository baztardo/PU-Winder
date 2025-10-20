// =============================================================================
// config_bldc_tft.h - Configuration for BLDC + TFT Project
// =============================================================================
// Customize pin assignments and parameters here
// =============================================================================

#pragma once

#include <cstdint>

// =============================================================================
// SPI Configuration (for TFT Display)
// =============================================================================

// SPI Instance
#define SPI_PORT        spi0

// SPI Pins (3-wire: SCK, MOSI)
#define SPI_SCK_PIN     18    // GPIO 18 (RP2040 SPI0 SCK)
#define SPI_MOSI_PIN    19    // GPIO 19 (RP2040 SPI0 TX)
#define SPI_MISO_PIN    16    // GPIO 16 (RP2040 SPI0 RX) - optional for read
#define SPI_FREQ_HZ     62500000  // 62.5 MHz (safe for most ST7789)

// =============================================================================
// TFT Display Configuration (ST7789)
// =============================================================================

// Control Pins
#define TFT_CS_PIN      17    // Chip Select (GPIO 17)
#define TFT_DC_PIN      20    // Data/Command (GPIO 20)
#define TFT_RST_PIN     21    // Reset (GPIO 21)

// Display Size
#define TFT_WIDTH       240   // Display width in pixels
#define TFT_HEIGHT      320   // Display height in pixels

// Display Colors (16-bit RGB565)
#define COLOR_BLACK     0x0000
#define COLOR_WHITE     0xFFFF
#define COLOR_RED       0xF800
#define COLOR_GREEN     0x07E0
#define COLOR_BLUE      0x001F
#define COLOR_CYAN      0x07FF
#define COLOR_MAGENTA   0xF81F
#define COLOR_YELLOW    0xFFE0

// =============================================================================
// BLDC Motor PWM Configuration
// =============================================================================

// PWM Output Pin
#define BLDC_PWM_PIN        2       // GPIO 2
#define BLDC_PWM_SLICE      1       // PWM slice (auto-determined from pin)
#define BLDC_PWM_FREQ_HZ    16000   // 16 kHz (standard for motor drivers)

// PWM Duty Range
#define BLDC_MIN_DUTY       0       // Minimum duty (0% = stop)
#define BLDC_MAX_DUTY       1000    // Maximum duty (1000 = 100%)
#define BLDC_DEAD_BAND      50      // Minimum speed threshold (avoid stall)

// =============================================================================
// BLDC Motor Control Pins
// =============================================================================

// Direction Control (forward/reverse)
#define BLDC_DIR_PIN        26      // GPIO 26
#define BLDC_DIR_FORWARD    1       // GPIO high = forward
#define BLDC_DIR_REVERSE    0       // GPIO low = reverse

// Brake Control (free-wheel/brake)
#define BLDC_BRAKE_PIN      27      // GPIO 27
#define BLDC_BRAKE_ON       1       // GPIO high = brake
#define BLDC_BRAKE_OFF      0       // GPIO low = free-wheel

// =============================================================================
// Hall Sensor Inputs (for speed measurement)
// =============================================================================

// Hall Sensor Pins
#define BLDC_HALL_A_PIN     3       // GPIO 3 - Phase A
#define BLDC_HALL_B_PIN     4       // GPIO 4 - Phase B
#define BLDC_HALL_C_PIN     5       // GPIO 5 - Phase C

// Hall Sensor Configuration
#define HALL_DEBOUNCE_US    100     // Minimum time between edges (µs)
#define HALL_EDGES_PER_REV  6       // 6 edges = 1 revolution (3 sensors × 2 edges)

// =============================================================================
// BLDC Motor Parameters
// =============================================================================

// Speed Control
#define BLDC_RAMP_TIME_MS   2000    // Default ramp time (ms)
#define BLDC_RAMP_STEP_MS   50      // Ramp step interval (ms)

// Estimated RPM Calculation
// RPM = (60,000 * HALL_FREQ_HZ) / HALL_EDGES_PER_REV
// For a 3-pole motor at 3000 RPM max:
// Hall frequency at max = (3000 / 60) × 6 = 300 Hz
#define MAX_ESTIMATED_RPM   3000    // For display calculations

// =============================================================================
// Display Update Rates
// =============================================================================

#define DISPLAY_UPDATE_MS   500     // Update TFT every 500ms
#define SERIAL_UPDATE_MS    200     // Serial debug output every 200ms

// =============================================================================
// Debug Configuration
// =============================================================================

#define DEBUG_UART          uart0   // Serial port for debug output
#define DEBUG_UART_BAUD     115200  // Serial baud rate
#define DEBUG_PRINT_ENABLE  1       // Set to 0 to disable printf()
#define DEBUG_VERBOSE       0       // Set to 1 for detailed ISR debug

// =============================================================================
// Macro Helpers
// =============================================================================

// Convert percentage (0-100) to PWM duty (0-1000)
#define PERCENT_TO_DUTY(pct)  ((pct * 1000) / 100)

// Convert PWM duty (0-1000) to percentage
#define DUTY_TO_PERCENT(duty) ((duty * 100) / 1000)

// Clamp value between min and max
#define CLAMP(val, min, max) \
    ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

// =============================================================================
// Board-Specific Notes
// =============================================================================

/*
 * SKR-Pico Board Layout:
 * ═══════════════════════
 * 
 * Top edge (left to right):
 * GND - 3V3 - GND - GPIO28(ADC2) - GPIO27 - GPIO26 - GND - GPIO20 - GPIO19 - GPIO18
 * 
 * Bottom edge (left to right):
 * GPIO17 - GPIO16 - GPIO15 - GPIO14 - GPIO13 - GPIO12 - GPIO11 - GPIO10 - GPIO9 - GPIO8
 * GPIO7 - GPIO6 - GPIO5 - GPIO4 - GPIO3 - GPIO2 - GPIO1(SCL) - GPIO0(SDA)
 * 
 * Available PWM Slices:
 * GPIO0/GPIO1:   PWM_SLICE 0
 * GPIO2/GPIO3:   PWM_SLICE 1 ✓ (BLDC_PWM_PIN uses this)
 * GPIO4/GPIO5:   PWM_SLICE 2
 * GPIO6/GPIO7:   PWM_SLICE 3
 * GPIO8/GPIO9:   PWM_SLICE 4
 * GPIO10/GPIO11: PWM_SLICE 5
 * GPIO12/GPIO13: PWM_SLICE 6
 * GPIO14/GPIO15: PWM_SLICE 7
 * GPIO16/GPIO17: PWM_SLICE 8
 * GPIO18/GPIO19: PWM_SLICE 9
 * GPIO20/GPIO21: PWM_SLICE 10
 * GPIO26/GPIO27: PWM_SLICE 13
 * GPIO28:        PWM_SLICE 14
 * 
 * SPI Instances:
 * SPI0: SCK=GPIO18, TX=GPIO19, RX=GPIO16, CSn=any GPIO
 * SPI1: SCK=GPIO10, TX=GPIO11, RX=GPIO8,  CSn=any GPIO
 * 
 * I2C Instances (NOT used in this project):
 * I2C0: SDA=GPIO0, SCL=GPIO1
 * I2C1: SDA=GPIO2, SCL=GPIO3, or SDA=GPIO26, SCL=GPIO27
 */

// =============================================================================
// Pin Conflict Check (compile-time verification)
// =============================================================================

#if (SPI_SCK_PIN == BLDC_PWM_PIN) || \
    (SPI_MOSI_PIN == BLDC_PWM_PIN) || \
    (SPI_SCK_PIN == BLDC_DIR_PIN) || \
    (SPI_MOSI_PIN == BLDC_DIR_PIN)
#error "PIN CONFLICT: SPI and BLDC pins overlap!"
#endif

#if (BLDC_PWM_PIN == BLDC_DIR_PIN) || \
    (BLDC_PWM_PIN == BLDC_BRAKE_PIN) || \
    (BLDC_DIR_PIN == BLDC_BRAKE_PIN)
#error "PIN CONFLICT: BLDC pins overlap!"
#endif

#if (BLDC_HALL_A_PIN == BLDC_PWM_PIN) || \
    (BLDC_HALL_B_PIN == BLDC_PWM_PIN) || \
    (BLDC_HALL_C_PIN == BLDC_PWM_PIN)
#error "PIN CONFLICT: BLDC hall sensor overlaps with motor control!"
#endif

// =============================================================================
// Default Test Parameters
// =============================================================================

// Initial motor state
#define MOTOR_INIT_SPEED    0       // Start stopped (0-1000)
#define MOTOR_INIT_DIR      1       // Start forward (1 = forward)
#define MOTOR_INIT_BRAKE    1       // Start with brake on (1 = brake)

// Automatic test mode (if enabled)
#define AUTO_TEST_ENABLED   0       // Set to 1 to auto-run test sequence
#define AUTO_TEST_DURATION  30000   // 30 seconds total test time

#endif // CONFIG_BLDC_TFT_H