// =============================================================================
// main.cpp - Winder Application Main
// Purpose: High-level application logic and coordination
// =============================================================================

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <cstdio>

// Include all our modular libraries
#include "config.h"
#include "tmc2209.h"
#include "encoder.h"
#include "stepcompress.h"
#include "move_queue.h"
#include "scheduler.h"
#include "lcd_display.h"
#include "winding_controller.h"

// =============================================================================
// Global Objects
// =============================================================================
MoveQueue move_queue;
Encoder spindle_encoder;
TMC2209_UART tmc_spindle(uart1, 0);
TMC2209_UART tmc_traverse(uart1, 0);
LCDDisplay lcd(i2c0, 0x27, 20, 4);  // 20x4 LCD at address 0x27
Scheduler scheduler(&move_queue, &spindle_encoder);
WindingController winding_controller(&move_queue, &spindle_encoder, &lcd);

// =============================================================================
// Function Prototypes
// =============================================================================
void init_hardware();
void init_motors();
void setup_winding_parameters();

// =============================================================================
// Main Application
// =============================================================================
int main() {
    // Short delay for hardware stabilization
    sleep_ms(100);
    
    // Initialize all hardware
    init_hardware();
    
    // Initialize LCD and show startup message
    lcd.clear();
    lcd.print_at(0, 0, "Wire Winder v1.0");
    lcd.print_at(0, 1, "Initializing...");
    sleep_ms(1000);
    
    // Initialize motor drivers
    lcd.print_at(0, 2, "Motors...");
    init_motors();
    sleep_ms(500);
    
    // Start scheduler ISR
    lcd.print_at(0, 3, "Scheduler...");
    if (!scheduler.start(HEARTBEAT_US)) {
        lcd.clear();
        lcd.print_at(0, 0, "ERROR:");
        lcd.print_at(0, 1, "Scheduler failed!");
        while (1) tight_loop_contents();
    }
    sleep_ms(500);
    
    // Initialize winding controller
    winding_controller.init();
    sleep_ms(500);
    
    // Setup winding parameters
    setup_winding_parameters();
    
    // Show ready message
    lcd.clear();
    lcd.print_at(0, 0, "System Ready");
    lcd.print_at(0, 1, "Auto-starting in 3s");
    sleep_ms(3000);
    
    // Auto-start winding sequence
    if (winding_controller.start()) {
        lcd.clear();
        lcd.print_at(0, 0, "Starting...");
    }
    
    // Main loop - winding controller runs here
    while (true) {
        winding_controller.update();
        
        // Small delay to prevent tight loop
        sleep_ms(10);
    }
    
    return 0;
}

// =============================================================================
// Hardware Initialization
// =============================================================================
void init_hardware() {
    // Initialize move queue (GPIO pins)
    move_queue.init();
    
    // Initialize encoder
    spindle_encoder.init();
    
    // Initialize I2C bus for LCD
    i2c_init(i2c0, I2C_FREQ_HZ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    // Initialize LCD
    lcd.init();
    lcd.backlight(true);
}

// =============================================================================
// Motor Driver Initialization
// =============================================================================
void init_motors() {
    // Initialize UART communication
    tmc_spindle.begin(TMC_UART_BAUD);
    tmc_traverse.begin(TMC_UART_BAUD);
    sleep_ms(100);
    
    // Initialize spindle driver
    tmc_spindle.init_driver(SPINDLE_CURRENT_MA, MOTOR_MICROSTEPS);
    
    // Initialize traverse driver
    tmc_traverse.init_driver(TRAVERSE_CURRENT_MA, MOTOR_MICROSTEPS);
    
    // Enable motors
    move_queue.set_enable(AXIS_SPINDLE, true);
    move_queue.set_enable(AXIS_TRAVERSE, true);
}

// =============================================================================
// Setup Winding Parameters
// =============================================================================
void setup_winding_parameters() {
    WindingParams params;
    
    // Configure winding job
    params.target_turns = 1000;          // 1000 turns total
    params.spindle_rpm = 300.0f;         // 300 RPM spindle speed
    params.wire_diameter_mm = 0.064f;    // 43 AWG wire (0.064mm)
    params.layer_width_mm = 50.0f;       // 50mm winding width
    params.start_position_mm = 20.0f;    // Start 20mm from home
    params.ramp_time_sec = 3.0f;         // 3 second ramp up/down
    
    // Calculate and set
    params.calculate_layers();
    winding_controller.set_parameters(params);
    
    // Display parameters on LCD
    lcd.clear();
    lcd.print_at(0, 0, "Winding Setup:");
    lcd.printf_at(0, 1, "Turns: %lu", params.target_turns);
    lcd.printf_at(0, 2, "Layers: %lu", params.total_layers);
    lcd.printf_at(0, 3, "RPM: %.0f", params.spindle_rpm);
    sleep_ms(2000);
}