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
TMC2209_UART tmc_spindle(8, 0x00);   // GPIO8 = X driver socket (spindle)
TMC2209_UART tmc_traverse(9, 0x00);  // GPIO9 = Y driver socket (traverse)
LCDDisplay lcd(i2c0, 0x27, 20, 4);  // 20x4 LCD at address 0x27
Scheduler scheduler(&move_queue, &spindle_encoder);
WindingController winding_controller(&move_queue, &spindle_encoder, &lcd);


void show_tmc_status() {
    uint32_t traverse_status = 0;
    uint32_t spindle_status = 0;
    
    bool traverse_ok = tmc_traverse.readRegister(TMC_REG_DRV_STATUS, &traverse_status, 2000);
    bool spindle_ok = tmc_spindle.readRegister(TMC_REG_DRV_STATUS, &spindle_status, 2000);
    
    lcd.clear();
    lcd.print_at(0, 0, "TMC Status:");
    
    if (!traverse_ok) {
        lcd.print_at(0, 1, "T: COMM FAIL!");
    } else {
        bool t_temp_warn = (traverse_status & (1 << 26)) != 0;
        bool t_temp_shut = (traverse_status & (1 << 27)) != 0;
        
        if (t_temp_shut) {
            lcd.print_at(0, 1, "T: OVERHEAT!!!");
        } else if (t_temp_warn) {
            lcd.print_at(0, 1, "T: Hot Warning");
        } else {
            lcd.print_at(0, 1, "T: OK");
        }
    }
    
    if (!spindle_ok) {
        lcd.print_at(0, 2, "S: COMM FAIL!");
    } else {
        bool s_temp_warn = (spindle_status & (1 << 26)) != 0;
        bool s_temp_shut = (spindle_status & (1 << 27)) != 0;
        
        if (s_temp_shut) {
            lcd.print_at(0, 2, "S: OVERHEAT!!!");
        } else if (s_temp_warn) {
            lcd.print_at(0, 2, "S: Hot Warning");
        } else {
            lcd.print_at(0, 2, "S: OK");
        }
    }
    
    sleep_ms(10000);
}

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
    sleep_ms(1000);
    
    // Start scheduler ISR
    lcd.print_at(0, 3, "Scheduler...");
    if (!scheduler.start(HEARTBEAT_US)) {
        lcd.clear();
        lcd.print_at(0, 0, "ERROR:");
        lcd.print_at(0, 1, "Scheduler failed!");
        while (1) tight_loop_contents();
    }
    //sleep_ms(500);

    sleep_ms(2000);  // Give time to read "Setting Current"
    show_tmc_status();  // This MUST be called!
    
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
    
    // Enable motors FIRST (before UART init)
    move_queue.set_enable(AXIS_SPINDLE, true);
    move_queue.set_enable(AXIS_TRAVERSE, true);
    sleep_ms(100);

    lcd.clear();  // Changed from lcd-> to lcd.
    lcd.print_at(0, 0, "Setting Current");
    
    // Set motor currents
    bool spindle_ok = tmc_spindle.set_rms_current(SPINDLE_CURRENT_MA, R_SENSE);
    bool traverse_ok = tmc_traverse.set_rms_current(TRAVERSE_CURRENT_MA, R_SENSE);

    // Initialize traverse driver with CORRECT formula
    tmc_traverse.init_driver(TRAVERSE_CURRENT_MA, MOTOR_MICROSTEPS);
    
    // Show on LCD what we calculated
    float cs = (32.0f * 1.414f * 0.250f * 0.11f / 0.325f) - 1.0f;
    lcd.printf_at(0, 1, "T: %.0fmA CS=%d", 250.0f, (int)(cs + 0.5f));
    
    tmc_spindle.init_driver(SPINDLE_CURRENT_MA, MOTOR_MICROSTEPS);
    
    cs = (32.0f * 1.414f * 2.8f * 0.11f / 0.180f) - 1.0f;
    lcd.printf_at(0, 2, "S: %.0fmA CS=%d", 2800.0f, (int)(cs + 0.5f));
    
    sleep_ms(3000);

    
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
