/**
 * SKR-Pico Encoder Monitor with I2C LCD
 * 
 * This program monitors a 360 PPR quadrature encoder with Z-index
 * and displays:
 * - Revolution count
 * - RPM (positive for CW, negative for CCW)
 * - Pulses per revolution count
 * - Direction (CW/CCW)
 * 
 * Hardware:
 * - SKR-Pico board (RP2040)
 * - I2C LCD (20x4)
 * - 360 PPR Encoder with A/B/Z channels
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "config.h"
#include "src/i2c/i2c_helper.h"
#include "src/drivers/lcd_pcf8574.h"
#include "src/encoder/encoder.h"

// LCD and Encoder objects
lcd_t lcd;
encoder_t encoder;

// Timing variables
uint32_t last_display_update = 0;

void update_display(void);

int main() {
    // Initialize stdio for USB serial debugging
    stdio_init_all();
    
    // Wait for USB serial connection (optional, can comment out)
    sleep_ms(2000);
    
    printf("\n=================================\n");
    printf("SKR-Pico Encoder Monitor\n");
    printf("=================================\n");
    printf("Initializing...\n");
    
    // Initialize I2C using helper library
    i2c_config_t i2c_config = {
        .port = I2C_PORT,
        .sda_pin = I2C_SDA_PIN,
        .scl_pin = I2C_SCL_PIN,
        .baudrate = I2C_BAUDRATE
    };
    
    if (!i2c_helper_init(&i2c_config)) {
        printf("ERROR: Failed to initialize I2C!\n");
        return -1;
    }
    
    printf("I2C initialized on GPIO%d (SDA) and GPIO%d (SCL) at %d Hz\n", 
           I2C_SDA_PIN, I2C_SCL_PIN, I2C_BAUDRATE);
    
    // Scan I2C bus for devices
    uint8_t found_devices[128];
    int device_count = i2c_helper_scan(I2C_PORT, found_devices);
    
    // Initialize LCD
    lcd_init(&lcd, I2C_PORT, LCD_ADDRESS, LCD_COLS, LCD_ROWS);
    lcd_backlight_on(&lcd);
    lcd_clear(&lcd);
    
    printf("LCD initialized at address 0x%02X\n", LCD_ADDRESS);
    
    // Display startup message
    lcd_set_cursor(&lcd, 0, 0);
    lcd_print(&lcd, "Encoder Monitor");
    lcd_set_cursor(&lcd, 0, 1);
    lcd_print(&lcd, "Initializing...");
    sleep_ms(2000);
    
    // Initialize encoder
    encoder_init(&encoder, ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_Z_PIN, 
                 ENCODER_PPR, ENCODER_CPR);
    
    printf("Encoder initialized:\n");
    printf("  A: GPIO%d, B: GPIO%d, Z: GPIO%d\n", ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_Z_PIN);
    printf("  PPR: %d, CPR: %d\n", ENCODER_PPR, ENCODER_CPR);
    
    lcd_clear(&lcd);
    lcd_set_cursor(&lcd, 0, 0);
    lcd_print(&lcd, "Ready!");
    sleep_ms(1000);
    
    printf("Initialization complete!\n");
    printf("Starting main loop...\n\n");
    
    // Main loop
    while (1) {
        // Calculate RPM continuously
        encoder_calculate_rpm(&encoder);
        
        // Update display at regular intervals
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if (current_time - last_display_update >= UPDATE_INTERVAL_MS) {
            last_display_update = current_time;
            update_display();
        }
        
        // Small delay to prevent busy-waiting
        sleep_ms(10);
    }
    
    return 0;
}

void update_display(void) {
    // Get current encoder values
    int32_t revolutions = encoder_get_revolutions(&encoder);
    float rpm = encoder_get_rpm(&encoder);
    int32_t pulses = encoder_get_pulses_this_rev(&encoder);
    bool is_cw = encoder_get_direction(&encoder);
    int32_t total_count = encoder_get_count(&encoder);
    
    // Clear display
    lcd_clear(&lcd);
    
    // Line 1: Revolution count
    lcd_set_cursor(&lcd, 0, 0);
    lcd_print(&lcd, "Rev: ");
    lcd_print_int(&lcd, revolutions);
    
    // Line 2: RPM with direction indicator
    lcd_set_cursor(&lcd, 0, 1);
    lcd_print(&lcd, "RPM: ");
    if (rpm >= 0) {
        lcd_print(&lcd, " ");  // Space for alignment
    }
    lcd_print_float(&lcd, rpm, 1);
    lcd_print(&lcd, " ");
    lcd_print(&lcd, is_cw ? "CW " : "CCW");
    
    // Line 3: Pulses this revolution
    lcd_set_cursor(&lcd, 0, 2);
    lcd_print(&lcd, "Pulses/Rev: ");
    lcd_print_int(&lcd, pulses);
    
    // Line 4: Total count
    lcd_set_cursor(&lcd, 0, 3);
    lcd_print(&lcd, "Count: ");
    lcd_print_int(&lcd, total_count);
    
    // Debug output to USB serial
    printf("Rev: %ld | RPM: %.1f | Dir: %s | Pulses: %ld | Count: %ld\n",
           revolutions, rpm, is_cw ? "CW" : "CCW", pulses, total_count);
}
