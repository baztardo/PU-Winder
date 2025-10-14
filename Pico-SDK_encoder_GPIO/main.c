#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "encoder_gpio.h"
#include "i2c_helper.h"
#include "lcd_pcf8574.h"
#include "config.h"
#include <stdio.h>

// Global variables
encoder_gpio_t encoder;
lcd_t lcd;
uint32_t last_display_update = 0;

int main() {
    // Initialize stdio for USB serial debugging
    stdio_init_all();
    sleep_ms(2000);
    
    // Turn on GPIO 21 (for future use)
    gpio_init(21);
    gpio_set_dir(21, GPIO_OUT);
    gpio_put(21, 1);
    
    printf("\n=================================\n");
    printf("Encoder Monitor - GPIO + LCD\n");
    printf("=================================\n\n");
    
    // Initialize I2C for LCD
    i2c_config_t i2c_config = {
        .port = I2C_PORT,
        .sda_pin = I2C_SDA_PIN,
        .scl_pin = I2C_SCL_PIN,
        .baudrate = I2C_BAUDRATE
    };
    
    if (!i2c_helper_init(&i2c_config)) {
        printf("ERROR: Failed to initialize I2C!\n");
        while (1) tight_loop_contents();
    }
    printf("I2C initialized successfully\n");
    
    // Scan I2C bus for devices
    printf("Scanning I2C bus...\n");
    uint8_t found_devices[128];
    int device_count = i2c_helper_scan(I2C_PORT, found_devices);
    
    if (device_count == 0) {
        printf("WARNING: No I2C devices found!\n");
    }
    
    // Initialize LCD
    printf("Initializing LCD at address 0x%02X...\n", LCD_ADDRESS);
    lcd_init(&lcd, I2C_PORT, LCD_ADDRESS, LCD_COLS, LCD_ROWS);
    
    // Display startup message
    lcd_clear(&lcd);
    lcd_set_cursor(&lcd, 0, 0);
    lcd_print(&lcd, "Encoder Monitor");
    lcd_set_cursor(&lcd, 0, 1);
    lcd_print(&lcd, "GPIO Version");
    lcd_set_cursor(&lcd, 0, 2);
    lcd_print(&lcd, "Initializing...");
    sleep_ms(1500);
    
    // Initialize GPIO encoder
    printf("Initializing GPIO encoder...\n");
    if (!encoder_gpio_init(&encoder,
                           ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_Z_PIN,
                           ENCODER_PPR, ENCODER_CPR)) {
        printf("ERROR: Failed to initialize GPIO encoder!\n");
        lcd_clear(&lcd);
        lcd_set_cursor(&lcd, 0, 0);
        lcd_print(&lcd, "Encoder Init Fail!");
        while (1) tight_loop_contents();
    }
    printf("GPIO encoder initialized successfully\n");
    printf("  A: GPIO%d, B: GPIO%d, Z: GPIO%d\n", 
           ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_Z_PIN);
    printf("  PPR: %d, CPR: %d\n\n", ENCODER_PPR, ENCODER_CPR);
    
    // Display ready message
    lcd_clear(&lcd);
    lcd_set_cursor(&lcd, 0, 0);
    lcd_print(&lcd, "Ready!");
    sleep_ms(1000);
    
    // Main loop
    while (1) {
        // Calculate RPM
        encoder_gpio_calculate_rpm(&encoder);
        
        // Get encoder values
        int32_t count = encoder_gpio_get_count(&encoder);
        int32_t revolutions = encoder_gpio_get_revolutions(&encoder);
        int32_t pulses = encoder_gpio_get_pulses_this_rev(&encoder);
        bool is_cw = encoder_gpio_get_direction(&encoder);
        float rpm = encoder_gpio_get_rpm(&encoder);
        
        // Update display every UPDATE_INTERVAL_MS
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_display_update >= UPDATE_INTERVAL_MS) {
            last_display_update = now;
            
            // Print to serial
            printf("Count: %7ld | Rev: %4ld | Pulse: %4ld | RPM: %7.1f %s\n",
                   count, revolutions, pulses, rpm, is_cw ? "CW " : "CCW");
            
            // Update LCD
            lcd_clear(&lcd);
            
            // Line 1: Count
            lcd_set_cursor(&lcd, 0, 0);
            lcd_print(&lcd, "Cnt:");
            lcd_print_int(&lcd, count);
            
            // Line 2: Revolutions and Direction
            lcd_set_cursor(&lcd, 0, 1);
            lcd_print(&lcd, "Rev:");
            lcd_print_int(&lcd, revolutions);
            lcd_set_cursor(&lcd, 14, 1);
            lcd_print(&lcd, is_cw ? "CW " : "CCW");
            
            // Line 3: Pulses this revolution
            lcd_set_cursor(&lcd, 0, 2);
            lcd_print(&lcd, "Pls:");
            lcd_print_int(&lcd, pulses);
            lcd_print(&lcd, "/");
            lcd_print_int(&lcd, ENCODER_CPR);
            
            // Line 4: RPM (always positive since direction is shown)
            lcd_set_cursor(&lcd, 0, 3);
            lcd_print(&lcd, "RPM:");
            lcd_print_float(&lcd, rpm < 0 ? -rpm : rpm, 1);
        }
        
        // Small delay to prevent overwhelming the system
        sleep_ms(10);
    }
    
    return 0;
}