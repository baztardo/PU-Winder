#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "encoder.h"
#include "i2c_helper.h"
#include "lcd_pcf8574.h"
#include "config.h"
#include <stdio.h>

encoder_t encoder;
lcd_t lcd;
uint32_t last_display_update = 0;

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n=================================\n");
    printf("Encoder Monitor - PIO Version\n");
    printf("=================================\n\n");
    
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
    
    printf("Scanning I2C bus...\n");
    uint8_t found_devices[128];
    int device_count = i2c_helper_scan(I2C_PORT, found_devices);
    
    if (device_count == 0) {
        printf("WARNING: No I2C devices found!\n");
    }
    
    printf("Initializing LCD at address 0x%02X...\n", LCD_ADDRESS);
    lcd_init(&lcd, I2C_PORT, LCD_ADDRESS, LCD_COLS, LCD_ROWS);
    
    lcd_clear(&lcd);
    lcd_set_cursor(&lcd, 0, 0);
    lcd_print(&lcd, "Encoder Monitor");
    lcd_set_cursor(&lcd, 0, 1);
    lcd_print(&lcd, "PIO Version");
    lcd_set_cursor(&lcd, 0, 2);
    lcd_print(&lcd, "Initializing...");
    sleep_ms(1500);
    
    // Add this BEFORE encoder_init()
    printf("\n=== PRE-PIO HARDWARE TEST ===\n");
    gpio_init(ENCODER_A_PIN);
    gpio_init(ENCODER_B_PIN);
    gpio_init(ENCODER_Z_PIN);
    gpio_set_dir(ENCODER_A_PIN, GPIO_IN);
    gpio_set_dir(ENCODER_B_PIN, GPIO_IN);
    gpio_set_dir(ENCODER_Z_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_A_PIN);
    gpio_pull_up(ENCODER_B_PIN);
    gpio_pull_up(ENCODER_Z_PIN);

    printf("Manually rotate encoder and watch for changes:\n");
    for (int i = 0; i < 50; i++) {
        bool a = gpio_get(ENCODER_A_PIN);
        bool b = gpio_get(ENCODER_B_PIN);
        bool z = gpio_get(ENCODER_Z_PIN);
        printf("A=%d B=%d Z=%d\n", a, b, z);
        sleep_ms(100);
    }
    printf("=== Did you see changes? If NO, check wiring! ===\n\n");
    sleep_ms(2000);

    // NOW initialize PIO
    printf("Initializing PIO encoder...\n");
    if (!encoder_init(&encoder, pio0, 
                      ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_Z_PIN,
                      ENCODER_PPR, ENCODER_CPR)) {
        printf("ERROR: Failed to initialize PIO encoder!\n");
        lcd_clear(&lcd);
        lcd_set_cursor(&lcd, 0, 0);
        lcd_print(&lcd, "PIO Init Failed!");
        while (1) tight_loop_contents();
    }
    printf("PIO encoder initialized successfully\n");
    printf("  A: GPIO%d, B: GPIO%d, Z: GPIO%d\n", 
           ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_Z_PIN);
    printf("  PPR: %ld, CPR: %ld\n\n", ENCODER_PPR, ENCODER_CPR);
    
    lcd_clear(&lcd);
    lcd_set_cursor(&lcd, 0, 0);
    lcd_print(&lcd, "Ready!");
    sleep_ms(1000);
    
while (1) {
    encoder_process(&encoder);
    encoder_calculate_rpm(&encoder);
    
    // Get encoder values - MOVE THESE TO THE TOP OF THE LOOP
    int32_t count = encoder_get_count(&encoder);
    int32_t revolutions = encoder_get_revolutions(&encoder);
    int32_t pulses = encoder_get_pulses_this_rev(&encoder);
    bool is_cw = encoder_get_direction(&encoder);
    float rpm = encoder_get_rpm(&encoder);
    
    // DEBUG PINS - ADD THIS
    static uint32_t last_pin_check = 0;
    uint32_t time_now = to_ms_since_boot(get_absolute_time());
    if (time_now - last_pin_check > 100) {
        printf("RAW: A=%d B=%d Z=%d | Cnt=%ld\n",
               gpio_get(ENCODER_A_PIN), gpio_get(ENCODER_B_PIN), 
               gpio_get(ENCODER_Z_PIN), count);
        last_pin_check = time_now;
    }
    
    // Update display every UPDATE_INTERVAL_MS
    if (time_now - last_display_update >= UPDATE_INTERVAL_MS) {
        last_display_update = time_now;
            
            printf("Count: %7ld | Rev: %4ld | Pulse: %4ld | RPM: %7.1f %s\n",
                   count, revolutions, pulses, rpm, is_cw ? "CW " : "CCW");
            
            lcd_clear(&lcd);
            
            lcd_set_cursor(&lcd, 0, 0);
            lcd_print(&lcd, "Cnt:");
            lcd_print_int(&lcd, count);
            
            lcd_set_cursor(&lcd, 0, 1);
            lcd_print(&lcd, "Rev:");
            lcd_print_int(&lcd, revolutions);
            lcd_set_cursor(&lcd, 14, 1);
            lcd_print(&lcd, is_cw ? "CW " : "CCW");
            
            lcd_set_cursor(&lcd, 0, 2);
            lcd_print(&lcd, "Pls:");
            lcd_print_int(&lcd, pulses);
            lcd_print(&lcd, "/");
            lcd_print_int(&lcd, ENCODER_CPR);
            
            lcd_set_cursor(&lcd, 0, 3);
            lcd_print(&lcd, "RPM:");
            lcd_print_float(&lcd, rpm, 1);
        }
        
        sleep_ms(10);
    }
    
    return 0;
}
