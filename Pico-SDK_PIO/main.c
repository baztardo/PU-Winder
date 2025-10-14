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
    
    // Optional: quick scan (quiet output)
    uint8_t found_devices[128];
    (void)i2c_helper_scan(I2C_PORT, found_devices);
    
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
    printf("\n=== PRE-PIO HARDWARE TEST (A/B/Z) ===\n");
    gpio_init(ENCODER_A_PIN);
    gpio_init(ENCODER_B_PIN);
    gpio_init(ENCODER_Z_PIN);
    gpio_set_dir(ENCODER_A_PIN, GPIO_IN);
    gpio_set_dir(ENCODER_B_PIN, GPIO_IN);
    gpio_set_dir(ENCODER_Z_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_A_PIN);
    gpio_pull_up(ENCODER_B_PIN);
    gpio_pull_up(ENCODER_Z_PIN);

    printf("Rotate encoder; confirm A/B/Z toggle:\n");
    for (int i = 0; i < 50; i++) {
        bool a = gpio_get(ENCODER_A_PIN);
        bool b = gpio_get(ENCODER_B_PIN);
        bool z = gpio_get(ENCODER_Z_PIN);
        printf("A=%d B=%d Z=%d\n", a, b, z);
        sleep_ms(100);
    }
    printf("=== A/B/Z seen? If NO, check wiring! ===\n\n");
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
    printf("PIO encoder initialized. A:%d B:%d Z:%d | PPR:%ld CPR:%ld\n\n",
           ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_Z_PIN, ENCODER_PPR, ENCODER_CPR);
    
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
    
    // Quiet mode: remove periodic RAW pin prints
    
    // Update display every UPDATE_INTERVAL_MS
    if (time_now - last_display_update >= UPDATE_INTERVAL_MS) {
        last_display_update = time_now;
            
            printf("Cnt:%ld Rev:%ld Pls:%ld RPM:%.1f %s\n",
                   count, revolutions, pulses, rpm, is_cw ? "CW" : "CCW");
            
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
