// MINIMAL MOTOR TEST - Bypasses all complex logic
// This ONLY tests if GPIO step pulses make motors move

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <cstdio>

// Pin definitions from config.h
#define SPINDLE_STEP_PIN    11
#define SPINDLE_DIR_PIN     10
#define SPINDLE_ENA_PIN     12
#define TRAVERSE_STEP_PIN   6
#define TRAVERSE_DIR_PIN    5
#define TRAVERSE_ENA_PIN    7

void simple_step_pulse(uint pin) {
    gpio_put(pin, 1);
    busy_wait_us(2);  // 2us pulse
    gpio_put(pin, 0);
}

void test_motor(uint step_pin, uint dir_pin, uint ena_pin, const char* name) {
    printf("\n=== Testing %s Motor ===\n", name);
    
    // Initialize pins
    gpio_init(step_pin);
    gpio_set_dir(step_pin, GPIO_OUT);
    gpio_put(step_pin, 0);
    
    gpio_init(dir_pin);
    gpio_set_dir(dir_pin, GPIO_OUT);
    gpio_put(dir_pin, 1);  // Forward
    
    gpio_init(ena_pin);
    gpio_set_dir(ena_pin, GPIO_OUT);
    gpio_put(ena_pin, 0);  // Enable (active low)
    
    sleep_ms(100);
    
    // Generate 200 step pulses slowly (1 revolution)
    printf("Sending 200 step pulses at 100Hz...\n");
    for (int i = 0; i < 200; i++) {
        simple_step_pulse(step_pin);
        sleep_ms(10);  // 100Hz = 10ms period
        
        if (i % 50 == 0) {
            printf("  Step %d/200\n", i);
        }
    }
    
    printf("Done! Motor should have rotated 1 revolution.\n");
    sleep_ms(1000);
}

int main() {
    stdio_init_all();
    sleep_ms(2000);  // Wait for USB serial
    
    printf("\n\n");
    printf("==========================================\n");
    printf("   MINIMAL MOTOR TEST - DIRECT GPIO\n");
    printf("==========================================\n");
    printf("This test bypasses all complex logic.\n");
    printf("If motors don't move, it's hardware.\n");
    printf("==========================================\n\n");
    
    // Test spindle motor
    test_motor(SPINDLE_STEP_PIN, SPINDLE_DIR_PIN, SPINDLE_ENA_PIN, "SPINDLE");
    
    // Test traverse motor  
    test_motor(TRAVERSE_STEP_PIN, TRAVERSE_DIR_PIN, TRAVERSE_ENA_PIN, "TRAVERSE");
    
    printf("\n==========================================\n");
    printf("   TESTS COMPLETE\n");
    printf("==========================================\n");
    printf("If motors moved: Hardware OK, check main firmware\n");
    printf("If motors didn't move: Hardware issue\n");
    printf("==========================================\n\n");
    
    // Blink LED forever to show test is done
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    while (true) {
        gpio_put(25, 1);
        sleep_ms(500);
        gpio_put(25, 0);
        sleep_ms(500);
    }
    
    return 0;
}
