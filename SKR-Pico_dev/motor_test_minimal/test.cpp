// ABSOLUTE MINIMAL MOTOR TEST
// NO queues, NO ISR, NO complex logic
// Just GPIO pulses in a loop

#include "pico/stdlib.h"
#include <cstdio>

// Spindle pins
#define SPINDLE_STEP  11
#define SPINDLE_DIR   10
#define SPINDLE_ENA   12

// Traverse pins
#define TRAVERSE_STEP 6
#define TRAVERSE_DIR  5
#define TRAVERSE_ENA  7

int main() {
    stdio_init_all();
    sleep_ms(3000);
    
    printf("\n\n");
    printf("==================================\n");
    printf("  MINIMAL MOTOR TEST\n");
    printf("  NO QUEUE, NO ISR, PURE GPIO\n");
    printf("==================================\n\n");
    
    // Initialize ALL pins
    printf("Initializing pins...\n");
    gpio_init(SPINDLE_STEP); gpio_set_dir(SPINDLE_STEP, GPIO_OUT); gpio_put(SPINDLE_STEP, 0);
    gpio_init(SPINDLE_DIR);  gpio_set_dir(SPINDLE_DIR, GPIO_OUT);  gpio_put(SPINDLE_DIR, 1);
    gpio_init(SPINDLE_ENA);  gpio_set_dir(SPINDLE_ENA, GPIO_OUT);  gpio_put(SPINDLE_ENA, 0); // Enable LOW
    
    gpio_init(TRAVERSE_STEP); gpio_set_dir(TRAVERSE_STEP, GPIO_OUT); gpio_put(TRAVERSE_STEP, 0);
    gpio_init(TRAVERSE_DIR);  gpio_set_dir(TRAVERSE_DIR, GPIO_OUT);  gpio_put(TRAVERSE_DIR, 1);
    gpio_init(TRAVERSE_ENA);  gpio_set_dir(TRAVERSE_ENA, GPIO_OUT);  gpio_put(TRAVERSE_ENA, 0); // Enable LOW
    
    printf("Pins initialized.\n");
    printf("Enable pins set to 0V (motors enabled)\n");
    printf("Direction pins set HIGH (forward)\n\n");
    
    sleep_ms(500);
    
    // Test forever in a loop
    int test_num = 1;
    while (true) {
        printf("=== TEST #%d ===\n", test_num++);
        
        // Test spindle - 200 pulses (1 revolution) at 10Hz
        printf("SPINDLE: Sending 200 pulses at 10Hz...\n");
        for (int i = 0; i < 200; i++) {
            gpio_put(SPINDLE_STEP, 1);
            busy_wait_us(5);      // 5us high
            gpio_put(SPINDLE_STEP, 0);
            sleep_ms(100);        // 10Hz = 100ms period
            
            if (i % 50 == 0 && i > 0) {
                printf("  Spindle: %d/200 pulses\n", i);
            }
        }
        printf("SPINDLE: Done! Motor should have turned 1 revolution.\n\n");
        
        sleep_ms(1000);
        
        // Test traverse - 200 pulses at 10Hz  
        printf("TRAVERSE: Sending 200 pulses at 10Hz...\n");
        for (int i = 0; i < 200; i++) {
            gpio_put(TRAVERSE_STEP, 1);
            busy_wait_us(5);      // 5us high
            gpio_put(TRAVERSE_STEP, 0);
            sleep_ms(100);        // 10Hz = 100ms period
            
            if (i % 50 == 0 && i > 0) {
                printf("  Traverse: %d/200 pulses\n", i);
            }
        }
        printf("TRAVERSE: Done! Motor should have turned 1 revolution.\n\n");
        
        printf("Waiting 5 seconds before next test...\n\n");
        sleep_ms(5000);
    }
    
    return 0;
}
