#include "encoder.h"
#include "encoder.pio.h"
#include "pico/time.h"
#include <stdio.h>
#include <stdlib.h>

static encoder_t *global_encoder = NULL;

const int8_t encoder_states[16] = {
    0, +1, -1, 0,   // prev=00
    -1, 0, 0, +1,   // prev=01
    +1, 0, 0, -1,   // prev=10
    0, -1, +1, 0    // prev=11
};

static void encoder_z_handler(uint gpio, uint32_t events) {
    if (global_encoder == NULL) return;
    if (gpio != global_encoder->pin_z) return;
    
    uint64_t current_time = time_us_64();
    if (current_time - global_encoder->last_z_time < 15000) {
        return;
    }
    
    static int32_t last_z_count = 0;
    int32_t counts_since_last = labs(global_encoder->count - last_z_count);
    
    if (counts_since_last < (global_encoder->cpr * 3 / 5)) {
        return;
    }
    
    printf("Z pulse! Count: %ld (delta: %ld)\n", 
           global_encoder->count, counts_since_last);
    
    last_z_count = global_encoder->count;
    global_encoder->last_z_time = current_time;
    global_encoder->pulses_this_rev = 0;
    
    if (global_encoder->direction_cw) {
        global_encoder->revolution_count++;
    } else {
        global_encoder->revolution_count--;
    }
}

bool encoder_init(encoder_t *enc, PIO pio, uint8_t pin_a, uint8_t pin_b, 
                  uint8_t pin_z, int32_t ppr, int32_t cpr) {
    enc->pin_a = pin_a;
    enc->pin_b = pin_b;
    enc->pin_z = pin_z;
    enc->ppr = ppr;
    enc->cpr = cpr;
    enc->pio = pio;
    enc->count = 0;
    enc->revolution_count = 0;
    enc->pulses_this_rev = 0;
    enc->direction_cw = true;
    enc->last_z_time = 0;
    enc->last_count_for_rpm = 0;
    enc->last_rpm_time = time_us_64();
    enc->current_rpm = 0.0f;
    enc->last_state = 0;
    
    // Ensure inputs have defined level before handing to PIO
    gpio_init(pin_a);
    gpio_init(pin_b);
    gpio_pull_up(pin_a);
    gpio_pull_up(pin_b);

    if (!pio_can_add_program(pio, &quadrature_encoder_program)) {
        printf("ERROR: Cannot add PIO program\n");
        return false;
    }
    
    enc->offset = pio_add_program(pio, &quadrature_encoder_program);
    printf("PIO program loaded at offset %d\n", enc->offset);
    
    int sm = pio_claim_unused_sm(pio, false);
    if (sm < 0) {
        printf("ERROR: No free PIO state machine\n");
        return false;
    }
    enc->sm = (uint)sm;
    
    printf("Claimed state machine %d on %s\n", enc->sm, pio == pio0 ? "pio0" : "pio1");
    
    // Initialize PIO
    quadrature_encoder_program_init(pio, enc->sm, enc->offset, pin_a);
    
    printf("PIO state machine initialized and running\n");
    printf("  Pin A (input): GPIO %d\n", pin_a);
    printf("  Pin B (input): GPIO %d\n", pin_b);
    
    sleep_ms(100);
    
    if (pio_sm_is_rx_fifo_empty(pio, enc->sm)) {
        printf("WARNING: PIO RX FIFO is empty!\n");
    } else {
        printf("SUCCESS: PIO RX FIFO has data\n");
        
        // **NEW: Dump first 10 FIFO entries to see what PIO is actually sampling**
        printf("\n=== DUMPING FIRST 10 FIFO ENTRIES ===\n");
        for (int i = 0; i < 10 && !pio_sm_is_rx_fifo_empty(pio, enc->sm); i++) {
            uint32_t data = pio_sm_get(pio, enc->sm);
            uint8_t state = data & 0x03;
            printf("  FIFO[%d]: raw=0x%08lX, state=0x%02X, A=%d, B=%d\n",
                   i, data, state, (state >> 1) & 1, state & 1);
        }
        printf("=== END FIFO DUMP ===\n\n");
    }
    
    // Read several entries to settle and establish a solid initial state
    uint8_t observed_state = 0xFF;
    for (int i = 0; i < 8 && !pio_sm_is_rx_fifo_empty(pio, enc->sm); i++) {
        uint32_t data = pio_sm_get(pio, enc->sm);
        observed_state = (uint8_t)(data & 0x03);
    }
    enc->last_state = (observed_state == 0xFF) ? 0 : observed_state;
    printf("Initial state set to: 0x%02X (A=%d, B=%d)\n",
           enc->last_state,
           (enc->last_state >> 1) & 1,
           enc->last_state & 1);
    
    // Configure Z pin and choose active edge dynamically
    gpio_init(pin_z);
    gpio_set_dir(pin_z, GPIO_IN);
    gpio_pull_up(pin_z);
    bool z_idle_high = gpio_get(pin_z);
    uint32_t z_edge = z_idle_high ? GPIO_IRQ_EDGE_FALL : GPIO_IRQ_EDGE_RISE;
    printf("Z pulse pin: GPIO %d (pull-up), idle=%d, trigger edge=%s\n",
           pin_z, z_idle_high ? 1 : 0, z_edge == GPIO_IRQ_EDGE_FALL ? "FALL" : "RISE");

    global_encoder = enc;

    gpio_acknowledge_irq(pin_z, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    gpio_set_irq_enabled_with_callback(pin_z, z_edge, true, &encoder_z_handler);
    
    printf("\n=== Encoder initialization complete ===\n");
    printf("Ready to count. Try rotating the encoder...\n\n");
    
    return true;
}

void encoder_process(encoder_t *enc) {
    static uint32_t total_changes = 0;
    static uint32_t total_reads = 0;
    static uint32_t same_state_count = 0;
    static uint32_t last_debug_time = 0;
    uint32_t changes_this_call = 0;
    
    // **NEW: Track how many times we read from FIFO**
    int fifo_reads_this_call = 0;
    
    while (!pio_sm_is_rx_fifo_empty(enc->pio, enc->sm)) {
        uint32_t data = pio_sm_get(enc->pio, enc->sm);
        uint8_t current_state = (uint8_t)(data & 0x03);
        
        fifo_reads_this_call++;
        total_reads++;
        
        // **NEW: Print EVERY state for first 100 reads**
        if (total_reads <= 100) {
            printf("[%lu] Raw: 0x%08lX | State: 0x%02X (A=%d B=%d) | Last: 0x%02X | ",
                   total_reads, data, current_state,
                   (current_state >> 1) & 1, current_state & 1,
                   enc->last_state);
        }
        
        if (current_state != enc->last_state) {
            uint8_t index = (uint8_t)((enc->last_state << 2) | current_state);
            int8_t change = encoder_states[index];
            
            if (total_reads <= 100) {
                printf("CHANGE! Index=0x%02X, delta=%+d, count=%ld\n",
                       index, change, enc->count + change);
            }
            
            if (change != 0) {
                enc->count += change;
                enc->pulses_this_rev++;
                enc->direction_cw = (change > 0);
                
                if (enc->pulses_this_rev >= enc->cpr) {
                    enc->pulses_this_rev = 0;
                }
                
                changes_this_call++;
                total_changes++;
            }
            
            enc->last_state = current_state;
        } else {
            same_state_count++;
            if (total_reads <= 100) {
                printf("same (total_same=%lu)\n", same_state_count);
            }
        }
    }
    
    // Report FIFO processing stats
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_debug_time > 1000) {
        int fifo_level = pio_sm_get_rx_fifo_level(enc->pio, enc->sm);
        printf("\n=== STATS ===\n");
        printf("  Count: %ld\n", enc->count);
        printf("  Total FIFO reads: %lu\n", total_reads);
        printf("  Total changes: %lu\n", total_changes);
        printf("  Same state reads: %lu\n", same_state_count);
        printf("  FIFO level: %d/8\n", fifo_level);
        printf("  Last state: 0x%02X\n", enc->last_state);
        printf("=============\n\n");
        last_debug_time = now;
    }
}

void encoder_reset(encoder_t *enc) {
    enc->count = 0;
    enc->revolution_count = 0;
    enc->pulses_this_rev = 0;
    enc->last_count_for_rpm = 0;
    enc->current_rpm = 0.0f;
}

int32_t encoder_get_count(encoder_t *enc) {
    return enc->count;
}

int32_t encoder_get_revolutions(encoder_t *enc) {
    return enc->revolution_count;
}

int32_t encoder_get_pulses_this_rev(encoder_t *enc) {
    return enc->pulses_this_rev;
}

bool encoder_get_direction(encoder_t *enc) {
    return enc->direction_cw;
}

float encoder_get_rpm(encoder_t *enc) {
    return enc->current_rpm;
}

void encoder_calculate_rpm(encoder_t *enc) {
    uint64_t current_time = time_us_64();
    uint64_t delta_time = current_time - enc->last_rpm_time;
    
    if (delta_time >= 100000) {
        int32_t current_count = enc->count;
        int32_t delta_count = current_count - enc->last_count_for_rpm;
        
        enc->current_rpm = (float)delta_count / enc->cpr * (60000000.0f / delta_time);
        
        enc->last_count_for_rpm = current_count;
        enc->last_rpm_time = current_time;
    }
}