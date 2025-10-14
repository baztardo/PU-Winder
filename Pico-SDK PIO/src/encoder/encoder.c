#include "encoder.h"
#include "encoder.pio.h"
#include "pico/time.h"
#include <stdio.h>

static encoder_t *global_encoder = NULL;

const int8_t encoder_states[16] = {
    0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0
};

static void encoder_z_handler(uint gpio, uint32_t events) {
    if (global_encoder == NULL) return;
    if (gpio != global_encoder->pin_z) return;
    
    uint64_t current_time = time_us_64();
    
    // Much longer debounce - 50ms
    if (current_time - global_encoder->last_z_time < 50000) {
        return;
    }
    
    // Only count if we've actually moved (count changed)
    static int32_t last_z_count = 0;
    if (global_encoder->count == last_z_count) {
        return;  // Spurious trigger, ignore
    }
    last_z_count = global_encoder->count;
    
    printf("Z pulse! Count: %ld\n", global_encoder->count);
    
    global_encoder->pulses_this_rev = 0;
    
    if (global_encoder->direction_cw) {
        global_encoder->revolution_count++;
    } else {
        global_encoder->revolution_count--;
    }
    
    global_encoder->last_z_time = current_time;
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
    
    // DON'T initialize A and B as GPIO - let PIO do it!
    // We'll read initial state after PIO init
    
    if (!pio_can_add_program(pio, &quadrature_encoder_program)) {
        printf("ERROR: Cannot add PIO program\n");
        return false;
    }
    
    enc->offset = pio_add_program(pio, &quadrature_encoder_program);
    
    int sm = pio_claim_unused_sm(pio, false);
    if (sm < 0) {
        printf("ERROR: No free PIO state machine\n");
        return false;
    }
    enc->sm = (uint)sm;
    
    // Initialize PIO FIRST - this sets up the pins
    quadrature_encoder_program_init(pio, enc->sm, enc->offset, pin_a);
    
    printf("PIO encoder initialized on %s, SM %d\n", 
           pio == pio0 ? "pio0" : "pio1", enc->sm);
    
    // Now read initial state using GPIO (PIO owns the pins now)
    sleep_ms(10);
    uint8_t a_state = gpio_get(pin_a) ? 1 : 0;
    uint8_t b_state = gpio_get(pin_b) ? 1 : 0;
    enc->last_state = (a_state << 1) | b_state;
    
    printf("Initial encoder state: A=%d, B=%d, combined=%d\n", 
           a_state, b_state, enc->last_state);
    
    // Configure Z pin (this one stays as regular GPIO interrupt)
    gpio_init(pin_z);
    gpio_set_dir(pin_z, GPIO_IN);
    gpio_pull_up(pin_z);
    
    global_encoder = enc;
    
    gpio_set_irq_enabled_with_callback(pin_z, GPIO_IRQ_EDGE_RISE, true, 
                                       &encoder_z_handler);
    
    printf("Encoder initialization complete\n");
    
    return true;
}

void encoder_process(encoder_t *enc) {
    static uint32_t last_print = 0;
    int changes_processed = 0;
    
    while (!pio_sm_is_rx_fifo_empty(enc->pio, enc->sm)) {
        uint32_t data = pio_sm_get(enc->pio, enc->sm);
        uint8_t current_state = data & 0x03;
        
        // Print every state we see (for first few seconds)
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_print > 100 && now < 10000) {  // First 10 seconds
            printf("PIO State: %d (A=%d, B=%d)\n", 
                   current_state, (current_state >> 1) & 1, current_state & 1);
            last_print = now;
        }
        
        if (current_state != enc->last_state) {
            uint8_t index = (enc->last_state << 2) | current_state;
            int8_t change = encoder_states[index];
            
            if (changes_processed < 20) {
                printf("State change: %d->%d, Index: %d, Change: %d\n", 
                       enc->last_state, current_state, index, change);
            }
            changes_processed++;
            
            if (change != 0) {
                enc->count += change;
                enc->pulses_this_rev++;
                
                if (change > 0) {
                    enc->direction_cw = true;
                } else {
                    enc->direction_cw = false;
                }
                
                if (enc->pulses_this_rev >= enc->cpr) {
                    enc->pulses_this_rev = 0;
                }
            }
            
            enc->last_state = current_state;
        }
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