#include "encoder.h"
#include "encoder.pio.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include <stdio.h>
#include <stdlib.h>

// Z counting robustness knobs
#ifndef Z_MIN_INTERVAL_US
#define Z_MIN_INTERVAL_US 20000u // ignore Z edges within 20ms of last (debounce)
#endif
#ifndef Z_REQUIRE_AB_STATE
#define Z_REQUIRE_AB_STATE 0     // do not require a specific A/B state by default
#endif
#ifndef Z_REQUIRED_AB_STATE
#define Z_REQUIRED_AB_STATE 0x3  // typical index gating when A=1 and B=1
#endif

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
    if (current_time - global_encoder->last_z_time < Z_MIN_INTERVAL_US) {
        return;
    }
    
    static int32_t last_z_count = 0;
    int32_t counts_since_last = labs(global_encoder->count - last_z_count);
    int32_t min_counts_between_z = global_encoder->cpr > 0 ? (global_encoder->cpr / 4) : 1;
    if (counts_since_last < min_counts_between_z) {
        return;
    }
    
    // Optional gating: only accept Z when A/B at required state
    if (Z_REQUIRE_AB_STATE) {
        uint ab = ((gpio_get(global_encoder->pin_a) & 1) << 1) |
                  (gpio_get(global_encoder->pin_b) & 1);
        if (ab != Z_REQUIRED_AB_STATE) {
            return;
        }
    }
    
    // Count every qualified Z edge; rely on time debounce above to avoid bounce
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
    
    // No verbose FIFO prints in high-speed mode
    
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

    // Configure PIO IRQ to drain RX FIFO in ISR (reduces main loop load)
    if (enc->pio == pio0) {
        // Enable RX not empty source for this state machine
        pio_set_irq0_source_enabled(pio0, (enum pio_interrupt_source)(pis_sm0_rx_fifo_not_empty + enc->sm), true);
        irq_set_exclusive_handler(PIO0_IRQ_0, [](){
            encoder_t *e = global_encoder;
            if (!e || e->pio != pio0) return;
            while (!pio_sm_is_rx_fifo_empty(e->pio, e->sm)) {
                uint32_t data = pio_sm_get(e->pio, e->sm);
                uint8_t current_state = (uint8_t)(data & 0x03);
                if (current_state != e->last_state) {
                    uint8_t index = (uint8_t)((e->last_state << 2) | current_state);
                    int8_t change = encoder_states[index];
                    if (change != 0) {
                        e->count += change;
                        e->pulses_this_rev++;
                        e->direction_cw = (change > 0);
                        if (e->pulses_this_rev >= e->cpr) {
                            e->pulses_this_rev = 0;
                        }
                    }
                    e->last_state = current_state;
                }
            }
            // Clear the IRQ (safe even if already deasserted)
            irq_clear(PIO0_IRQ_0);
        });
        irq_set_enabled(PIO0_IRQ_0, true);
    } else {
        pio_set_irq0_source_enabled(pio1, (enum pio_interrupt_source)(pis_sm0_rx_fifo_not_empty + enc->sm), true);
        irq_set_exclusive_handler(PIO1_IRQ_0, [](){
            encoder_t *e = global_encoder;
            if (!e || e->pio != pio1) return;
            while (!pio_sm_is_rx_fifo_empty(e->pio, e->sm)) {
                uint32_t data = pio_sm_get(e->pio, e->sm);
                uint8_t current_state = (uint8_t)(data & 0x03);
                if (current_state != e->last_state) {
                    uint8_t index = (uint8_t)((e->last_state << 2) | current_state);
                    int8_t change = encoder_states[index];
                    if (change != 0) {
                        e->count += change;
                        e->pulses_this_rev++;
                        e->direction_cw = (change > 0);
                        if (e->pulses_this_rev >= e->cpr) {
                            e->pulses_this_rev = 0;
                        }
                    }
                    e->last_state = current_state;
                }
            }
            irq_clear(PIO1_IRQ_0);
        });
        irq_set_enabled(PIO1_IRQ_0, true);
    }
    
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
    
    while (!pio_sm_is_rx_fifo_empty(enc->pio, enc->sm)) {
        uint32_t data = pio_sm_get(enc->pio, enc->sm);
        uint8_t current_state = (uint8_t)(data & 0x03);
        
        total_reads++;
        
        if (current_state != enc->last_state) {
            uint8_t index = (uint8_t)((enc->last_state << 2) | current_state);
            int8_t change = encoder_states[index];
            
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
        }
    }
    
    // Quiet: no periodic stats in normal operation
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