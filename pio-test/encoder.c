#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "quad_encoder.pio.h"
#include "z_index.pio.h"
#include "encoder.h"

// Transition lookup: rows old(AB), cols new(AB)
//  code = (old<<2)|new  -> delta = dir_table[code]
static const int8_t dir_table[16] = {
/* old=00 */  0, +1, -1,  0,
/* old=01 */ -1,  0,  0, +1,
/* old=10 */ +1,  0,  0, -1,
/* old=11 */  0, -1, +1,  0
};

static encoder_t *s_enc = NULL;   // bound for IRQ callback (Z pulses)

// PIO0 IRQ0 handler for z_index.pio
static void __isr pio0_irq0_handler(void) {
    // Clear the IRQ (we used 'irq nowait 0' from the PIO program)
    // The PIO raises source 'pis_interrupt0'
    if (pio0->irq & 1u) {
        pio0->irq = 1u; // write 1 to clear
        if (s_enc) {
            // bump revs based on last known direction
            if      (s_enc->direction > 0) s_enc->revolutions++;
            else if (s_enc->direction < 0) s_enc->revolutions--;
        }
    }
}

void encoder_init(encoder_t *enc, PIO pio, uint sm_ab, uint sm_z,
                  uint pin_a, uint pin_b, uint pin_z, float ab_clkdiv)
{
    // A/B input pins
    gpio_pull_up(pin_a);
    gpio_pull_up(pin_b);
    // Z input pin
    gpio_pull_up(pin_z);

    uint off_ab = pio_add_program(pio, &quad_encoder_program);
    quad_encoder_program_init(pio, sm_ab, off_ab, pin_a, ab_clkdiv);

    uint off_z  = pio_add_program(pio, &z_index_program);
    z_index_program_init(pio, sm_z, off_z, pin_z);

    // Route PIO0 IRQ0 to our handler and enable the source from PIO
    irq_set_exclusive_handler(PIO0_IRQ_0, pio0_irq0_handler);
    irq_set_enabled(PIO0_IRQ_0, true);
    pio_set_irq0_source_enabled(pio, pis_interrupt0, true);

    enc->position   = 0;
    enc->revolutions = 0;
    enc->direction  = 0;
    enc->last_time  = get_absolute_time();

    s_enc = enc; // bind for Z IRQ
}

void encoder_update(encoder_t *enc, PIO pio, uint sm_ab)
{
    while (!pio_sm_is_rx_fifo_empty(pio, sm_ab)) {
        uint32_t code = pio_sm_get(pio, sm_ab) & 0xF;  // low 4 bits
        int8_t delta  = dir_table[code];
        if (delta) {
            enc->position += delta;
            enc->direction = (delta > 0) ? 1 : -1;
        }
        // else (delta==0) = no movement / bounce; ignore
    }
}

float encoder_get_rpm(encoder_t *enc, uint32_t pulses_per_rev)
{
    static int32_t last_pos = 0;
    absolute_time_t now = get_absolute_time();
    int64_t us = absolute_time_diff_us(enc->last_time, now);
    if (us <= 0) return 0.0f;
    enc->last_time = now;

    int32_t delta = enc->position - last_pos;
    last_pos = enc->position;

    float revs = (float)delta / (float)pulses_per_rev;
    return (revs * 1e6f / (float)us) * 60.0f;
}