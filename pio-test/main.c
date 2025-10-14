#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "encoder.h"

// ======= CHANGE YOUR PINS HERE (SKR-Pico V1.0) =======
// Set these to the GPIOs you wired your encoder to.
#define ENC_PIN_A   3
#define ENC_PIN_B   4
#define ENC_PIN_Z   25

// Pulses per revolution of your A/B (full-cycle count, not edges per channel)
#define PULSES_PER_REV  1024u

// PIO/SM selection (kept simple & consistent)
#define PIO_AB   pio0
#define SM_AB    0
#define PIO_Z    pio0
#define SM_Z     1

encoder_t g_encoder;

int main() {
    stdio_init_all();
    sleep_ms(500); // allow USB CDC to enumerate

    // A/B sample clock divider. 1.0 = sys_clk; use bigger value to reduce FIFO load.
    // For many encoders, 2~10 is a good starting range.
    const float ab_clkdiv = 4.0f;

    encoder_init(&g_encoder, PIO_AB, SM_AB, SM_Z,
                 ENC_PIN_A, ENC_PIN_B, ENC_PIN_Z, ab_clkdiv);

    absolute_time_t last_print = get_absolute_time();

    while (true) {
        // Drain A/B FIFO and update direction/position
        encoder_update(&g_encoder, PIO_AB, SM_AB);

        // Print at ~10 Hz
        if (absolute_time_diff_us(last_print, get_absolute_time()) > 100000) {
            last_print = get_absolute_time();
            float rpm = encoder_get_rpm(&g_encoder, PULSES_PER_REV);
            printf("Dir:%2d  Pos:%8ld  Rev:%6ld  RPM:%7.2f\r\n",
                   g_encoder.direction,
                   (long)g_encoder.position,
                   (long)g_encoder.revolutions,
                   rpm);
        }
        tight_loop_contents();
    }
}