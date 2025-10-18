#include "spindle_step_pio.h"
#include "spindle_step.pio.h"
#include "hardware/clocks.h"
#include <cstdio>

static inline uint32_t cycles_from_sps(float sps) {
    if (sps <= 0.0f) return 0;
    // PIO runs at system clock (default 125 MHz). Half period cycles = clk / (2*sps)
    uint32_t clk = clock_get_hz(clk_sys);
    float half = (float)clk / (2.0f * sps);
    if (half < 2.0f) half = 2.0f; // ensure minimum
    if (half > 0x7FFFFFFF) half = 0x7FFFFFFF;
    return (uint32_t)half;
}

bool spindle_step_pio_init(spindle_step_pio_t* ctx, PIO pio, uint sm, uint step_gpio) {
    if (!ctx) return false;
    ctx->pio = pio;
    ctx->sm = sm;
    ctx->step_gpio = step_gpio;

    if (!pio_can_add_program(pio, &spindle_step_program)) {
        printf("ERROR: Cannot add spindle_step PIO program!\n");
        return false;
    }
    ctx->offset = pio_add_program(pio, &spindle_step_program);
    spindle_step_program_init(pio, sm, ctx->offset, step_gpio);
    printf("PIO: Spindle step initialized on PIO%d SM%d, GPIO%d, offset=%d\n", 
           pio == pio0 ? 0 : 1, sm, step_gpio, ctx->offset);
    return true;
}

bool spindle_step_pio_queue_cv(spindle_step_pio_t* ctx, uint32_t step_count, float steps_per_sec) {
    if (!ctx) {
        printf("PIO_QUEUE ERROR: ctx is NULL!\n");
        return false;
    }
    if (step_count == 0) {
        printf("PIO_QUEUE ERROR: step_count is 0!\n");
        return false;
    }
    if (steps_per_sec <= 0.0f) {
        printf("PIO_QUEUE ERROR: steps_per_sec=%.1f invalid!\n", steps_per_sec);
        return false;
    }
    
    uint32_t half_cycles = cycles_from_sps(steps_per_sec);
    printf("  [PIO_QUEUE] %lu steps @ %.1f sps -> half_cycles=%lu\n", 
           step_count, steps_per_sec, half_cycles);
    
    // Check FIFO status before pushing
    uint32_t fifo_level = pio_sm_get_tx_fifo_level(ctx->pio, ctx->sm);
    printf("  [PIO_FIFO] TX level=%lu/4 before push\n", fifo_level);
    
    // Push half_period and step_count to TX FIFO (will block if full briefly)
    pio_sm_put_blocking(ctx->pio, ctx->sm, half_cycles);
    pio_sm_put_blocking(ctx->pio, ctx->sm, step_count);
    
    fifo_level = pio_sm_get_tx_fifo_level(ctx->pio, ctx->sm);
    printf("  [PIO_FIFO] TX level=%lu/4 after push - SUCCESS!\n", fifo_level);
    
    return true;
}

void spindle_step_pio_stop(spindle_step_pio_t* ctx) {
    if (!ctx) return;
    
    printf("  [PIO_STOP] Stopping PIO SM%d...\n", ctx->sm);
    
    // Disable state machine immediately
    pio_sm_set_enabled(ctx->pio, ctx->sm, false);
    
    // Clear TX FIFO to remove queued steps
    pio_sm_clear_fifos(ctx->pio, ctx->sm);
    
    // DO NOT RESTART! It breaks pin configuration!
    // Just re-enable - the .wrap will naturally loop back to start
    pio_sm_set_enabled(ctx->pio, ctx->sm, true);
    
    printf("  [PIO_STOP] PIO stopped and cleared - ready for next move\n");
}

void spindle_step_pio_deinit(spindle_step_pio_t* ctx) {
    if (!ctx) return;
    pio_sm_set_enabled(ctx->pio, ctx->sm, false);
    pio_remove_program(ctx->pio, &spindle_step_program, ctx->offset);
}
