#include "spindle_step_pio.h"
#include "spindle_step.pio.h"
#include "hardware/clocks.h"

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

    if (!pio_can_add_program(pio, &spindle_step_program)) return false;
    ctx->offset = pio_add_program(pio, &spindle_step_program);
    spindle_step_program_init(pio, sm, ctx->offset, step_gpio);
    return true;
}

bool spindle_step_pio_queue_cv(spindle_step_pio_t* ctx, uint32_t step_count, float steps_per_sec) {
    if (!ctx || step_count == 0 || steps_per_sec <= 0.0f) return false;
    uint32_t half_cycles = cycles_from_sps(steps_per_sec);
    // Push half_period and step_count to TX FIFO (will block if full briefly)
    pio_sm_put_blocking(ctx->pio, ctx->sm, half_cycles);
    pio_sm_put_blocking(ctx->pio, ctx->sm, step_count);
    return true;
}

void spindle_step_pio_stop(spindle_step_pio_t* ctx) {
    if (!ctx) return;
    // Disable state machine immediately
    pio_sm_set_enabled(ctx->pio, ctx->sm, false);
    // Clear TX FIFO to remove queued steps
    pio_sm_clear_fifos(ctx->pio, ctx->sm);
    // Re-enable state machine for next use
    pio_sm_restart(ctx->pio, ctx->sm);
    pio_sm_set_enabled(ctx->pio, ctx->sm, true);
}

void spindle_step_pio_deinit(spindle_step_pio_t* ctx) {
    if (!ctx) return;
    pio_sm_set_enabled(ctx->pio, ctx->sm, false);
    pio_remove_program(ctx->pio, &spindle_step_program, ctx->offset);
}
