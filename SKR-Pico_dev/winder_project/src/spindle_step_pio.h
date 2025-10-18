#pragma once

#include "hardware/pio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    PIO   pio;
    uint  sm;
    uint  offset;
    uint  step_gpio;
} spindle_step_pio_t;

// Initialize spindle step PIO on given PIO/sm and GPIO for STEP
bool spindle_step_pio_init(spindle_step_pio_t* ctx, PIO pio, uint sm, uint step_gpio);

// Queue a constant-velocity burst: step_count steps at steps_per_sec
// Returns false if arguments invalid; blocks briefly if FIFO is full
bool spindle_step_pio_queue_cv(spindle_step_pio_t* ctx, uint32_t step_count, float steps_per_sec);

// Stop immediately - clears FIFO and disables state machine
void spindle_step_pio_stop(spindle_step_pio_t* ctx);

// Clear/disable (optional)
void spindle_step_pio_deinit(spindle_step_pio_t* ctx);

#ifdef __cplusplus
}
#endif
