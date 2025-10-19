// =============================================================================
// dma_stepper.h - DMA-Driven PIO Stepper (Van Hunter Adams technique)
// =============================================================================

#pragma once

#include "hardware/pio.h"
#include "hardware/dma.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DMA_STEPPER_MAX_STEPS 10000

typedef struct {
    PIO pio;
    uint sm;
    uint offset;
    uint step_gpio;
    int dma_chan;
    uint32_t* interval_buffer;
    uint32_t buffer_size;
    bool active;
} dma_stepper_t;

bool dma_stepper_init(dma_stepper_t* ctx, PIO pio, uint sm, uint step_gpio);
bool dma_stepper_queue_constant_velocity(dma_stepper_t* ctx, uint32_t step_count, float steps_per_sec);
bool dma_stepper_queue_ramp(dma_stepper_t* ctx, uint32_t step_count, float start_sps, float end_sps);
bool dma_stepper_is_busy(dma_stepper_t* ctx);
void dma_stepper_stop(dma_stepper_t* ctx);
void dma_stepper_deinit(dma_stepper_t* ctx);

#ifdef __cplusplus
}
#endif

#endif // DMA_STEPPER_H
