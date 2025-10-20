// =============================================================================
// dma_stepper.cpp - DMA-Driven PIO Stepper Implementation
// =============================================================================

#include "dma_stepper.h"
#include "dma_step.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "pico/stdlib.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>

bool dma_stepper_init(dma_stepper_t* ctx, PIO pio, uint sm, uint step_gpio) {
    if (!ctx) return false;
    
    ctx->pio = pio;
    ctx->sm = sm;
    ctx->step_gpio = step_gpio;
    ctx->active = false;
    
    // Allocate interval buffer
    ctx->interval_buffer = (uint32_t*)malloc(DMA_STEPPER_MAX_STEPS * 2 * sizeof(uint32_t));
    if (!ctx->interval_buffer) {
        printf("[DMA_STEPPER] ERROR: Failed to allocate buffer!\n");
        return false;
    }
    ctx->buffer_size = 0;
    
    // Add PIO program
    if (!pio_can_add_program(pio, &dma_step_program)) {
        printf("[DMA_STEPPER] ERROR: Cannot add PIO program!\n");
        free(ctx->interval_buffer);
        return false;
    }
    ctx->offset = pio_add_program(pio, &dma_step_program);
    
    // Initialize PIO with clock divider of 1.0 (full speed)
    dma_step_program_init(pio, sm, ctx->offset, step_gpio, 1.0f);
    
    // Claim DMA channel
    ctx->dma_chan = dma_claim_unused_channel(true);
    
    printf("[DMA_STEPPER] Initialized on PIO%d SM%d, GPIO%d, DMA channel %d\n",
           pio == pio0 ? 0 : 1, sm, step_gpio, ctx->dma_chan);
    
    return true;
}

bool dma_stepper_queue_constant_velocity(dma_stepper_t* ctx, uint32_t step_count, float steps_per_sec) {
    if (!ctx || step_count == 0 || steps_per_sec <= 0.0f) return false;
    if (step_count > DMA_STEPPER_MAX_STEPS) step_count = DMA_STEPPER_MAX_STEPS;
    
    // Calculate delay in PIO cycles (125 MHz clock)
    uint32_t clk_hz = clock_get_hz(clk_sys);
    float cycles_per_step = (float)clk_hz / steps_per_sec;
    uint32_t delay_cycles = (uint32_t)(cycles_per_step / 2.0f);  // Half for high, half for low
    
    if (delay_cycles < 4) delay_cycles = 4;  // Minimum delay
    
    printf("[DMA_STEPPER] CV: %lu steps @ %.1f sps -> %lu cycles/half\n",
           step_count, steps_per_sec, delay_cycles);
    
    // Fill buffer with constant delays
    ctx->buffer_size = 0;
    for (uint32_t i = 0; i < step_count; i++) {
        ctx->interval_buffer[ctx->buffer_size++] = delay_cycles;  // High phase
        ctx->interval_buffer[ctx->buffer_size++] = delay_cycles;  // Low phase
    }
    
    // Configure DMA to transfer buffer to PIO TX FIFO
    dma_channel_config c = dma_channel_get_default_config(ctx->dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);   // Read from buffer sequentially
    channel_config_set_write_increment(&c, false); // Write to same PIO FIFO address
    channel_config_set_dreq(&c, pio_get_dreq(ctx->pio, ctx->sm, true));  // Pace by PIO TX
    
    dma_channel_configure(
        ctx->dma_chan,
        &c,
        &ctx->pio->txf[ctx->sm],    // Write to PIO TX FIFO
        ctx->interval_buffer,        // Read from our buffer
        ctx->buffer_size,            // Transfer count
        true                         // Start immediately
    );
    
    ctx->active = true;
    return true;
}

bool dma_stepper_queue_ramp(dma_stepper_t* ctx, uint32_t step_count, float start_sps, float end_sps) {
    if (!ctx || step_count == 0) return false;
    if (step_count > DMA_STEPPER_MAX_STEPS) step_count = DMA_STEPPER_MAX_STEPS;
    
    uint32_t clk_hz = clock_get_hz(clk_sys);
    
    printf("[DMA_STEPPER] RAMP: %lu steps from %.1f to %.1f sps\n",
           step_count, start_sps, end_sps);
    
    // Pre-compute acceleration profile
    ctx->buffer_size = 0;
    for (uint32_t i = 0; i < step_count; i++) {
        // Quadratic ramp profile
        float frac = (float)i / (float)(step_count - 1);
        float current_sps = start_sps + (end_sps - start_sps) * (frac * frac);
        
        float cycles_per_step = (float)clk_hz / current_sps;
        uint32_t delay_cycles = (uint32_t)(cycles_per_step / 2.0f);
        
        if (delay_cycles < 4) delay_cycles = 4;
        
        ctx->interval_buffer[ctx->buffer_size++] = delay_cycles;  // High
        ctx->interval_buffer[ctx->buffer_size++] = delay_cycles;  // Low
    }
    
    // Configure and start DMA
    dma_channel_config c = dma_channel_get_default_config(ctx->dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(ctx->pio, ctx->sm, true));
    
    dma_channel_configure(
        ctx->dma_chan,
        &c,
        &ctx->pio->txf[ctx->sm],
        ctx->interval_buffer,
        ctx->buffer_size,
        true
    );
    
    ctx->active = true;
    return true;
}

bool dma_stepper_is_busy(dma_stepper_t* ctx) {
    if (!ctx) return false;
    return dma_channel_is_busy(ctx->dma_chan);
}

void dma_stepper_stop(dma_stepper_t* ctx) {
    if (!ctx) return;
    
    dma_channel_abort(ctx->dma_chan);
    pio_sm_set_enabled(ctx->pio, ctx->sm, false);
    pio_sm_clear_fifos(ctx->pio, ctx->sm);
    pio_sm_set_enabled(ctx->pio, ctx->sm, true);
    
    ctx->active = false;
    
    printf("[DMA_STEPPER] Stopped\n");
}

void dma_stepper_deinit(dma_stepper_t* ctx) {
    if (!ctx) return;
    
    dma_stepper_stop(ctx);
    dma_channel_unclaim(ctx->dma_chan);
    pio_sm_set_enabled(ctx->pio, ctx->sm, false);
    pio_remove_program(ctx->pio, &dma_step_program, ctx->offset);
    
    if (ctx->interval_buffer) {
        free(ctx->interval_buffer);
        ctx->interval_buffer = NULL;
    }
}
