// =============================================================================
// winding_controller.cpp - Winding Controller Implementation
// =============================================================================

#include "winding_controller.h"
#include "config.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "scheduler.h"  
#include "pico/malloc.h"
#include <cmath>
#include <algorithm>
#include "spindle_step_pio.h"

spindle_step_pio_t spindle_step_pio;

extern Scheduler scheduler;

static spindle_step_pio_t g_spindle;

// Optional helper: get current free heap in bytes
static uint32_t get_free_heap() {
    extern char __StackLimit, __bss_end__;
    return (uint32_t)(&__StackLimit - &__bss_end__);
}

WindingController::WindingController(MoveQueue* mq, Encoder* enc, LCDDisplay* lcd)
    : move_queue(mq)
    , encoder(enc)
    , lcd(lcd)
    , state(WindingState::IDLE)
    , current_layer(0)
    , turns_completed(0)
    , turns_this_layer(0)
    , current_rpm(0)
    , last_encoder_position(0)
    , last_rpm_update_time(0)
    , traverse_direction(true)
    , current_traverse_position_mm(0) {
}

void WindingController::init() {
    printf("WindingController::init() called\n");
    state = WindingState::IDLE;
    printf("Initializing spindle PIO...\n");
    bool pio_ok = spindle_step_pio_init(&spindle_step_pio, pio0, 2, SPINDLE_STEP_PIN);
    printf("Spindle PIO init result: %d\n", pio_ok);
    lcd->clear();
    lcd->print_at(0, 0, "Winder Ready");
    lcd->print_at(0, 1, "Press Start...");
}

void WindingController::set_parameters(const WindingParams& p) {
    params = p;
    params.calculate_layers();
}

bool WindingController::start() {
    if (state != WindingState::IDLE) return false;
    
    // Reset counters
    current_layer = 0;
    turns_completed = 0;
    turns_this_layer = 0;
    current_traverse_position_mm = 0;
    // Reset ramp / sync state
    ramp_started = false;
    ramp_start_time = 0;
    turn_accum = 0.0;
    encoder_sign = 0;
    traverse_steps_emitted = 0.0;
    last_encoder_position = encoder->get_position();
    int32_t pos_now = encoder->get_position();
    enc_last_sync = pos_now;
    enc_last_rpm  = pos_now;
    
    // Start sequence
    state = WindingState::HOMING_SPINDLE;
    lcd->clear();
    lcd->print_at(0, 0, "Starting Winding...");
    
    return true;
}

void WindingController::stop() {
    state = WindingState::IDLE;
    move_queue->clear_queue(AXIS_SPINDLE);
    move_queue->clear_queue(AXIS_TRAVERSE);
    lcd->clear();
    lcd->print_at(0, 0, "Stopped");
}

void WindingController::emergency_stop() {
    state = WindingState::ERROR;
    move_queue->clear_queue(AXIS_SPINDLE);
    move_queue->clear_queue(AXIS_TRAVERSE);
    move_queue->set_enable(AXIS_SPINDLE, false);
    move_queue->set_enable(AXIS_TRAVERSE, false);
    
    lcd->clear();
    lcd->print_at(0, 0, "EMERGENCY STOP!");
    lcd->print_at(0, 1, "System Halted");
}

void WindingController::update() {
    update_rpm();
    
    switch (state) {
        case WindingState::IDLE:
            // Waiting for start command
            break;
            
        case WindingState::HOMING_SPINDLE:
            home_spindle();
            break;
            
        case WindingState::HOMING_TRAVERSE:
            home_traverse();
            break;
            
        case WindingState::MOVING_TO_START:
            move_to_start();
            break;
            
        case WindingState::RAMPING_UP:
            ramp_up_spindle();
            if (state == WindingState::WINDING)
                sleep_ms(200);  // let the queue stabilize
            break;
            
        case WindingState::WINDING:
            execute_winding();
            break;
            
        case WindingState::RAMPING_DOWN:
            ramp_down_spindle();
            break;
            
        case WindingState::COMPLETE:
            // Display completion message
            lcd->clear();
            lcd->print_at(0, 0, "Winding Complete!");
            lcd->printf_at(0, 1, "Turns: %lu", turns_completed);
            lcd->printf_at(0, 2, "Layers: %lu", current_layer);
            state = WindingState::IDLE;
            break;
            
        case WindingState::ERROR:
            // Stay in error state until reset
            break;
    }
    
    update_display();
}

void WindingController::home_spindle() {
    lcd->clear();
    lcd->print_at(0, 0, "Homing Spindle...");
    lcd->print_at(0, 1, "Finding Z-Index");
    
    // Wait for Z index pulse
    // Rotate spindle slowly and watch for Z pulse
    static bool waiting_for_z = true;
    static uint32_t start_time = 0;
    
    if (waiting_for_z) {
        start_time = time_us_32();
        waiting_for_z = false;
        
        // Generate slow rotation move (one revolution to find Z)
        // Use PIO for spindle stepping (not move_queue)
        uint32_t steps_per_rev = 200 * MOTOR_MICROSTEPS;  // 3200 for 16x microstepping
        float slow_sps = 200.0f;  // 200 steps/sec = slow rotation
        
        ::spindle_step_pio_queue_cv(&spindle_step_pio, steps_per_rev, slow_sps);
    }
    
    // Check for Z pulse
    if (encoder->check_z_pulse()) {
        // Found Z index!
        encoder->reset();
        
        // CRITICAL: Stop the PIO immediately!
        ::spindle_step_pio_stop(&spindle_step_pio);
        move_queue->clear_queue(AXIS_SPINDLE);
        
        lcd->clear();
        lcd->print_at(0, 0, "Z Index Found!");
        printf("Z-index detected! Moving to traverse homing\n");
        sleep_ms(1000);
        
        state = WindingState::HOMING_TRAVERSE;
        waiting_for_z = true;
    }
    
    // Timeout after 10 seconds
    if ((time_us_32() - start_time) > 10000000) {
        lcd->print_at(0, 3, "Z Index Timeout!");
        state = WindingState::ERROR;
        waiting_for_z = true;
    }
}

void WindingController::home_traverse() {
    static enum { INIT, MOVING_TO_SWITCH, BACKING_OFF, DONE } homing_state = INIT;
    static bool lcd_updated = false;
    
    if (homing_state == INIT && !lcd_updated) {
        lcd->clear();
        lcd->print_at(0, 0, "Homing Traverse...");
        printf("Starting traverse homing\n");
        lcd_updated = true;
    }
    
    switch (homing_state) {
        case INIT:
            // Initialize home switch pin
            gpio_init(TRAVERSE_HOME_PIN);
            gpio_set_dir(TRAVERSE_HOME_PIN, GPIO_IN);
            gpio_pull_up(TRAVERSE_HOME_PIN);
            
            // Set direction towards home
            move_queue->set_direction(AXIS_TRAVERSE, false);
            homing_state = MOVING_TO_SWITCH;
            // UI suppressed
            break;
            
        case MOVING_TO_SWITCH:
            // Move towards switch at homing speed
            if (gpio_get(TRAVERSE_HOME_PIN) == 0) {  // Switch triggered (active low)
                move_queue->clear_queue(AXIS_TRAVERSE);
                // UI suppressed
                
                // Back off 2mm at moderate speed
                uint32_t backoff_steps = mm_to_steps(2.0f);
                auto chunks = StepCompressor::compress_constant_velocity(
                    backoff_steps, TRAVERSE_HOMING_SPEED
                );
                
                move_queue->set_direction(AXIS_TRAVERSE, true);  // Reverse direction
                for (const auto& chunk : chunks) {
                    move_queue->push_chunk(AXIS_TRAVERSE, chunk);
                }
                
                homing_state = BACKING_OFF;
                // UI suppressed
            } else {
                // Continue moving towards switch at homing speed
                if (!move_queue->is_active(AXIS_TRAVERSE)) {
                    auto chunks = StepCompressor::compress_constant_velocity(
                        1000, TRAVERSE_HOMING_SPEED
                    );
                    for (const auto& chunk : chunks) {
                        move_queue->push_chunk(AXIS_TRAVERSE, chunk);
                    }
                }
            }
            break;
            
        case BACKING_OFF:
            // Wait for backoff to complete
            if (!move_queue->is_active(AXIS_TRAVERSE) && 
                !move_queue->has_chunk(AXIS_TRAVERSE)) {
                
                // UI suppressed
                current_traverse_position_mm = 0;
                homing_state = DONE;
            }
            break;
            
        case DONE:
            lcd->clear();
            lcd->print_at(0, 0, "Traverse Homed!");
            printf("Traverse homing complete\n");
            sleep_ms(1000);
            state = WindingState::MOVING_TO_START;
            homing_state = INIT;
            lcd_updated = false;
            break;
    }
}

void WindingController::move_to_start() {
    // Quiet LCD during move to start
    
    static bool move_queued = false;
    
    if (!move_queued) {
        uint32_t steps = mm_to_steps(params.start_position_mm);
        auto chunks = StepCompressor::compress_trapezoid(
            steps, 0, TRAVERSE_RAPID_SPEED, TRAVERSE_RAPID_ACCEL, 20.0
        );
        
        move_queue->set_direction(AXIS_TRAVERSE, true);
        for (const auto& chunk : chunks) {
            move_queue->push_chunk(AXIS_TRAVERSE, chunk);
        }
        
        move_queued = true;
    }
    
    // Wait for move to complete
    if (!move_queue->is_active(AXIS_TRAVERSE) && 
        !move_queue->has_chunk(AXIS_TRAVERSE)) {
        
        current_traverse_position_mm = params.start_position_mm;
        // UI suppressed
        sleep_ms(500);
        
        state = WindingState::RAMPING_UP;
        move_queued = false;
    }
}

void WindingController::ramp_up_spindle() {
    static bool banner_printed = false;
    if (!banner_printed) banner_printed = true;

    if (params.spindle_rpm <= 0.0f || params.ramp_time_sec <= 0.0f) {
        lcd->print_at(0, 1, "Param error!");
        state = WindingState::ERROR;
        return;
    }

    if (!ramp_started) {
        lcd->clear();
        lcd->print_at(0, 0, "Ramping Up...");
        lcd->printf_at(0, 1, "Target: %.0f RPM", params.spindle_rpm);
        printf("Starting spindle ramp up to %.1f RPM over %.1f seconds\n", params.spindle_rpm, params.ramp_time_sec);
        
        ramp_started = true;
        ramp_start_time = time_us_32();

        // CRITICAL: Enable spindle motor!
        move_queue->set_enable(AXIS_SPINDLE, true);
        bool spindle_dir = (SPINDLE_DIR_INVERT == 0);
        move_queue->set_direction(AXIS_SPINDLE, spindle_dir);
        printf("Spindle motor enabled, direction: %d\n", spindle_dir);

        const uint32_t steps_per_rev = 200u * SPINDLE_MICROSTEPS;  // Use SPINDLE, not MOTOR!
        // Account for 2:1 gear ratio
        const float stepper_rpm = params.spindle_rpm * SPINDLE_GEAR_RATIO;
        const float target_sps_nom = (stepper_rpm / 60.0f) * steps_per_rev;
        // Use centralized speed limit from config.h
        const float max_sps = MAX_SPINDLE_SPS;
        const float target_sps = std::min(target_sps_nom, max_sps);

        const int   N_slices = 24;
        const float slice_s  = params.ramp_time_sec / (float)N_slices;
        const float sps_min  = std::max(100.0f, target_sps * 0.02f);

        for (int i = 1; i <= N_slices; ++i) {
            float frac = (float)i / (float)N_slices;
            float sps  = sps_min + (target_sps - sps_min) * (frac * frac);
            // CRITICAL: Ensure each ramp slice respects max_sps limit!
            if (sps > max_sps) sps = max_sps;
            uint32_t steps = (uint32_t)std::max(1.0f, sps * slice_s);
            printf("Ramp slice %d: queuing %lu steps at %.1f sps\n", i, steps, sps);
            ::spindle_step_pio_queue_cv(&spindle_step_pio, steps, sps);
        }
        printf("Ramp up: All %d slices queued to PIO\n", N_slices);
        return;
    }

    uint32_t elapsed_ms = (time_us_32() - ramp_start_time) / 1000;
    bool time_done = (elapsed_ms >= (uint32_t)(params.ramp_time_sec * 1000.0f));

    if (time_done) {
        ramp_started = false;

        // FIXED: Use SPINDLE_MICROSTEPS (4x), not MOTOR_MICROSTEPS (16x)!
        // Account for gear ratio: 40T→20T means stepper at HALF spindle speed
        float steps_per_rev_f = 200.0f * SPINDLE_MICROSTEPS;
        float stepper_rpm = params.spindle_rpm * SPINDLE_GEAR_RATIO;  // 0.5 ratio
        float target_sps = (stepper_rpm / 60.0f) * steps_per_rev_f;
        // Match ramp-up and continuous limits
        const float max_sps = MAX_SPINDLE_SPS;  // From config.h
        if (target_sps > max_sps) target_sps = max_sps;

        uint32_t spindle_steps = (uint32_t)(target_sps * 1.5f);
        printf("  [CONTINUOUS] Initial queue: %lu steps @ %.1f sps (%.1f RPM)\n", 
               spindle_steps, target_sps, params.spindle_rpm);
        ::spindle_step_pio_queue_cv(&spindle_step_pio, spindle_steps, target_sps);

        state = WindingState::WINDING;
        banner_printed = false;
        return;
    }
}

void WindingController::execute_winding() {
    // CRITICAL: Keep spindle running with PIO!
    // PIO FIFO is 4 deep. We queue 2 values per move (half_period + step_count).
    // So we can have at most 2 moves queued. Check if FIFO has room and refill.
    
    // Check PIO FIFO depth (TX FIFO)
    uint32_t fifo_level = pio_sm_get_tx_fifo_level(spindle_step_pio.pio, spindle_step_pio.sm);
    
    // If FIFO has room (< 2 entries used out of 4), queue more steps
    if (fifo_level < 2) {
        // Calculate continuous spindle movement
        // Account for gear ratio: stepper at HALF spindle speed
        float stepper_rps = (params.spindle_rpm / 60.0f) * SPINDLE_GEAR_RATIO;
        uint32_t steps_per_rev = 200 * SPINDLE_MICROSTEPS;  // Spindle uses 4x
        float target_sps = stepper_rps * steps_per_rev;
        
        // Apply max speed limit (same as ramp-up)
        const float max_sps = MAX_SPINDLE_SPS;  // From config.h
        if (target_sps > max_sps) target_sps = max_sps;
        
        // Queue 1.5 seconds worth of steps (matches ramp-up logic)
        uint32_t spindle_steps = (uint32_t)(target_sps * 1.5f);
        
        printf("  [CONTINUOUS] Re-queueing %lu steps @ %.1f sps (%.1f RPM)\n", 
               spindle_steps, target_sps, params.spindle_rpm);
        ::spindle_step_pio_queue_cv(&spindle_step_pio, spindle_steps, target_sps);
    }
    
    // Now sync traverse to spindle
    sync_traverse_to_spindle();
    
    // Check if we've completed target turns
    if (turns_completed >= params.target_turns) {
        printf("Target turns reached! Stopping spindle immediately.\n");
        // CRITICAL: Stop PIO immediately to prevent overrun!
        spindle_step_pio_stop(&spindle_step_pio);
        state = WindingState::RAMPING_DOWN;
        return;
    }
}

void WindingController::sync_traverse_to_spindle() {
    // Current encoder position
    const int32_t pos = encoder->get_position();
    int32_t delta = pos - last_encoder_position;

    if (delta <= 0) {
        // no forward progress this tick
        return;
    }

    // Full turns completed since last time
    uint32_t new_turns = (uint32_t)(delta / ENCODER_CPR);
    if (new_turns == 0) {
        // Haven't crossed a full revolution yet; keep feeding spindle elsewhere
        return;
    }

    // Bookkeeping
    turns_completed   += new_turns;
    turns_this_layer  += new_turns;
    last_encoder_position += (int32_t)(new_turns * ENCODER_CPR);

    // End-of-layer handling
    if (turns_this_layer >= params.turns_per_layer) {
        current_layer++;
        turns_this_layer = 0;
        traverse_direction = !traverse_direction; // zig-zag
        lcd->printf_at(0, 2, "Layer: %lu/%lu", current_layer, params.total_layers);
    }

    // Required traverse distance for these turns
    float traverse_mm = new_turns * params.wire_pitch_mm;
    uint32_t traverse_steps = mm_to_steps(traverse_mm);
    if (traverse_steps == 0) return;

    // Use measured spindle RPM for true synchronization
    // current_rpm is updated from encoder in update_rpm()
    float spindle_rps_meas = current_rpm / 60.0f;
    // Traverse speed (mm/s) = spindle RPS * wire pitch
    float traverse_mmps = spindle_rps_meas * params.wire_pitch_mm;

    // Convert to steps/s
    float steps_per_mm = mm_to_steps(1.0f);
    float traverse_sps = traverse_mmps * steps_per_mm;

    // Floor to a minimum for smoothness
    if (traverse_sps < TRAVERSE_MIN_WINDING_SPEED) {
        traverse_sps = TRAVERSE_MIN_WINDING_SPEED;
    }

    // Queue the move
    move_queue->set_direction(AXIS_TRAVERSE, traverse_direction);
    auto chunks = StepCompressor::compress_constant_velocity(traverse_steps, traverse_sps);
    for (const auto& c : chunks) {
        move_queue->push_chunk(AXIS_TRAVERSE, c);
    }
}

void WindingController::ramp_down_spindle() {
    lcd->clear();
    lcd->print_at(0, 0, "Winding Complete!");
    lcd->printf_at(0, 1, "Turns: %lu", turns_completed);
    
    static bool ramp_started = false;
    static uint32_t ramp_start_time = 0;
    
    if (!ramp_started) {
        printf("Ramp-down: Spindle already stopped at target turns.\n");
        
        // PIO already stopped in execute_winding() when target reached
        // Just ensure it's stopped
        spindle_step_pio_stop(&spindle_step_pio);
        
        // Spindle already stopped - just disable motor
        move_queue->set_enable(AXIS_SPINDLE, false);
        printf("Spindle motor disabled.\n");
        ramp_started = true;
    }
    
    // Spindle already stopped, just wait a moment then complete
    sleep_ms(500);
    state = WindingState::COMPLETE;
    ramp_started = false;
}

void WindingController::update_rpm() {
    uint32_t now = time_us_32();
    uint32_t dt_us = now - last_rpm_update_time;
    if (dt_us < 500000) return; // 100 ms to 500 ms

    int32_t pos = encoder->get_position();
    int32_t delta = pos - enc_last_rpm;
    enc_last_rpm = pos;

    if (ENCODER_INVERT) delta = -delta;

    float dt = dt_us / 1000000.0f;
    float rps = (delta / (float)ENCODER_CPR) / dt;
    current_rpm = rps * 60.0f;

    last_rpm_update_time = now;
    printf("[ENC] pos=%ld rpm=%.1f\n", (long)encoder->get_position(), current_rpm);
}

void WindingController::update_display() {
    return; // TODO: remove this    
    static uint32_t last_update_ms = 0;
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (now_ms - last_update_ms < 200) {
        return; // throttle LCD updates to ~5 Hz
    }
    last_update_ms = now_ms;

    // Only render during WINDING to avoid clashing with state-specific messages
    if (state != WindingState::WINDING) {
        return;
    }

    const int32_t enc_counts = encoder->get_position();
    const float turns_f = (float)enc_counts / (float)ENCODER_CPR;
    turns_completed = enc_counts / ENCODER_CPR;

    lcd->printf_at(0, 0, "RPM:%4.0f  WIND", current_rpm);
    lcd->printf_at(0, 1, "Turns: %7.2f", turns_f);
    lcd->printf_at(0, 2, "Layer: %lu/%lu", current_layer, params.total_layers);
    lcd->printf_at(0, 3, "Cnt:%ld L:%lu/%lu",
                   (long)enc_counts,
                   (unsigned long)turns_this_layer,
                   (unsigned long)params.turns_per_layer);
}

uint32_t WindingController::mm_to_steps(float mm) {
    // Lead screw: 5mm per revolution
    // Motor: 200 steps * microsteps per revolution
    float revs = mm / TRAVERSE_PITCH_MM;
    uint32_t steps = (uint32_t)(revs * 200 * TRAVERSE_MICROSTEPS);
    return steps;
}

float WindingController::steps_to_mm(uint32_t steps) {
    float revs = steps / (200.0f * TRAVERSE_MICROSTEPS);
    return revs * TRAVERSE_PITCH_MM;
}
