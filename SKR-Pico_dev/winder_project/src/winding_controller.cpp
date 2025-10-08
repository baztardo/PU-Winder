// =============================================================================
// winding_controller.cpp - Winding Controller Implementation
// =============================================================================

#include "winding_controller.h"
#include "config.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <cmath>
#include <algorithm>

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
    state = WindingState::IDLE;
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
    lcd->print_at(0, 1, "Finding Z Index");
    
    // Wait for Z index pulse
    // Rotate spindle slowly and watch for Z pulse
    static bool waiting_for_z = true;
    static uint32_t start_time = 0;
    
    if (waiting_for_z) {
        start_time = time_us_32();
        waiting_for_z = false;
        
        // Generate slow rotation move (one revolution to find Z)
        auto chunks = StepCompressor::compress_constant_velocity(
            3200,  // One revolution
            200    // Slow speed
        );
        
        for (const auto& chunk : chunks) {
            move_queue->push_chunk(AXIS_SPINDLE, chunk);
        }
    }
    
    // Check for Z pulse
    if (encoder->check_z_pulse()) {
        // Found Z index!
        encoder->reset();
        move_queue->clear_queue(AXIS_SPINDLE);
        
        lcd->print_at(0, 2, "Z Index Found!");
        sleep_ms(500);
        
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
    lcd->clear();
    lcd->print_at(0, 0, "Homing Traverse...");
    
    static enum { INIT, MOVING_TO_SWITCH, BACKING_OFF, DONE } homing_state = INIT;
    
    switch (homing_state) {
        case INIT:
            // Initialize home switch pin
            gpio_init(TRAVERSE_HOME_PIN);
            gpio_set_dir(TRAVERSE_HOME_PIN, GPIO_IN);
            gpio_pull_up(TRAVERSE_HOME_PIN);
            
            // Set direction towards home
            move_queue->set_direction(AXIS_TRAVERSE, false);
            homing_state = MOVING_TO_SWITCH;
            lcd->print_at(0, 1, "Moving to switch");
            break;
            
        case MOVING_TO_SWITCH:
            // Move slowly towards switch
            if (gpio_get(TRAVERSE_HOME_PIN) == 0) {  // Switch triggered (active low)
                move_queue->clear_queue(AXIS_TRAVERSE);
                lcd->print_at(0, 1, "Switch found!");
                
                // Back off 2mm
                uint32_t backoff_steps = mm_to_steps(2.0f);
                auto chunks = StepCompressor::compress_constant_velocity(
                    backoff_steps, 200
                );
                
                move_queue->set_direction(AXIS_TRAVERSE, true);  // Reverse direction
                for (const auto& chunk : chunks) {
                    move_queue->push_chunk(AXIS_TRAVERSE, chunk);
                }
                
                homing_state = BACKING_OFF;
                lcd->print_at(0, 2, "Backing off...");
            } else {
                // Continue moving towards switch
                if (!move_queue->is_active(AXIS_TRAVERSE)) {
                    auto chunks = StepCompressor::compress_constant_velocity(100, 200);
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
                
                lcd->print_at(0, 2, "Home complete!");
                current_traverse_position_mm = 0;
                homing_state = DONE;
            }
            break;
            
        case DONE:
            sleep_ms(500);
            state = WindingState::MOVING_TO_START;
            homing_state = INIT;
            break;
    }
}

void WindingController::move_to_start() {
    lcd->clear();
    lcd->print_at(0, 0, "Moving to Start");
    lcd->printf_at(0, 1, "Target: %.1fmm", params.start_position_mm);
    
    static bool move_queued = false;
    
    if (!move_queued) {
        uint32_t steps = mm_to_steps(params.start_position_mm);
        auto chunks = StepCompressor::compress_trapezoid(
            steps, 0, 500, 1000, 20.0
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
        lcd->print_at(0, 2, "In position!");
        sleep_ms(500);
        
        state = WindingState::RAMPING_UP;
        move_queued = false;
    }
}

void WindingController::ramp_up_spindle() {
    lcd->clear();
    lcd->print_at(0, 0, "Ramping Up...");
    
    static bool ramp_started = false;
    static uint32_t ramp_start_time = 0;
    
    if (!ramp_started) {
        ramp_start_time = time_us_32();
        ramp_started = true;
        
        // Calculate steps for ramp time
        float target_rps = params.spindle_rpm / 60.0f;  // Revolutions per second
        uint32_t steps_per_rev = 200 * MOTOR_MICROSTEPS;
        float target_sps = target_rps * steps_per_rev;  // Steps per second
        
        uint32_t ramp_steps = (uint32_t)(target_sps * params.ramp_time_sec);
        
        // Generate ramp-up move
        auto chunks = StepCompressor::compress_trapezoid(
            ramp_steps, 0, target_sps, target_sps / params.ramp_time_sec, 20.0
        );
        
        for (const auto& chunk : chunks) {
            move_queue->push_chunk(AXIS_SPINDLE, chunk);
        }
    }
    
    lcd->printf_at(0, 1, "RPM: %.0f", current_rpm);
    lcd->printf_at(0, 2, "Target: %.0f", params.spindle_rpm);
    
    // Check if ramp complete
    uint32_t elapsed_ms = (time_us_32() - ramp_start_time) / 1000;
    if (elapsed_ms > (params.ramp_time_sec * 1000)) {
        lcd->print_at(0, 3, "At speed!");
        sleep_ms(500);
        
        state = WindingState::WINDING;
        ramp_started = false;
    }
}

void WindingController::execute_winding() {
    // This is the main winding loop
    sync_traverse_to_spindle();
    
    // Check if we've completed target turns
    if (turns_completed >= params.target_turns) {
        state = WindingState::RAMPING_DOWN;
        return;
    }
}

void WindingController::sync_traverse_to_spindle() {
    // Get current spindle position in encoder counts
    int32_t encoder_pos = encoder->get_position();
    int32_t encoder_delta = encoder_pos - last_encoder_position;
    
    if (encoder_delta <= 0) return;  // No movement yet
    
    // Calculate how many turns have completed
    uint32_t new_turns = encoder_delta / ENCODER_CPR;
    
    if (new_turns > 0) {
        turns_completed += new_turns;
        turns_this_layer += new_turns;
        last_encoder_position += new_turns * ENCODER_CPR;
        
        // Calculate required traverse movement
        float traverse_move_mm = new_turns * params.wire_pitch_mm;
        
        // Check if we need to reverse at end of layer
        if (turns_this_layer >= params.turns_per_layer) {
            // New layer!
            current_layer++;
            turns_this_layer = 0;
            traverse_direction = !traverse_direction;
            
            lcd->printf_at(0, 2, "Layer: %lu/%lu", 
                          current_layer, params.total_layers);
        }
        
        // Generate traverse movement
        uint32_t traverse_steps = mm_to_steps(traverse_move_mm);
        
        if (traverse_steps > 0) {
            auto chunks = StepCompressor::compress_constant_velocity(
                traverse_steps,
                500  // Moderate speed
            );
            
            move_queue->set_direction(AXIS_TRAVERSE, traverse_direction);
            for (const auto& chunk : chunks) {
                move_queue->push_chunk(AXIS_TRAVERSE, chunk);
            }
        }
    }
    
    // Keep spindle running at constant speed
    if (move_queue->get_queue_depth(AXIS_SPINDLE) < 10) {
        float target_rps = params.spindle_rpm / 60.0f;
        uint32_t steps_per_rev = 200 * MOTOR_MICROSTEPS;
        float target_sps = target_rps * steps_per_rev;
        
        auto chunks = StepCompressor::compress_constant_velocity(
            steps_per_rev,  // One revolution
            target_sps
        );
        
        for (const auto& chunk : chunks) {
            move_queue->push_chunk(AXIS_SPINDLE, chunk);
        }
    }
}

void WindingController::ramp_down_spindle() {
    lcd->clear();
    lcd->print_at(0, 0, "Ramping Down...");
    
    static bool ramp_started = false;
    
    if (!ramp_started) {
        move_queue->clear_queue(AXIS_SPINDLE);
        move_queue->clear_queue(AXIS_TRAVERSE);
        
        // Generate ramp-down
        float current_rps = params.spindle_rpm / 60.0f;
        uint32_t steps_per_rev = 200 * MOTOR_MICROSTEPS;
        float current_sps = current_rps * steps_per_rev;
        
        uint32_t ramp_steps = (uint32_t)(current_sps * params.ramp_time_sec);
        
        auto chunks = StepCompressor::compress_trapezoid(
            ramp_steps, current_sps, 0, -current_sps / params.ramp_time_sec, 20.0
        );
        
        for (const auto& chunk : chunks) {
            move_queue->push_chunk(AXIS_SPINDLE, chunk);
        }
        
        ramp_started = true;
    }
    
    lcd->printf_at(0, 1, "RPM: %.0f", current_rpm);
    
    // Wait for spindle to stop
    if (!move_queue->is_active(AXIS_SPINDLE) && 
        !move_queue->has_chunk(AXIS_SPINDLE)) {
        
        state = WindingState::COMPLETE;
        ramp_started = false;
    }
}

void WindingController::update_rpm() {
    uint32_t now = time_us_32();
    uint32_t dt_us = now - last_rpm_update_time;
    
    if (dt_us < 100000) return;  // Update every 100ms
    
    int32_t encoder_pos = encoder->get_position();
    int32_t delta = encoder_pos - last_encoder_position;
    
    float dt_sec = dt_us / 1000000.0f;
    float rps = (delta / (float)ENCODER_CPR) / dt_sec;
    current_rpm = rps * 60.0f;
    
    last_rpm_update_time = now;
}

void WindingController::update_display() {
    if (state == WindingState::WINDING) {
        lcd->printf_at(0, 0, "Winding: %lu/%lu", 
                      turns_completed, params.target_turns);
        lcd->printf_at(0, 1, "RPM: %.0f", current_rpm);
        lcd->printf_at(0, 2, "Layer: %lu/%lu", 
                      current_layer, params.total_layers);
        lcd->printf_at(0, 3, "L-Turns: %lu/%lu",
                      turns_this_layer, params.turns_per_layer);
    }
}

uint32_t WindingController::mm_to_steps(float mm) {
    // Lead screw: 5mm per revolution
    // Motor: 200 steps * microsteps per revolution
    float revs = mm / TRAVERSE_PITCH_MM;
    uint32_t steps = (uint32_t)(revs * 200 * MOTOR_MICROSTEPS);
    return steps;
}

float WindingController::steps_to_mm(uint32_t steps) {
    float revs = steps / (200.0f * MOTOR_MICROSTEPS);
    return revs * TRAVERSE_PITCH_MM;
}