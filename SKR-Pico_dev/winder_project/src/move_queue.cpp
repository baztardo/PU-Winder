// =============================================================================
// move_queue.cpp - MCU Move Queue Implementation
// =============================================================================

#include "move_queue.h"
#include "config.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <algorithm>
#include <cstring>

MoveQueue::MoveQueue() {
    memset((void*)head, 0, sizeof(head));
    memset((void*)tail, 0, sizeof(tail));
    memset(active_running, 0, sizeof(active_running));
    memset(step_count, 0, sizeof(step_count));
}

void MoveQueue::init() {
    // Initialize spindle stepper pins
    gpio_init(SPINDLE_STEP_PIN);
    gpio_set_dir(SPINDLE_STEP_PIN, GPIO_OUT);
    gpio_put(SPINDLE_STEP_PIN, 0);
    
    gpio_init(SPINDLE_DIR_PIN);
    gpio_set_dir(SPINDLE_DIR_PIN, GPIO_OUT);
    gpio_put(SPINDLE_DIR_PIN, 0);
    
    gpio_init(SPINDLE_ENA_PIN);
    gpio_set_dir(SPINDLE_ENA_PIN, GPIO_OUT);
    gpio_put(SPINDLE_ENA_PIN, 0);  // Active low - motor enabled
    
    // Initialize traverse stepper pins
    gpio_init(TRAVERSE_STEP_PIN);
    gpio_set_dir(TRAVERSE_STEP_PIN, GPIO_OUT);
    gpio_put(TRAVERSE_STEP_PIN, 0);
    
    gpio_init(TRAVERSE_DIR_PIN);
    gpio_set_dir(TRAVERSE_DIR_PIN, GPIO_OUT);
    gpio_put(TRAVERSE_DIR_PIN, 0);
    
    gpio_init(TRAVERSE_ENA_PIN);
    gpio_set_dir(TRAVERSE_ENA_PIN, GPIO_OUT);
    gpio_put(TRAVERSE_ENA_PIN, 0);  // Active low - motor enabled
}

bool MoveQueue::push_chunk(uint8_t axis, const StepChunk& chunk) {
    if (axis >= NUM_AXES) return false;
    
    uint16_t h = head[axis];
    uint16_t t = tail[axis];
    
    // Check if queue is full
    if (((h + 1) % MOVE_CHUNKS_CAPACITY) == t) {
        return false;
    }
    
    queues[axis][h] = chunk;
    head[axis] = (h + 1) % MOVE_CHUNKS_CAPACITY;
    
    return true;
}

bool MoveQueue::pop_chunk(uint8_t axis, StepChunk& out) {
    if (axis >= NUM_AXES) return false;
    
    uint16_t h = head[axis];
    uint16_t t = tail[axis];
    
    // Check if queue is empty
    if (h == t) return false;
    
    out = queues[axis][t];
    tail[axis] = (t + 1) % MOVE_CHUNKS_CAPACITY;
    
    return true;
}

bool MoveQueue::has_chunk(uint8_t axis) {
    if (axis >= NUM_AXES) return false;
    return head[axis] != tail[axis];
}

uint32_t MoveQueue::get_queue_depth(uint8_t axis) const {
    if (axis >= NUM_AXES) return 0;
    
    uint16_t h = head[axis];
    uint16_t t = tail[axis];
    
    if (h >= t) {
        return h - t;
    } else {
        return MOVE_CHUNKS_CAPACITY - t + h;
    }
}

void MoveQueue::clear_queue(uint8_t axis) {
    if (axis >= NUM_AXES) return;
    tail[axis] = head[axis];
    active_running[axis] = false;
}

void MoveQueue::set_direction(uint8_t axis, bool forward) {
    uint pin = (axis == AXIS_SPINDLE) ? SPINDLE_DIR_PIN : TRAVERSE_DIR_PIN;
    gpio_put(pin, forward ? 1 : 0);
}

void MoveQueue::set_enable(uint8_t axis, bool enable) {
    uint pin = (axis == AXIS_SPINDLE) ? SPINDLE_ENA_PIN : TRAVERSE_ENA_PIN;
    gpio_put(pin, enable ? 0 : 1);  // Active low
}

bool MoveQueue::is_active(uint8_t axis) {
    if (axis >= NUM_AXES) return false;
    return active_running[axis];
}

int32_t MoveQueue::get_step_count(uint8_t axis) {
    if (axis >= NUM_AXES) return 0;
    return step_count[axis];
}

void MoveQueue::execute_step_pulse(uint32_t step_pin) {
    gpio_put(step_pin, 1);
    busy_wait_us(STEP_PULSE_US);
    gpio_put(step_pin, 0);
}

void MoveQueue::axis_isr_handler(uint8_t axis) {
    if (axis >= NUM_AXES) return;
    
    // If not running an active chunk, try to load one
    if (!active_running[axis]) {
        if (head[axis] == tail[axis]) {
            return;  // Queue empty
        }
        
        // Load next chunk
        active[axis] = queues[axis][tail[axis]];
        tail[axis] = (tail[axis] + 1) % MOVE_CHUNKS_CAPACITY;
        active_running[axis] = true;
        last_step_time[axis] = time_us_32();
        return;
    }
    
    // Check if it's time for the next step
    uint32_t now = time_us_32();
    int32_t time_diff = (int32_t)(now - last_step_time[axis]);
    
    if (time_diff < (int32_t)active[axis].interval_us) {
        return;  // Not time yet
    }
    
    // Execute step pulse
    uint step_pin = (axis == AXIS_SPINDLE) ? SPINDLE_STEP_PIN : TRAVERSE_STEP_PIN;
    execute_step_pulse(step_pin);
    
    last_step_time[axis] = now;
    step_count[axis]++;
    
    // Decrement count
    if (active[axis].count > 0) {
        active[axis].count--;
    }
    
    // Update interval with add
    int64_t next_interval = (int64_t)active[axis].interval_us + (int64_t)active[axis].add_us;
    active[axis].interval_us = (uint32_t)std::max((int64_t)1, next_interval);
    
    // Check if chunk finished
    if (active[axis].count == 0) {
        active_running[axis] = false;
    }
}
