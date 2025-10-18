// =============================================================================
// scheduler.cpp - Hardware Timer ISR Scheduler Implementation
// =============================================================================

#include "scheduler.h"
#include "pico/stdlib.h"
#include "config.h"
#include "pico/time.h"
#include <cstdio>

// Note: Dead code removed - steppers array and scheduler_queue_step were never used
// The actual step execution happens in move_queue->handle_isr_tick()

// Global pointer to scheduler instance for static callback
static Scheduler* g_scheduler_instance = nullptr;

Scheduler::Scheduler(MoveQueue* mq, Encoder* enc)
    : move_queue(mq)
    , spindle_encoder(enc)
    , tick_count(0)
    , interval_us(HEARTBEAT_US)
    , running(false)
    , user_callback(nullptr)
    , user_callback_data(nullptr) {
    
    g_scheduler_instance = this;
}

bool Scheduler::start(uint32_t interval) {
    gpio_init(SCHED_HEARTBEAT_PIN);
    gpio_set_dir(SCHED_HEARTBEAT_PIN, GPIO_OUT);
    gpio_put(SCHED_HEARTBEAT_PIN, 0);

    for (int i = 0; i < 3; i++) {
        gpio_put(SCHED_HEARTBEAT_PIN, 1);
        sleep_ms(100);
        gpio_put(SCHED_HEARTBEAT_PIN, 0);
        sleep_ms(100);
    }

    if (running) return false;
    
    interval_us = interval;
    tick_count = 0;
    
    printf("Starting scheduler ISR at %lu us intervals...\n", interval_us);
    
    // Start repeating timer
    // Negative interval means "call me every N microseconds from now"
    bool success = add_repeating_timer_us(
        -(int32_t)interval_us,
        timer_callback,
        this,
        &timer
    );
    
    if (success) {
        running = true;
        printf("Scheduler ISR started successfully!\n");
    } else {
        printf("ERROR: Failed to start scheduler ISR!\n");
    }
    
    return success;
}

void Scheduler::stop() {
    if (!running) return;
    
    cancel_repeating_timer(&timer);
    running = false;
}

bool Scheduler::is_running() const {
    return running;
}

uint32_t Scheduler::get_tick_count() const {
    return tick_count;
}

uint32_t Scheduler::get_frequency_hz() const {
    if (interval_us == 0) return 0;
    return 1000000 / interval_us;
}

void Scheduler::register_callback(void (*callback)(void*), void* user_data) {
    user_callback = callback;
    user_callback_data = user_data;
}

bool Scheduler::timer_callback(repeating_timer_t* rt) {
    if (g_scheduler_instance) {
        g_scheduler_instance->handle_isr();
    }
    return true;  // Keep repeating
}

void Scheduler::handle_isr() {
    tick_count++;
    
    // Debug: Print first few ISR calls
    static bool isr_debug_done = false;
    if (!isr_debug_done && tick_count <= 3) {
        printf("ISR tick %lu\n", tick_count);
        if (tick_count == 3) isr_debug_done = true;
    }

    // Update encoder state
    if (spindle_encoder) {
        spindle_encoder->update();
    }
    
    // Process move queues for both axes
    if (move_queue) {
        move_queue->handle_isr_tick();
    }
    
    // Call user callback if registered
    if (user_callback) {
        user_callback(user_callback_data);
    }
    // -----------------------------------------------------------------------------
    // Heartbeat LED toggle (safe for ISR)
    // -----------------------------------------------------------------------------
    static uint32_t last_toggle = 0;
    static bool led_state = false;
    if ((tick_count - last_toggle) >= 5000) {   // toggle every ~0.5s at 10kHz
        led_state = !led_state;
        gpio_put(SCHED_HEARTBEAT_PIN, led_state);
        last_toggle = tick_count;
    }
}
