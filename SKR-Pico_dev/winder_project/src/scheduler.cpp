// =============================================================================
// scheduler.cpp - Hardware Timer ISR Scheduler Implementation
// =============================================================================

#include "scheduler.h"
#include "pico/stdlib.h"
#include "config.h"


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

    // Update encoder state
    if (spindle_encoder) {
        spindle_encoder->update();
    }
    
    // Process move queues for both axes
    if (move_queue) {
        move_queue->axis_isr_handler(AXIS_SPINDLE);
        move_queue->axis_isr_handler(AXIS_TRAVERSE);
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
    if ((tick_count - last_toggle) >= 500) {   // toggle every ~0.5s
        led_state = !led_state;
        gpio_put(SCHED_HEARTBEAT_PIN, led_state);
        last_toggle = tick_count;
    }

}