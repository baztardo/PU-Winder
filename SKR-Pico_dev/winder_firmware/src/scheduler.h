// =============================================================================
// scheduler.h - Hardware Timer ISR Scheduler
// Purpose: Manage high-frequency ISR for step timing and encoder updates
// =============================================================================

#pragma once

#include "pico/stdlib.h"
#include "move_queue.h"
#include "encoder.h"
#include <cstdint>

// Forward declarations
class MoveQueue;
class Encoder;

// =============================================================================
// Scheduler Class
// =============================================================================
class Scheduler {
public:
    /**
     * @brief Constructor
     * @param move_queue Pointer to move queue instance
     * @param spindle_encoder Pointer to spindle encoder instance
     */
    Scheduler(MoveQueue* move_queue, Encoder* spindle_encoder);
    
    /**
     * @brief Initialize and start the hardware timer ISR
     * @param interval_us ISR interval in microseconds
     * @return true if successful
     */
    bool start(uint32_t interval_us);
    
    /**
     * @brief Stop the hardware timer ISR
     */
    void stop();
    
    /**
     * @brief Check if scheduler is running
     * @return true if ISR is active
     */
    bool is_running() const;
    
    /**
     * @brief Get ISR execution count
     * @return Number of times ISR has executed
     */
    uint32_t get_tick_count() const;
    
    /**
     * @brief Get ISR frequency in Hz
     * @return Frequency in Hz
     */
    uint32_t get_frequency_hz() const;
    
    /**
     * @brief Register a callback function to be called from ISR
     * @param callback Function pointer
     * @param user_data User data passed to callback
     */
    void register_callback(void (*callback)(void*), void* user_data);

private:
    MoveQueue* move_queue;
    Encoder* spindle_encoder;
    repeating_timer_t timer;
    volatile uint32_t tick_count;
    uint32_t interval_us;
    bool running;
    
    void (*user_callback)(void*);
    void* user_callback_data;
    
    /**
     * @brief Static ISR callback (required for Pico SDK)
     * @param rt Repeating timer handle
     * @return true to keep repeating
     */
    static bool timer_callback(repeating_timer_t* rt);
    
    /**
     * @brief Instance ISR handler
     */
    void handle_isr();
};