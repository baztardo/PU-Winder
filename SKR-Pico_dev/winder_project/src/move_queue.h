// =============================================================================
// move_queue.h - MCU Move Queue & Step Consumer
// Purpose: Fixed-size move queue with ISR-driven step execution
// =============================================================================

#pragma once

#include "stepcompress.h"
#include "config.h"
#include <cstdint>

// =============================================================================
// MoveQueue Class
// =============================================================================
class MoveQueue {
public:
    /**
     * @brief Constructor
     */
    MoveQueue();
    
    /**
     * @brief Initialize GPIO pins for stepper control
     */
    void init();
    
    /**
     * @brief Push a step chunk to an axis queue
     * @param axis Axis identifier (AXIS_SPINDLE or AXIS_TRAVERSE)
     * @param chunk Step chunk to queue
     * @return true if successful, false if queue full
     */
    bool push_chunk(uint8_t axis, const StepChunk& chunk);
    
    /**
     * @brief Pop a chunk from an axis queue
     * @param axis Axis identifier
     * @param out Output chunk
     * @return true if successful, false if queue empty
     */
    bool pop_chunk(uint8_t axis, StepChunk& out);
    
    /**
     * @brief Check if axis has chunks in queue
     * @param axis Axis identifier
     * @return true if queue has chunks
     */
    bool has_chunk(uint8_t axis);
    
    /**
     * @brief Clear all chunks from an axis queue
     * @param axis Axis identifier
     */
    void clear_queue(uint8_t axis);
    
    /**
     * @brief ISR handler for axis stepping
     * Call this from ISR at high frequency (e.g., 10 kHz)
     * @param axis Axis identifier
     */
    void axis_isr_handler(uint8_t axis);
    
    /**
     * @brief Get current queue depth
     * @param axis Axis identifier
     * @return Number of chunks in queue
     */
    void handle_isr_tick();

    /**
     * @brief Set direction pin state
     * @param axis Axis identifier
     * @param forward true for forward, false for reverse
     */
    void set_direction(uint8_t axis, bool forward);
    
    /**
     * @brief Enable/disable motor
     * @param axis Axis identifier
     * @param enable true to enable motor
     */
    void set_enable(uint8_t axis, bool enable);
    
    /**
     * @brief Check if axis is currently executing steps
     * @param axis Axis identifier
     * @return true if actively stepping
     */
    bool is_active(uint8_t axis);
    
    /**
     * @brief Get total steps executed on axis since init
     * @param axis Axis identifier
     * @return Total step count
     */
    int32_t get_step_count(uint8_t axis);

    /**
     * @brief Get current queue depth
     * @param axis Axis identifier
     * @return Number of chunks in queue
     */ 
    uint32_t get_queue_depth(uint8_t axis) const;

private:
    StepChunk queues[2][128];  // NUM_AXES x MOVE_CHUNKS_CAPACITY
    volatile uint16_t head[2];
    volatile uint16_t tail[2];
    
    StepChunk active[2];
    bool active_running[2];
    uint32_t last_step_time[2];
    int32_t step_count[2];
    
    /**
     * @brief Execute a step pulse on given pin
     * @param step_pin GPIO pin number
     */
    void execute_step_pulse(uint32_t step_pin);
};