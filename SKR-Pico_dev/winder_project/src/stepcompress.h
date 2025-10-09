// =============================================================================
// stepcompress.h - Klipper-Style Step Compression
// Purpose: Generate compressed step chunks with trapezoidal motion profiles
// =============================================================================

#pragma once

#include <vector>
#include <cstdint>

// =============================================================================
// Step Chunk Structure
// =============================================================================
struct StepChunk {
    uint32_t interval_us;  // Starting interval in microseconds
    int32_t add_us;        // Signed add applied after each step
    uint32_t count;        // Number of steps in this chunk
};

// =============================================================================
// StepCompressor Class
// =============================================================================
class StepCompressor {
public:
    /**
     * @brief Generate compressed step chunks for a trapezoidal move
     * @param total_steps Total number of steps in the move
     * @param start_vel Starting velocity (steps/sec)
     * @param cruise_vel Cruise velocity (steps/sec)
     * @param accel Acceleration (steps/sec²)
     * @param max_err_us Maximum allowed timing error per step (microseconds)
     * @return Vector of compressed step chunks
     */
    static std::vector<StepChunk> compress_trapezoid(
        uint32_t total_steps,
        double start_vel,
        double cruise_vel,
        double accel,
        double max_err_us = 20.0);
    
    /**
     * @brief Generate step chunks for constant velocity move
     * @param total_steps Total number of steps
     * @param velocity Constant velocity (steps/sec)
     * @param max_err_us Maximum allowed timing error
     * @return Vector of compressed step chunks
     */
    static std::vector<StepChunk> compress_constant_velocity(
        uint32_t total_steps,
        double velocity,
        double max_err_us = 20.0);
    
    /**
     * @brief Compress into an existing vector (to avoid allocations)
     * @param out_chunks Output vector to fill with chunks
     * @param total_steps Total number of steps
     * @param start_vel Starting velocity
     * @param cruise_vel Cruise velocity
     * @param accel Acceleration
     * @param max_err_us Maximum allowed timing error
     */
    static void compress_trapezoid_into(
        std::vector<StepChunk>& out_chunks,
        uint32_t total_steps,
        double start_vel,
        double cruise_vel,
        double accel,
        double max_err_us);


private:
    /**
     * @brief Generate absolute step times for trapezoidal profile
     * @param total_steps Total number of steps
     * @param start_vel Starting velocity
     * @param cruise_vel Cruise velocity
     * @param accel Acceleration
     * @return Vector of absolute step times in microseconds
     */
    static void generate_step_times_trapezoid(
        std::vector<uint64_t>& out_times,
        uint32_t total_steps,
        double start_vel,
        double cruise_vel,
        double accel);
    
    /**
     * @brief Fit a chunk using least-squares to a section of step times
     * @param times Vector of absolute step times
     * @param start Start index (inclusive)
     * @param end End index (exclusive)
     * @param out_interval_us Output: fitted interval
     * @param out_add_us Output: fitted add value
     * @param out_max_err Output: maximum error in fit
     * @return true if fit successful
     */
    static bool fit_chunk(
        const std::vector<uint64_t>& times,
        size_t start,
        size_t end,
        uint32_t& out_interval_us,
        int32_t& out_add_us,
        double& out_max_err);
};