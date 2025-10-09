// =============================================================================
// stepcompress.cpp - Step Compression Implementation
// =============================================================================

#include "stepcompress.h"
#include <cmath>
#include <algorithm>

std::vector<StepChunk> StepCompressor::compress_trapezoid(
    uint32_t total_steps,
    double start_vel,
    double cruise_vel,
    double accel,
    double max_err_us)
{
    std::vector<StepChunk> chunks;
    if (total_steps == 0) return chunks;
    
    // Generate absolute step times
    std::vector<uint64_t> times;
    generate_step_times_trapezoid(times, total_steps, start_vel, cruise_vel, accel);

    // Compress using bisect algorithm
    size_t pos = 0;
    while (pos < times.size()) {
        // Binary search for largest chunk that meets error tolerance
        size_t left = pos + 1;
        size_t right = std::min(times.size(), pos + 1000);
        size_t best_end = left;
        
        while (left <= right) {
            size_t mid = (left + right) / 2;
            uint32_t iv;
            int32_t ad;
            double err;
            
            if (fit_chunk(times, pos, mid, iv, ad, err) && err <= max_err_us) {
                best_end = mid;
                left = mid + 1;
            } else {
                if (mid == 0) break;
                right = mid - 1;
            }
        }
        
        // Create chunk
        uint32_t final_iv;
        int32_t final_ad;
        double final_err;
        
        if (fit_chunk(times, pos, best_end, final_iv, final_ad, final_err)) {
            StepChunk c;
            c.interval_us = final_iv;
            c.add_us = final_ad;
            c.count = best_end - pos;
            chunks.push_back(c);
            pos = best_end;
        } else {
            // Fallback: single step chunk
            StepChunk c;
            c.interval_us = 1000;
            c.add_us = 0;
            c.count = 1;
            chunks.push_back(c);
            pos++;
        }
    }
    
    return chunks;
}

std::vector<StepChunk> StepCompressor::compress_constant_velocity(
    uint32_t total_steps,
    double velocity,
    double max_err_us)
{
    // For constant velocity, we can just create one or a few large chunks
    std::vector<StepChunk> chunks;
    
    if (total_steps == 0 || velocity <= 0) return chunks;
    
    uint32_t interval_us = (uint32_t)(1e6 / velocity);
    
    StepChunk c;
    c.interval_us = interval_us;
    c.add_us = 0;  // No acceleration
    c.count = total_steps;
    chunks.push_back(c);
    
    return chunks;
}

void StepCompressor::compress_trapezoid_into(
    std::vector<StepChunk>& out_chunks,
    uint32_t total_steps,
    double start_vel,
    double cruise_vel,
    double accel,
    double /*max_err_us*/)
{
    out_chunks.clear();

    if (!total_steps) return;

    // Mild pre-reserve to avoid huge allocations.
    // Each chunk holds up to 64 steps; cap the reserve to something sane.
    uint32_t est_chunks = total_steps / 64u + 1u;
    if (est_chunks > 512u) est_chunks = 512u;
    out_chunks.reserve(est_chunks);

    // Simple trapezoid integrator without allocating a times[] array.
    // We increment velocity, accumulate time, and immediately emit compressed steps.
    double v = start_vel;
    double a = accel > 0.0 ? accel : 1e-9;
    if (cruise_vel < start_vel) cruise_vel = start_vel;

    StepChunk cur{};
    cur.interval_us = 0;
    cur.count = 0;
    cur.add_us = 0;

    // Keep the last absolute time to compute intervals on the fly.
    double t_abs = 0.0;
    double last_t_abs = 0.0;

    for (uint32_t i = 0; i < total_steps; ++i) {
        // Ramp velocity toward cruise
        v += a;
        if (v > cruise_vel) v = cruise_vel;

        // Advance time for one step (seconds)
        t_abs += 1.0 / v;

        // Convert to interval in microseconds (uint32_t)
        uint32_t interval_us = (uint32_t)((t_abs - last_t_abs) * 1e6 + 0.5);
        last_t_abs = t_abs;

        // Pack into chunks of up to 64 steps
        if (cur.count == 0) {
            cur.interval_us = interval_us;
            cur.add_us = 0;
        }
        ++cur.count;

        if (cur.count >= 64) {
            out_chunks.push_back(cur);
            cur.count = 0;
        }
    }

    if (cur.count) out_chunks.push_back(cur);
}


bool StepCompressor::fit_chunk(
    const std::vector<uint64_t>& times,
    size_t start,
    size_t end,
    uint32_t& out_interval_us,
    int32_t& out_add_us,
    double& out_max_err)
{
    if (end <= start) return false;
    
    size_t N = end - start;
    uint64_t t0 = (start == 0) ? 0ULL : times[start - 1];
    
    // Build normal equations for least-squares fit
    // Model: y_k = interval*k + add*k*(k-1)/2
    double S11 = 0, S12 = 0, S22 = 0, Sy1 = 0, Sy2 = 0;
    
    for (size_t i = 0; i < N; i++) {
        double k = (double)(i + 1);
        double x1 = k;
        double x2 = (k * (k - 1.0)) / 2.0;
        double y = (double)((int64_t)times[start + i] - (int64_t)t0);
        
        S11 += x1 * x1;
        S12 += x1 * x2;
        S22 += x2 * x2;
        Sy1 += x1 * y;
        Sy2 += x2 * y;
    }
    
    double det = S11 * S22 - S12 * S12;
    if (fabs(det) < 1e-12) return false;
    
    // Solve for interval and add
    double interval = (S22 * Sy1 - S12 * Sy2) / det;
    double add = (-S12 * Sy1 + S11 * Sy2) / det;
    
    // Calculate maximum error
    double maxerr = 0.0;
    for (size_t i = 0; i < N; i++) {
        double k = (double)(i + 1);
        double pred = (double)t0 + interval * k + add * (k * (k - 1.0)) / 2.0;
        double err = fabs((double)times[start + i] - pred);
        maxerr = std::max(maxerr, err);
    }
    
    out_interval_us = (uint32_t)llround(interval);
    out_add_us = (int32_t)llround(add);
    out_max_err = maxerr;
    
    return true;
}
// =============================================================================
// Non-allocating version: fills an existing vector instead of returning one
// =============================================================================
void StepCompressor::generate_step_times_trapezoid(
    std::vector<uint64_t>& out_times,
    uint32_t total_steps,
    double start_vel,
    double cruise_vel,
    double accel)
{
    out_times.clear();
    out_times.reserve(total_steps);
    if (total_steps == 0) return;

    double v = start_vel;
    double t = 0.0;
    double a = accel;

    for (uint32_t i = 0; i < total_steps; i++) {
        v += a;
        if (v > cruise_vel) v = cruise_vel;
        t += 1.0 / v;
        out_times.push_back((uint64_t)(t * 1e6)); // convert to µs
    }
}




