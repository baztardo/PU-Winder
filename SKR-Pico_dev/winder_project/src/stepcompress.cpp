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
    auto times = generate_step_times_trapezoid(total_steps, start_vel, cruise_vel, accel);
    
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

// =============================================================================
// Static memory version to avoid repeated heap allocation
// =============================================================================
std::vector<uint64_t>& StepCompressor::generate_step_times_trapezoid(
    uint32_t total_steps, double start_vel, double cruise_vel, double accel)
{
    static std::vector<uint64_t> times;
    times.clear();
    times.reserve(total_steps);

    if (total_steps == 0) return times;
    
    double v0 = start_vel;
    double v = cruise_vel;
    double a = std::max(accel, 1e-9);
    
    // Calculate distances for each phase
    double s_acc = (v * v - v0 * v0) / (2.0 * a);
    s_acc = std::max(0.0, s_acc);
    
    double s_dec = (v * v) / (2.0 * a);
    double s_total = (double)total_steps;
    double s_cruise = s_total - s_acc - s_dec;
    
    // Check if we need triangle profile
    if (s_cruise < 0.0) {
        // Triangle profile - no cruise phase
        double vp2 = ((2.0 * a * s_total) + v0 * v0) / 2.0;
        double vp = sqrt(std::max(0.0, vp2));
        s_acc = (vp * vp - v0 * v0) / (2.0 * a);
        s_dec = (vp * vp) / (2.0 * a);
        s_cruise = 0.0;
        v = vp;
    }
    
    double acc_end = s_acc;
    double cruise_end = s_acc + s_cruise;
    
    // Generate times for each step
    for (uint32_t n = 1; n <= total_steps; ++n) {
        double s = (double)n;
        double t_abs = 0.0;
        
        if (s <= acc_end + 1e-12) {
            // Acceleration phase: s = v0*t + 0.5*a*t²
            double A = 0.5 * a;
            double B = v0;
            double C = -s;
            double disc = B * B - 4 * A * C;
            t_abs = (-B + sqrt(std::max(0.0, disc))) / (2 * A);
        }
        else if (s <= cruise_end + 1e-12) {
            // Cruise phase
            double t_acc = (v0 == 0) ? sqrt(2 * acc_end / a) : (v - v0) / a;
            t_abs = t_acc + (s - acc_end) / v;
        }
        else {
            // Deceleration phase
            double t_acc = (v0 == 0) ? sqrt(2 * acc_end / a) : (v - v0) / a;
            double t_cruise = (cruise_end - acc_end) / v;
            double s_into_dec = s - cruise_end;
            double t_dec = (v - sqrt(v * v - 2 * a * s_into_dec)) / a;
            t_abs = t_acc + t_cruise + t_dec;
        }
        
        times.push_back((uint64_t)(t_abs * 1e6));  // Convert to microseconds
    }
    
    return times;
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
// Static-memory variant to fill an existing buffer instead of allocating new
// =============================================================================
void StepCompressor::compress_trapezoid_into(
    std::vector<StepChunk>& out_chunks,
    uint32_t total_steps,
    double start_vel,
    double cruise_vel,
    double accel,
    double max_err_us)
{
    out_chunks.clear();

    if (total_steps == 0) return;

    // Precompute times into a static buffer to avoid repeated allocations
    static std::vector<uint64_t> times;
    times.clear();
    times.reserve(total_steps);

    double v0 = start_vel;
    double v = cruise_vel;
    double a = std::max(accel, 1e-9);

    // Generate step times (reusing the algorithm)
    times = generate_step_times_trapezoid(total_steps, start_vel, cruise_vel, accel);

    size_t pos = 0;
    while (pos < times.size()) {
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

        uint32_t final_iv;
        int32_t final_ad;
        double final_err;

        if (fit_chunk(times, pos, best_end, final_iv, final_ad, final_err)) {
            StepChunk c;
            c.interval_us = final_iv;
            c.add_us = final_ad;
            c.count = best_end - pos;
            out_chunks.push_back(c);
            pos = best_end;
        } else {
            StepChunk c;
            c.interval_us = 1000;
            c.add_us = 0;
            c.count = 1;
            out_chunks.push_back(c);
            pos++;
        }
    }
}
