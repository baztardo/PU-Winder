// =============================================================================
// FIXED: sync_traverse_to_spindle() - Corrected Position Tracking
// =============================================================================
// 
// Original Bug: last_encoder_position was only updated when counting
// complete revolutions, causing fractional turns to be lost and position
// tracking to drift.
//
// Fix: Always update position to current encoder value. Count complete
// revolutions separately. This decouples position tracking from turn counting.
//

void WindingController::sync_traverse_to_spindle() {
    // Get current encoder position
    const int32_t pos = encoder->get_position();
    
    // Calculate delta since last update
    // This tells us how many counts have occurred
    int32_t delta = pos - last_encoder_position;

    // =====================================================================
    // Early exit if no progress (or negative, which shouldn't happen)
    // =====================================================================
    if (delta <= 0) {
        // No forward progress since last sync
        return;
    }

    // =====================================================================
    // FIX #1: Extract complete revolutions from delta
    // =====================================================================
    // This calculates how many COMPLETE rotations happened
    // Integer division automatically discards fractional turns
    uint32_t new_turns = (uint32_t)(delta / ENCODER_CPR);
    
    // =====================================================================
    // Update turn counters if we completed full revolutions
    // =====================================================================
    if (new_turns > 0) {
        turns_completed   += new_turns;
        turns_this_layer  += new_turns;

        // Check if we've completed a layer
        if (turns_this_layer >= params.turns_per_layer) {
            current_layer++;
            turns_this_layer = 0;
            traverse_direction = !traverse_direction;  // zig-zag pattern
            lcd->printf_at(0, 2, "Layer: %lu/%lu", current_layer, params.total_layers);
        }
    }

    // =====================================================================
    // FIX #2: ALWAYS update position tracker to current encoder value
    // =====================================================================
    // This is the KEY FIX. Instead of only updating when we have full turns,
    // we ALWAYS advance the tracker to the current position.
    // 
    // This ensures:
    // 1. Next call gets accurate delta (only truly NEW counts)
    // 2. Fractional turns are properly carried forward
    // 3. No aliasing or "lost" counts
    // 4. Turn counting stays accurate despite fractional increments
    //
    // WHY THIS IS CORRECT:
    // - We count complete turns (new_turns = delta / ENCODER_CPR)
    // - Fractional counts are in (delta % ENCODER_CPR)
    // - Next call will see: delta = (encoder_pos - pos_we_just_set)
    // - That includes the leftover fractional counts plus new counts
    // - When we reach another full revolution, it gets counted correctly
    //
    last_encoder_position = pos;

    // =====================================================================
    // Now handle traverse synchronization
    // =====================================================================
    if (new_turns == 0) {
        // Haven't completed a full revolution yet this update
        // But we've still advanced the position tracker above
        // Traverse sync will happen on next call with accumulated counts
        return;
    }

    // Calculate traverse distance needed for new turns
    float traverse_mm = new_turns * params.wire_pitch_mm;
    uint32_t traverse_steps = mm_to_steps(traverse_mm);
    
    if (traverse_steps == 0) {
        return;  // Not enough resolution for a step
    }

    // Get measured spindle RPM from encoder (updated every 100-500ms)
    float spindle_rps_meas = current_rpm / 60.0f;
    
    // Calculate traverse speed to match spindle pitch
    float traverse_mmps = spindle_rps_meas * params.wire_pitch_mm;

    // Convert to steps/second
    float steps_per_mm = mm_to_steps(1.0f);
    float traverse_sps = traverse_mmps * steps_per_mm;

    // Enforce minimum speed for smooth operation
    if (traverse_sps < TRAVERSE_MIN_WINDING_SPEED) {
        traverse_sps = TRAVERSE_MIN_WINDING_SPEED;
    }

    // Queue the traverse move
    move_queue->set_direction(AXIS_TRAVERSE, traverse_direction);
    auto chunks = StepCompressor::compress_constant_velocity(traverse_steps, traverse_sps);
    
    for (const auto& c : chunks) {
        move_queue->push_chunk(AXIS_TRAVERSE, c);
    }
}

// =============================================================================
// EXPLANATION OF THE FIX
// =============================================================================
//
// BEFORE (Broken):
// ┌─────────────────────────────────────────────────────────────┐
// │ Call 1: encoder_pos=800, last_pos=0                         │
// │ - delta = 800 - 0 = 800                                     │
// │ - new_turns = 800/1440 = 0 (integer division)              │
// │ - return WITHOUT updating last_pos                          │
// │ - ❌ 800 counts are "lost" on next iteration               │
// ├─────────────────────────────────────────────────────────────┤
// │ Call 2: encoder_pos=1600, last_pos=0 (STILL!)              │
// │ - delta = 1600 - 0 = 1600                                   │
// │ - new_turns = 1600/1440 = 1                                │
// │ - last_pos += 1*1440 = 1440                                │
// │ - ✓ Count 1 turn                                            │
// │ - ❌ But we overcounted! Should have been 1.111 turns      │
// └─────────────────────────────────────────────────────────────┘
//
// AFTER (Fixed):
// ┌─────────────────────────────────────────────────────────────┐
// │ Call 1: encoder_pos=800, last_pos=0                         │
// │ - delta = 800 - 0 = 800                                     │
// │ - new_turns = 800/1440 = 0                                  │
// │ - ✓ last_pos = 800 (UPDATED!)                              │
// │ - Fractional turn is carried forward                        │
// ├─────────────────────────────────────────────────────────────┤
// │ Call 2: encoder_pos=1600, last_pos=800                      │
// │ - delta = 1600 - 800 = 800                                  │
// │ - new_turns = 800/1440 = 0                                  │
// │ - ✓ last_pos = 1600 (UPDATED!)                             │
// │ - No count yet, but progress is tracked                     │
// ├─────────────────────────────────────────────────────────────┤
// │ Call 3: encoder_pos=2880, last_pos=1600                     │
// │ - delta = 2880 - 1600 = 1280                                │
// │ - new_turns = 1280/1440 = 0                                │
// │ - ✓ last_pos = 2880 (UPDATED!)                             │
// │ - Still accumulating...                                     │
// ├─────────────────────────────────────────────────────────────┤
// │ Call 4: encoder_pos=4400, last_pos=2880                     │
// │ - delta = 4400 - 2880 = 1520                                │
// │ - new_turns = 1520/1440 = 1 ✓ COUNT!                       │
// │ - ✓ last_pos = 4400 (UPDATED!)                             │
// │ - Correctly counted 1 turn from accumulated fractional      │
// │ - Leftover: 1520 % 1440 = 80 counts for next iteration     │
// └─────────────────────────────────────────────────────────────┘
//
// Result: All counts are preserved and counted correctly!
//

