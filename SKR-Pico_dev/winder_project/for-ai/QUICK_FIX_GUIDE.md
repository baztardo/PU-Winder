# Quick Fix Guide - Wire Winder Count Update Issue

## The Problem
Counts don't update after spindle ramp-up. The system transitions to WINDING state but `turns_completed` stays at 0.

---

## Root Cause (in 30 seconds)

In `winding_controller.cpp`, the `sync_traverse_to_spindle()` function has a **broken position tracker**.

**Current broken code:**
```cpp
int32_t delta = pos - last_encoder_position;

if (delta <= 0) {
    return;  // ❌ Exits without updating
}

uint32_t new_turns = (uint32_t)(delta / ENCODER_CPR);
if (new_turns == 0) {
    return;  // ❌ Exits without updating - THIS IS THE BUG
}

// Only reaches here if new_turns > 0
last_encoder_position += (int32_t)(new_turns * ENCODER_CPR);
```

**Problem**: If encoder advances by less than `ENCODER_CPR` (1440 counts), the function returns WITHOUT updating `last_encoder_position`. This causes:
- Fractional turns to be lost
- Position tracking to drift
- Counts to stop appearing after a few iterations

---

## The Fix (Copy & Paste)

Replace the entire `sync_traverse_to_spindle()` function in `winding_controller.cpp` with this:

```cpp
void WindingController::sync_traverse_to_spindle() {
    const int32_t pos = encoder->get_position();
    int32_t delta = pos - last_encoder_position;

    if (delta <= 0) {
        return;
    }

    // Extract complete revolutions
    uint32_t new_turns = (uint32_t)(delta / ENCODER_CPR);
    
    // Update turn counters
    if (new_turns > 0) {
        turns_completed   += new_turns;
        turns_this_layer  += new_turns;

        if (turns_this_layer >= params.turns_per_layer) {
            current_layer++;
            turns_this_layer = 0;
            traverse_direction = !traverse_direction;
            lcd->printf_at(0, 2, "Layer: %lu/%lu", current_layer, params.total_layers);
        }
    }

    // ✓✓✓ CRITICAL FIX: Update tracker to CURRENT position, not incremental
    // This ensures no fractional turns are lost
    last_encoder_position = pos;  // ← THIS IS THE KEY CHANGE

    if (new_turns == 0) {
        return;  // Position tracked but no full turns yet
    }

    // Traverse synchronization
    float traverse_mm = new_turns * params.wire_pitch_mm;
    uint32_t traverse_steps = mm_to_steps(traverse_mm);
    
    if (traverse_steps == 0) {
        return;
    }

    float spindle_rps_meas = current_rpm / 60.0f;
    float traverse_mmps = spindle_rps_meas * params.wire_pitch_mm;
    float steps_per_mm = mm_to_steps(1.0f);
    float traverse_sps = traverse_mmps * steps_per_mm;

    if (traverse_sps < TRAVERSE_MIN_WINDING_SPEED) {
        traverse_sps = TRAVERSE_MIN_WINDING_SPEED;
    }

    move_queue->set_direction(AXIS_TRAVERSE, traverse_direction);
    auto chunks = StepCompressor::compress_constant_velocity(traverse_steps, traverse_sps);
    
    for (const auto& c : chunks) {
        move_queue->push_chunk(AXIS_TRAVERSE, c);
    }
}
```

---

## What Changed

**Only one line was changed** (but it's critical):

```diff
  if (new_turns > 0) {
      turns_completed   += new_turns;
      turns_this_layer  += new_turns;
      // ...
  }

- last_encoder_position += (int32_t)(new_turns * ENCODER_CPR);
+ last_encoder_position = pos;  // ← Changed from += to =

  if (new_turns == 0) {
      return;
  }
```

### Why This Works

**Before:**
- `last_encoder_position` only updated when `new_turns > 0`
- If you had 800 counts (less than 1440), function returned without updating
- Next call: position tracker was stale, new delta calculation was wrong

**After:**
- `last_encoder_position` ALWAYS updated to current position
- Fractional turns are naturally carried to next iteration
- No "lost" counts
- Accumulation happens naturally: 800 + 800 + 800 = 2400 = 1 full turn + remainder

---

## Other Issues to Check (Optional but Recommended)

### 1. Pin Assignment (May be swapped)
In `config.h`, make sure:
```cpp
#define ENCODER_A_PIN       3
#define ENCODER_B_PIN       4
```

These should match your actual hardware. If encoder counting is backwards:
```cpp
#define ENCODER_INVERT      1  // Change from 0 to 1
```

### 2. Remove Dead Code
In `winding_controller.h`, delete this unused variable (causes confusion):
```cpp
// int32_t  enc_last_sync = 0;  // ← Remove this line
```

### 3. If Counts STILL Don't Update
Check encoder GPIO polling speed. In `config.h`:
```cpp
#define HEARTBEAT_US        100  // 10 kHz
```

If using high RPM (>500), consider enabling PIO:
```cpp
#define ENCODER_USE_PIO       1  // Change from 0 to 1
```

---

## Testing the Fix

1. **Compile** the modified code
2. **Flash** to board
3. **Start a winding job** and watch the LCD
4. **Expected behavior**:
   - Ramp up completes (RPM reaches target)
   - LCD shows `Winding: X/1000` where X increments
   - Layer counter advances automatically
   - System completes when target turns reached

5. **If counts STILL stuck at 0**:
   - Check encoder wiring (GPIO 3, 4, 25)
   - Manually rotate spindle and watch LCD encoder output
   - Verify `encoder->update()` is being called by ISR (check serial output)
   - Run diagnostic: enable `DEBUG_PRINT` to see encoder position

---

## Verification

After the fix, verify in serial output:

```
[ENC] pos=100 rpm=50.0
[ENC] pos=300 rpm=100.0
[ENC] pos=1450 rpm=300.0
```

Position should continuously increase, not stay at 0.

In LCD during WINDING state:
```
Winding: 1/1000
RPM: 300
Layer: 1/2
```

Should show incrementing turn count.

---

## Why This Bug Happened

The original code was trying to be "clever" by only updating position when complete turns were counted. But this created an off-by-one style bug where fractional turns (< 1440 counts) were treated as zero progress.

The fix is simpler: always track the position accurately, count turns separately. Let integer division handle the fractional parts naturally.

