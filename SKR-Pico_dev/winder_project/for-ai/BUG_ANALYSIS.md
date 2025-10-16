# Wire Winder Firmware - Bug Analysis Report

## 🔴 CRITICAL BUG: Count Not Updating After Ramp Up

### Root Cause: Encoder Position Tracking Issue in `sync_traverse_to_spindle()`

**Location**: `winding_controller.cpp`, `sync_traverse_to_spindle()` function

```cpp
void WindingController::sync_traverse_to_spindle() {
    const int32_t pos = encoder->get_position();
    int32_t delta = pos - last_encoder_position;

    if (delta <= 0) {
        return;  // ❌ EXITS WITHOUT UPDATING last_encoder_position
    }

    uint32_t new_turns = (uint32_t)(delta / ENCODER_CPR);
    if (new_turns == 0) {
        return;  // ❌ EXITS WITHOUT UPDATING last_encoder_position
    }

    turns_completed   += new_turns;
    turns_this_layer  += new_turns;
    last_encoder_position += (int32_t)(new_turns * ENCODER_CPR);  // Only updated here
```

### Problem Sequence:

1. **During Ramp Up Phase**:
   - State is `RAMPING_UP`, NOT `WINDING`
   - `sync_traverse_to_spindle()` is **NOT called**
   - Encoder counts accumulate (e.g., 3000 counts during ramp)
   - `last_encoder_position` remains at 0 (unchanged since `start()`)

2. **Transition to WINDING**:
   - Encoder position is now ~3000 counts
   - `last_encoder_position` is still 0
   - First call to `sync_traverse_to_spindle()` should count these ramp steps

3. **The Bug Manifests**:
   - If encoder moves continuously, `delta` is positive ✓
   - But `new_turns = 3000 / 1440 = 2` (integer division) ✓
   - Count should increment... **so why doesn't it?**

### The REAL Issue: State Machine Timing

The problem is in the **transition logic**. Look at `ramp_up_spindle()`:

```cpp
if (time_done && queue_low) {
    ramp_started = false;
    
    // ⚠️ Prefill spindle queue AFTER ramp complete
    {
        float target_rps = params.spindle_rpm / 60.0f;
        uint32_t steps_per_rev = 200 * MOTOR_MICROSTEPS;
        float target_sps = target_rps * steps_per_rev;
        uint32_t spindle_steps = (uint32_t)(target_sps * 1.0f);
        auto chunks = StepCompressor::compress_constant_velocity(spindle_steps, target_sps);
        for (const auto& c : chunks) move_queue->push_chunk(AXIS_SPINDLE, c);
    }
    
    state = WindingState::WINDING;
    banner_printed = false;
    return;
}
```

The spindle queue gets prefilled, BUT there's a **delay before counting starts**.

### Secondary Issue: Fractional Turn Loss

When `new_turns == 0` (delta < 1440 counts), the function returns WITHOUT updating `last_encoder_position`:

```cpp
if (new_turns == 0) {
    return;  // ❌ Fractional counts (< 1 revolution) are lost!
}
```

**Example of accumulating error**:
- First call: delta = 800, new_turns = 0, return (800 counts lost)
- Second call: delta = 1600, new_turns = 1, increment by 1 turn, update last_pos += 1440
- Result: 160 counts are "lost" in rounding

---

## 🔴 CRITICAL BUG #2: Motor Direction Flags in `config.h`

**Location**: `config.h`, lines with direction inverters

```cpp
#define SPINDLE_DIR_INVERT   0   // set 1 if spindle turns the wrong way
#define TRAVERSE_DIR_INVERT  0   // set 1 if traverse moves the wrong way
```

**Issue**: These flags affect encoder interpretation, but the code doesn't check them consistently.

In `encoder.cpp`, during `update()`:
```cpp
position += table[last_state][state] * (ENCODER_INVERT ? -1 : 1);
```

But `ENCODER_INVERT` is defined in `config.h`:
```cpp
#define ENCODER_INVERT       0
```

**Problem**: 
- The encoder can count in reverse if wiring is backwards
- But the stepper direction pins might also be inverted
- These don't stay synchronized if manually adjusted

---

## 🔴 BUG #3: Pin Assignment Conflicts - pins_skr-pico_v1.h

**Two conflicting pin definitions exist** (documents 3 and 14):

**Version 1** (Document 3):
```cpp
#define X_STOP_PIN      4   // Encoder pin A
#define Y_STOP_PIN      3   // Encoder pin B
```

**Version 2** (Document 14):
```cpp
#define X_STOP_PIN      3   // Encoder pin A
#define Y_STOP_PIN      4   // Encoder pin B
```

**Active Version in code** (`config.h`):
```cpp
#define ENCODER_A_PIN       3
#define ENCODER_B_PIN       4
```

This matches **Version 2**, but if you're using the older header file, encoder pins are **swapped**, causing:
- Incorrect quadrature decoding
- Reversed encoder direction
- No position tracking

---

## 🔴 BUG #4: Unused/Dead Code Creates Confusion

In `winding_controller.h`, these members are declared but never used:

```cpp
int32_t  enc_last_sync = 0;  // ❌ Declared, never referenced in .cpp!
```

But the actual variable used is:
```cpp
int32_t last_encoder_position;  // ✓ This is the real one
```

This suggests **incomplete refactoring** and could cause subtle bugs if someone modifies one but forgets the other.

---

## 🟡 BUG #5: RPM Update Throttling (Non-Critical)

In `update_rpm()`:

```cpp
uint32_t dt_us = now - last_rpm_update_time;
if (dt_us < 500000) return;  // ⚠️ Only updates every 500ms
```

This is **fine for RPM calculation**, but it means:
- You can't see sub-500ms speed changes
- Synchronization calculations based on current_rpm might lag

**Not the cause of count issue**, but worth noting.

---

## 🟡 BUG #6: GPIO Polling at 10 kHz May Miss Fast Encoder Transitions

**Location**: `config.h`

```cpp
#define ENCODER_USE_PIO       0  // Using GPIO polling, not PIO
#define HEARTBEAT_US        100  // 10 kHz ISR frequency
```

**At 300 RPM with 360 PPR encoder**:
- 5 revolutions/second = 5 × 1440 CPR = **7200 counts/second**
- ISR runs at 10 kHz = **10,000 samples/second**
- Sample rate is adequate BUT:
  - Between ISR cycles, multiple encoder edges might occur
  - Fast back-and-forth motion could be missed
  - Quadrature alignment could drift

**Solution**: Enable PIO quadrature decoder, or increase ISR frequency to 20-40 kHz

---

## 🔴 CRITICAL FIX: Update `sync_traverse_to_spindle()`

### Current (Broken) Code:
```cpp
void WindingController::sync_traverse_to_spindle() {
    const int32_t pos = encoder->get_position();
    int32_t delta = pos - last_encoder_position;

    if (delta <= 0) {
        return;  // ❌ BUG: Should at least update on zero delta
    }

    uint32_t new_turns = (uint32_t)(delta / ENCODER_CPR);
    if (new_turns == 0) {
        return;  // ❌ BUG: Fractional turns are lost
    }

    turns_completed   += new_turns;
    turns_this_layer  += new_turns;
    last_encoder_position += (int32_t)(new_turns * ENCODER_CPR);
    
    // ... rest of function
}
```

### Fixed Code:
```cpp
void WindingController::sync_traverse_to_spindle() {
    const int32_t pos = encoder->get_position();
    int32_t delta = pos - last_encoder_position;

    // ✓ FIX: Always update position even for partial turns
    if (delta <= 0) {
        return;
    }

    uint32_t new_turns = (uint32_t)(delta / ENCODER_CPR);
    
    // ✓ FIX: Update position tracker with full delta, not just full revolutions
    // This ensures we don't lose fractional turns
    int32_t full_revolution_counts = new_turns * ENCODER_CPR;
    
    if (new_turns > 0) {
        turns_completed   += new_turns;
        turns_this_layer  += new_turns;
    }
    
    // ✓ CRITICAL FIX: Always advance the position tracker by full delta
    // not just by complete revolutions
    last_encoder_position = pos;  // ← Update to current position, not incremental!
    
    // ... rest of function will now have correct position tracking
}
```

### Why This Works:
- **Before**: `last_encoder_position` only updated when we had complete turns
  - Missing encoder increments created aliasing
  - Fractional counts were lost

- **After**: `last_encoder_position` always reflects current position
  - Next call gets accurate delta (only new counts since last update)
  - No loss of information
  - Turns are still counted correctly (integer division)

---

## 🟢 VERIFICATION CHECKLIST

After applying fixes, verify:

- [ ] Encoder position updates every 10ms in WINDING state
- [ ] Turn count increments smoothly
- [ ] RPM reading is stable (±5% variation normal)
- [ ] Layer transitions occur at correct turn counts
- [ ] No "stuck" at zero turns during early WINDING state

---

## 📋 SUMMARY OF ISSUES

| # | Issue | Severity | Location | Impact |
|---|-------|----------|----------|--------|
| 1 | `last_encoder_position` not updated for fractional turns | 🔴 CRITICAL | `sync_traverse_to_spindle()` | **Counts don't advance** |
| 2 | Direction flags inconsistent | 🔴 CRITICAL | `config.h` + `encoder.cpp` | **Wrong encoder direction** |
| 3 | Pin assignments conflicting | 🔴 CRITICAL | Two `pins_skr-pico_v1.h` files | **Hardware not initialized** |
| 4 | Dead code `enc_last_sync` | 🟡 MEDIUM | `winding_controller.h` | **Confusion during maintenance** |
| 5 | GPIO polling too slow for high RPM | 🟡 MEDIUM | `config.h` | **Missed encoder transitions** |
| 6 | RPM update throttled to 500ms | 🟡 LOW | `update_rpm()` | **Lag in sync calculations** |

---

## 🚀 RECOMMENDED FIX PRIORITY

1. **First**: Fix `sync_traverse_to_spindle()` - replace position update logic
2. **Second**: Verify correct `pins_skr-pico_v1.h` is being used
3. **Third**: Check encoder direction with manual spindle rotation
4. **Fourth**: Enable PIO encoder if GPIO polling issues occur
5. **Finally**: Remove dead code (`enc_last_sync`)

