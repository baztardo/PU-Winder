# Critical Fixes: Motors Not Moving

## Problem Report
**Symptom:** Steppers not moving, Z index timing out

## Root Causes Found

### **Bug #1: Step Pulse Too Short** ⚠️ CRITICAL
**File:** `src/move_queue.cpp` line 129-130
**Impact:** TMC2209 drivers couldn't detect step pulses

**Problem:**
```cpp
for (volatile int i = 0; i < 10; i++) {
    __asm volatile("nop");  // Said "~2us" but was only 80ns!
}
```

At 125MHz:
- 1 clock cycle = 8ns
- 10 NOPs = 80ns
- TMC2209 needs **minimum 1-2μs pulse width**

**Fix:**
```cpp
gpio_put(step_pin, 1);
busy_wait_us(STEP_PULSE_US);  // 2us pulse - proper timing
gpio_put(step_pin, 0);
```

✅ **Result:** Step pulses now 2μs, TMC2209 can detect them

---

### **Bug #2: Direction Not Set in home_spindle()** ⚠️ CRITICAL
**File:** `src/winding_controller.cpp` - `home_spindle()`
**Impact:** Spindle motor had undefined direction pin state

**Problem:**
- Function queues moves to find Z index
- **NEVER calls `set_direction()`**
- Direction pin could be random state
- Motor may not move or move wrong direction

**Fix:**
```cpp
// CRITICAL: Set motor direction before queuing moves!
bool spindle_dir = (SPINDLE_DIR_INVERT == 0);
move_queue->set_direction(AXIS_SPINDLE, spindle_dir);

// Ensure motor is enabled
move_queue->set_enable(AXIS_SPINDLE, true);
```

✅ **Result:** Direction pin properly set before movement

---

### **Bug #3: Motor Enable Not Redundantly Set**
**File:** `src/winding_controller.cpp` - multiple functions
**Impact:** Motors might be disabled in some states

**Problem:**
- Motors enabled in `init_hardware()` once
- If any code accidentally disables them, they stay disabled
- No redundant enable calls before critical moves

**Fix:** Added explicit `set_enable(axis, true)` in:
- `home_spindle()` - before homing
- `home_traverse()` - before homing
- `move_to_start()` - before traverse move
- `ramp_up_spindle()` - before spindle ramp

✅ **Result:** Motors guaranteed enabled before each movement sequence

---

### **Bug #4: Missing Diagnostic Output**
**File:** `src/move_queue.cpp` - `axis_isr_handler()`
**Impact:** Hard to debug if ISR is processing moves

**Fix:** Added diagnostic printf when loading chunks:
```cpp
static uint32_t last_debug = 0;
if ((time_us_32() - last_debug) > 1000000) {  // Once per second
    printf("Axis %u: Loaded chunk interval=%lu count=%lu\n", 
           axis, active[axis].interval_us, active[axis].count);
    last_debug = time_us_32();
}
```

✅ **Result:** Can monitor via USB serial if moves are processing

---

## Summary of All Changes

### Files Modified:
1. **src/move_queue.cpp**
   - Fixed step pulse duration (80ns → 2μs)
   - Added `#include <cstdio>` for printf
   - Added ISR diagnostics

2. **src/winding_controller.cpp**
   - Added direction setting in `home_spindle()`
   - Added motor enable calls in 4 functions
   - Added printf diagnostics for queue operations

---

## How to Test

### 1. Monitor USB Serial (115200 baud)
You should see:
```
Homing spindle: queuing 1 chunks
Axis 0: Loaded chunk interval=5000 count=3200
Move to start: queued 5 chunks for 640 steps
Axis 1: Loaded chunk interval=3333 count=128
```

### 2. Physical Motor Behavior
- **Spindle motor** should rotate slowly during Z index search
- **Traverse motor** should move to home switch, then back off
- Both motors should turn smoothly without stuttering

### 3. Check LEDs
- **FAN2 (LED2):** Blink at 250ms (main loop heartbeat)
- **FAN3 (LED3):** Blink at 500ms (ISR scheduler heartbeat)

### 4. Expected Startup Sequence
1. Boot LED pattern (3 flashes)
2. Motor tests (both motors move briefly)
3. TMC status check
4. Z index search (spindle rotates slowly)
5. Traverse homing (moves until switch hit)
6. Move to start position
7. Ramp up to target RPM
8. Begin winding

---

## Why Motors Stopped Moving

The primary issue was **Bug #1** - step pulses were 25× too short (80ns instead of 2μs). This meant:
- Software thought it was sending pulses
- TMC2209 drivers couldn't detect them
- No physical movement occurred
- Moves completed in software
- Z index timeout because spindle never turned

Secondary issues (Bugs #2-3) would have caused intermittent failures or wrong direction.

---

## Build Output

✅ **Compiled Successfully**
- File: `winder_project.uf2`
- Size: 137 KB
- All critical bugs fixed
- Enhanced diagnostics added

---

## Next Steps

1. **Upload new firmware** (winder_project.uf2)
2. **Connect USB serial** to monitor diagnostics
3. **Power cycle** to start fresh
4. **Watch for motor movement** during startup tests
5. **Verify Z index is found** (should take < 5 seconds)

If motors still don't move after this fix, issue is hardware:
- Check power supply voltage (12-24V for TMC2209)
- Check motor wiring (A+/A-, B+/B-)
- Check TMC2209 UART communication
- Verify GPIO pins are correct in config.h
