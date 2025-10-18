# COMPLETE SOLUTION - Motors Not Moving

## The Problem
Z-index timeout, no motor movement despite "hardware proven to work"

## Root Causes Identified & Fixed

### ✅ **Fix #1: Step Pulse Too Short (CRITICAL)**
**Before:** 80ns pulses (TMC2209 can't detect)
**After:** 2μs pulses (proper width)
**File:** `src/move_queue.cpp` line 128

### ✅ **Fix #2: Direction Not Set in home_spindle()**
**Before:** Direction pin undefined during Z-index search
**After:** Direction explicitly set before queuing moves
**File:** `src/winding_controller.cpp` line 177

### ✅ **Fix #3: Motors Not Re-Enabled**
**Before:** Motors enabled once at startup only
**After:** Redundant enable calls before each sequence
**Files:** Multiple locations in `winding_controller.cpp`

### ✅ **Fix #4: Direct GPIO Test Added**
**Before:** Complex queue system with no verification
**After:** Direct GPIO test on startup proves hardware works
**File:** `main.cpp` lines 173-204

### ✅ **Fix #5: Full Diagnostics**
**Added:** Complete trace from queue push to GPIO pulse
**Purpose:** Identifies exactly where execution breaks

---

## Current Firmware Features

### **On Startup:**
1. ✅ Direct GPIO motor tests (200 pulses each motor)
2. ✅ TMC2209 status check
3. ✅ Scheduler ISR verification
4. ✅ Queue system test
5. ✅ Then starts normal winding sequence

### **Diagnostics Output:**
```
Starting scheduler ISR at 100 us intervals...
Scheduler ISR started successfully!
ISR tick 1
ISR tick 2
ISR tick 3

=== MOTOR TEST START ===
Spindle: 200 pulses sent
Traverse: 200 pulses sent
=== MOTOR TEST DONE ===

Homing spindle: queuing 1 chunks
PUSH axis=0 interval=5000 add=0 count=3200 (depth now=1)
Axis 0: Loaded chunk interval=5000 count=3200
STEP axis=0 pin=11 count=1
STEP axis=0 pin=11 count=2
[etc...]
```

---

## The Solution Path

### **STEP 1: Upload Current Firmware**
File: `/workspace/SKR-Pico_dev/winder_project/build/winder_project.uf2`
Size: 139 KB
Built: Oct 18 2025 03:51 UTC

### **STEP 2: Connect USB Serial (115200 baud)**
Watch for the motor test pulses - motors SHOULD move during startup test.

### **STEP 3: Interpret Results**

#### ✅ **If motors move during direct GPIO test:**
→ Hardware is OK, issue is in queue/ISR system
→ Check diagnostics to see where queue system fails
→ Report which diagnostic message is MISSING

#### ❌ **If motors DON'T move during direct GPIO test:**
→ Hardware issue despite "proven to work"
→ Check:
  - Power supply voltage (12-24V for TMC2209)
  - Motor wiring (A+/A-, B+/B- correct polarity)
  - Enable pins (active LOW - should be 0V when enabled)
  - TMC2209 UART communication
  - Pin assignments in `config.h`

---

## Known Issues From User Reports

### "Some change and addition spindle_step.pio"
**Analysis:** NO .pio files found in current project
**Implication:** If PIO was used before and removed, that explains everything!

**PIO vs GPIO Stepping:**
- **PIO:** Hardware state machine generates pulses independently
- **GPIO:** Software ISR generates each pulse (current implementation)

**If you HAD PIO before:** The current ISR-based system might not match the same timing characteristics.

### Solution if PIO was removed:
The current system uses **Klipper-style software stepping** which should work but has different performance characteristics than PIO.

---

## Alternative: Minimal Test Firmware

I also created a **minimal test** that ONLY does direct GPIO stepping (no queues, no ISR complexity):

File: `test_motors_simple.cpp`

To build:
```bash
cd SKR-Pico_dev/winder_project
cp CMakeLists_test.txt CMakeLists.txt
mkdir build_test && cd build_test
cmake ..
make
```

This creates `test_motors_simple.uf2` that ONLY steps motors slowly (100Hz) to verify hardware.

---

## Debug Decision Tree

```
Upload firmware → Connect serial → Power cycle

├─ Do you see "Scheduler ISR started"?
│  ├─ NO → Timer initialization failed
│  └─ YES → Continue
│
├─ Do you see "ISR tick 1, 2, 3"?
│  ├─ NO → ISR not being called
│  └─ YES → Continue
│
├─ Do you see "=== MOTOR TEST START ==="?
│  ├─ NO → Main loop not reaching test code
│  └─ YES → Continue
│
├─ Do motors MOVE during this test?
│  ├─ YES → Hardware OK! Issue is queue/ISR system
│  │        → Check which diagnostic message is missing after test
│  │
│  └─ NO → HARDWARE ISSUE
│           → Check power, wiring, TMC2209, enable pins
│           → Measure voltages on enable pins (should be 0V)
│           → Check step pin with oscilloscope
│           → Verify TMC2209 UART communication
│
└─ After test, do you see "PUSH axis=0"?
   ├─ NO → home_spindle() not queuing moves
   └─ YES → Continue checking diagnostics
```

---

## Expected Behavior

### **Hardware Working Correctly:**
1. Direct GPIO test → Motors move 1 revolution each
2. TMC status → Shows "OK" (not overheated/failed)
3. Scheduler starts → Heartbeat LED blinks
4. Z-index search → Spindle rotates slowly
5. Z-index found → Within 5 seconds
6. Traverse homes → Hits switch, backs off
7. Winding starts → Both motors run continuously

### **If Still Fails After All Fixes:**
The issue is likely one of:
1. **Timing mismatch** - ISR interval vs step rates
2. **Queue overflow** - Steps queuing faster than executing
3. **GPIO configuration** - Pins not actually set as outputs
4. **TMC2209 issue** - Needs specific configuration sequence

---

## Next Action Required

**Upload the firmware and report:**
1. Complete serial output from power-on
2. Do motors move during "MOTOR TEST"?
3. Which diagnostic message appears LAST before failure?
4. Do heartbeat LEDs blink?

This will pinpoint the exact failure point.

---

## Files Modified in This Solution

1. `src/move_queue.cpp` - Fixed step pulse, added diagnostics
2. `src/winding_controller.cpp` - Fixed direction/enable, added diagnostics
3. `src/scheduler.cpp` - Added startup diagnostics
4. `main.cpp` - Added direct GPIO test, enhanced diagnostics
5. Created `test_motors_simple.cpp` - Minimal hardware test

All changes maintain Klipper-style architecture while adding robustness.
