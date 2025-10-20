# 🔍 COMPREHENSIVE CODE AUDIT - v1.8.3

## 🚨 CRITICAL ISSUES FOUND:

### **1. HARDCODED SPEED LIMITS (3 locations!)**
**File:** `src/winding_controller.cpp`  
**Lines:** 345, 375, 406  
**Problem:** `const float max_sps = 50000.0f;` repeated 3 times!

**Impact:**
- If you want to change max speed, must edit 3 places
- Easy to miss one location
- Inconsistency risk

**Fix:** Add to config.h:
```cpp
#define SPINDLE_MAX_SPS  50000.0f  // Maximum steps/sec for spindle
```

---

### **2. WRONG MICROSTEPPING IN Z-HOMING!**
**File:** `src/winding_controller.cpp`  
**Line:** 172  
**Problem:**
```cpp
uint32_t steps_per_rev = 200 * MOTOR_MICROSTEPS;  // ❌ Uses 16x!
```

**Impact:**
- Z-homing uses **16x microstepping**
- But spindle is **4x microstepping**
- Wrong speed calculation!
- Z-homing runs at wrong RPM!

**Fix:** Change to `SPINDLE_MICROSTEPS`:
```cpp
uint32_t steps_per_rev = 200 * SPINDLE_MICROSTEPS;  // ✅ 4x
```

---

### **3. HARDCODED MOTOR STEPS (200)**
**File:** `src/winding_controller.cpp`  
**Lines:** 172, 340, 402, 497, 574  
**Problem:** `200` appears 5+ times (steps per revolution)

**Impact:**
- If you use different motor (e.g., 400 steps/rev)
- Must edit many locations

**Fix:** Add to config.h:
```cpp
#define MOTOR_STEPS_PER_REV  200  // Standard NEMA motor
```

---

### **4. HARDCODED RAMP PARAMETERS**
**File:** `src/winding_controller.cpp`  
**Line:** 348  
**Problem:**
```cpp
const int N_slices = 24;  // Ramp interpolation steps
```

**Impact:**
- Fixed ramp smoothness
- Can't tune for smoother/faster ramps

**Fix:** Add to config.h:
```cpp
#define SPINDLE_RAMP_SLICES  24  // Number of ramp interpolation steps
```

Also line 350:
```cpp
const float sps_min = std::max(100.0f, target_sps * 0.02f);
```

**Fix:** Add to config.h:
```cpp
#define SPINDLE_MIN_RAMP_SPS  100.0f  // Minimum speed during ramp
#define SPINDLE_RAMP_START_PERCENT  0.02f  // Start at 2% of target
```

---

### **5. HARDCODED QUEUE TIME**
**File:** `src/winding_controller.cpp`  
**Lines:** 378, 409  
**Problem:**
```cpp
uint32_t spindle_steps = (uint32_t)(target_sps * 1.5f);  // 1.5 seconds
```

**Impact:**
- Fixed buffer size
- Can't tune for smoother/responsive control

**Fix:** Add to config.h:
```cpp
#define SPINDLE_QUEUE_TIME_SEC  1.5f  // Seconds of steps to queue ahead
```

---

### **6. HARDCODED RPM UPDATE RATE**
**File:** `src/winding_controller.cpp`  
**Line:** 527  
**Problem:**
```cpp
if (dt_us < 500000) return;  // 500ms = 0.5 seconds
```

**Impact:**
- RPM only updates every 0.5 seconds
- Could be too slow for fast changes
- Could be too fast and noisy

**Fix:** Add to config.h:
```cpp
#define ENCODER_RPM_UPDATE_US  500000  // Update RPM every 500ms
```

---

### **7. HARDCODED Z-HOMING SPEED**
**File:** `src/winding_controller.cpp`  
**Line:** 173  
**Problem:**
```cpp
float slow_sps = 200.0f;  // 200 steps/sec = slow rotation
```

**Impact:**
- Fixed Z-homing speed
- Too slow wastes time, too fast might miss Z

**Fix:** Add to config.h:
```cpp
#define SPINDLE_Z_HOMING_SPS  200.0f  // Steps/sec during Z-index search
```

---

### **8. HARDCODED SLEEP DELAYS (main.cpp)**
**File:** `main.cpp`  
**Lines:** Multiple (45, 47, 92, 109, 127, 132, 153, 162, etc.)  
**Problem:** Various `sleep_ms()` delays scattered throughout

**Examples:**
```cpp
sleep_ms(10000);  // Line 109 - USB serial wait
sleep_ms(5000);   // Line 127 - Version display
sleep_ms(3000);   // Line 92, 162 - UI delays
sleep_ms(1000);   // Line 132, 190, 275 - State transitions
sleep_ms(500);    // Line 153, 230, 236, 308
```

**Impact:**
- Hard to tune UI responsiveness
- Inconsistent timing

**Fix:** Add to config.h:
```cpp
// UI Timing (milliseconds)
#define UI_STARTUP_DELAY_MS    10000  // Wait for USB serial
#define UI_VERSION_DELAY_MS    5000   // Show version screen
#define UI_COUNTDOWN_DELAY_MS  3000   // "Starting in 3s"
#define UI_STATE_DELAY_MS      1000   // State transition feedback
#define UI_QUICK_DELAY_MS      500    // Quick feedback
```

---

### **9. HARDCODED FIFO CHECK THRESHOLD**
**File:** `src/winding_controller.cpp`  
**Line:** 398  
**Problem:**
```cpp
if (fifo_level < 2) {  // Refill when < 2/4 entries
```

**Impact:**
- Fixed buffering strategy
- Might need tuning for different speeds

**Fix:** Add to config.h:
```cpp
#define PIO_FIFO_REFILL_THRESHOLD  2  // Refill when < N entries (out of 4)
```

---

### **10. HARDCODED STABILIZATION DELAY**
**File:** `src/winding_controller.cpp`  
**Line:** 128  
**Problem:**
```cpp
sleep_ms(200);  // let the queue stabilize
```

**Impact:**
- Arbitrary delay
- Might be too long/short

**Fix:** Add to config.h:
```cpp
#define MOTION_STABILIZE_DELAY_MS  200  // Queue stabilization time
```

---

### **11. CONFLICTING MICROSTEPPING DEFINITIONS**
**File:** `src/config.h`  
**Lines:** 109-111  
**Problem:**
```cpp
#define SPINDLE_MICROSTEPS  4   // Spindle uses 4x
#define TRAVERSE_MICROSTEPS 16  // Traverse uses 16x
#define MOTOR_MICROSTEPS    16  // ⚠️ LEGACY - causes confusion!
```

**Impact:**
- `MOTOR_MICROSTEPS` still used in line 172 of winding_controller.cpp
- Creates bugs!

**Fix:** 
1. Remove `MOTOR_MICROSTEPS` entirely
2. Fix line 172 to use `SPINDLE_MICROSTEPS`

---

## 📋 COMPLETE REFACTORING CHECKLIST:

### **Priority 1: CRITICAL BUGS** 🚨
- [ ] Fix line 172: Change `MOTOR_MICROSTEPS` → `SPINDLE_MICROSTEPS`
- [ ] Remove `MOTOR_MICROSTEPS` from config.h
- [ ] Verify all microstepping references

### **Priority 2: CONSOLIDATE LIMITS** 🎯
- [ ] Add `SPINDLE_MAX_SPS` to config.h
- [ ] Replace 3× `max_sps = 50000.0f` with `SPINDLE_MAX_SPS`
- [ ] Add `MOTOR_STEPS_PER_REV` to config.h
- [ ] Replace all `200` with `MOTOR_STEPS_PER_REV`

### **Priority 3: MOTION PARAMETERS** ⚙️
- [ ] Add `SPINDLE_RAMP_SLICES` to config.h
- [ ] Add `SPINDLE_MIN_RAMP_SPS` to config.h
- [ ] Add `SPINDLE_RAMP_START_PERCENT` to config.h
- [ ] Add `SPINDLE_QUEUE_TIME_SEC` to config.h
- [ ] Add `SPINDLE_Z_HOMING_SPS` to config.h
- [ ] Add `PIO_FIFO_REFILL_THRESHOLD` to config.h
- [ ] Add `MOTION_STABILIZE_DELAY_MS` to config.h

### **Priority 4: TIMING/UI** ⏱️
- [ ] Add `ENCODER_RPM_UPDATE_US` to config.h
- [ ] Add UI timing constants to config.h
- [ ] Replace hardcoded sleep_ms() values

---

## 🎯 RECOMMENDED NEW config.h SECTION:

```cpp
// =============================================================================
// SPINDLE MOTION LIMITS (Performance Tuning)
// =============================================================================
#define MOTOR_STEPS_PER_REV     200      // Standard NEMA stepper
#define SPINDLE_MAX_SPS         50000.0f // Max steps/sec (tune for your motor!)

// Ramp-up tuning
#define SPINDLE_RAMP_SLICES     24       // Smoothness (more = smoother, slower)
#define SPINDLE_MIN_RAMP_SPS    100.0f   // Minimum speed at ramp start
#define SPINDLE_RAMP_START_PERCENT 0.02f // Start at 2% of target speed

// Z-Index homing
#define SPINDLE_Z_HOMING_SPS    200.0f   // Search speed (slower = more reliable)

// PIO buffering
#define SPINDLE_QUEUE_TIME_SEC  1.5f     // Seconds of motion to buffer
#define PIO_FIFO_REFILL_THRESHOLD 2      // Refill when < 2/4 entries used

// =============================================================================
// ENCODER/RPM TIMING
// =============================================================================
#define ENCODER_RPM_UPDATE_US   500000   // Update RPM every 500ms (0.5 sec)

// =============================================================================
// UI TIMING (Milliseconds)
// =============================================================================
#define UI_STARTUP_DELAY_MS     10000    // USB serial initialization
#define UI_VERSION_DELAY_MS     5000     // Version screen display time
#define UI_COUNTDOWN_DELAY_MS   3000     // Auto-start countdown
#define UI_STATE_DELAY_MS       1000     // State transition feedback
#define UI_QUICK_DELAY_MS       500      // Quick UI updates
#define MOTION_STABILIZE_DELAY_MS 200    // Motion queue stabilization
```

---

## 📊 IMPACT SUMMARY:

| Issue | Severity | Files Affected | Lines to Change |
|-------|----------|----------------|-----------------|
| Wrong microstepping in Z-homing | 🔴 CRITICAL | 1 | 1 |
| Duplicate max_sps | 🟡 HIGH | 1 | 3 |
| Hardcoded 200 steps | 🟡 HIGH | 1 | 5+ |
| Hardcoded ramp params | 🟢 MEDIUM | 1 | 3 |
| Hardcoded sleep delays | 🟢 MEDIUM | 1 | 10+ |
| Other constants | 🟢 LOW | 2 | 5+ |

**Total:** ~30 lines to refactor across 2 files

---

## 🚀 NEXT STEPS:

1. **Test v1.8.3 first!** (user is testing now)
2. **Fix CRITICAL bug** (line 172 microstepping)
3. **Consolidate all constants** to config.h
4. **Release v1.8.4** with clean, maintainable code

Let me know test results and I'll implement the refactoring! 🎯
