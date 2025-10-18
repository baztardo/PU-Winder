# Klipper-Style GPIO Stepper Refactor Plan

## 🎯 Goal:
Replace PIO-based stepping with proven **GPIO + Timer ISR** approach from Klipper

---

## ✅ What We Keep (It Works!):
1. **MoveQueue** - Queue structure is solid
2. **StepCompressor** - Generates correct `interval`, `count`, `add` values
3. **Encoder (PIO)** - Working perfectly for A/B channels
4. **TMC2209 UART** - Driver communication works
5. **LCD** - Display is functional
6. **WindingController** - State machine logic is good
7. **Overall architecture** - Well-designed!

---

## 🔄 What We Replace (PIO Issues):
1. **`spindle_step.pio`** → Remove entirely
2. **`spindle_step_pio.cpp/h`** → Replace with `gpio_stepper.cpp/h`
3. **PIO-based stepping** → GPIO toggle in timer ISR

---

## 📋 Implementation Steps:

### Step 1: Create GPIO Stepper Module (Klipper-style)
**File:** `SKR-Pico_dev/winder_project/src/gpio_stepper.cpp/h`

```cpp
// gpio_stepper.h
struct gpio_stepper {
    uint32_t interval;      // Current step interval (us)
    int16_t add;            // Acceleration delta
    uint32_t count;         // Remaining steps
    uint32_t next_step_time;
    uint step_pin;
    uint dir_pin;
    bool step_state;
};

void gpio_stepper_init(gpio_stepper* s, uint step_pin, uint dir_pin);
void gpio_stepper_event(gpio_stepper* s);  // Called from timer ISR
void gpio_stepper_queue_move(gpio_stepper* s, uint32_t interval, 
                              uint16_t count, int16_t add);
```

### Step 2: Modify Scheduler
**File:** `src/scheduler.cpp`

Add stepper event to hardware timer ISR:
```cpp
void scheduler_isr() {
    // Existing encoder, LCD updates...
    
    // NEW: Call stepper event
    if (spindle_stepper.count > 0) {
        gpio_stepper_event(&spindle_stepper);
    }
    if (traverse_stepper.count > 0) {
        gpio_stepper_event(&traverse_stepper);
    }
}
```

### Step 3: Update WindingController
**File:** `src/winding_controller.cpp`

Replace:
```cpp
::spindle_step_pio_queue_cv(&spindle_step_pio, steps, sps);
```

With:
```cpp
auto chunks = StepCompressor::compress_constant_velocity(steps, sps);
for (const auto& chunk : chunks) {
    gpio_stepper_queue_move(&spindle_stepper, 
                            chunk.interval, 
                            chunk.count, 
                            chunk.add);
}
```

### Step 4: Remove PIO Dependencies
- Remove `src/spindle_step.pio`
- Remove `src/spindle_step_pio.cpp/h`
- Remove `pico_generate_pio_header(spindle_step)` from `CMakeLists.txt`
- Keep encoder PIO (it works!)

---

## 📊 Expected Results:

### ✅ Advantages of GPIO Method:
1. **Proven by Klipper** - Runs millions of 3D printers
2. **Simpler debugging** - No PIO black box
3. **Direct hardware control** - No muxing issues
4. **Timing is precise** - Hardware timer + GPIO toggle
5. **Both axes use same method** - Consistent behavior

### ⚠️ Considerations:
1. **ISR timing critical** - Must complete quickly (<10µs)
2. **Interrupt priority** - Stepper ISR must be high priority
3. **Step pulse width** - Ensure minimum 2µs pulse width

---

## 🧪 Testing Plan:

### Phase 1: Bare-Bones Test
1. Create minimal GPIO stepper test
2. Generate 1000 steps at 1000 sps
3. Measure with oscilloscope/LED

### Phase 2: Integration
1. Replace spindle PIO with GPIO
2. Keep traverse on move_queue (already GPIO)
3. Test Z-homing

### Phase 3: Full System
1. Both axes on GPIO stepper
2. Run full winding cycle
3. Validate synchronization

---

## 🕐 Time Estimate:
- **Quick PIO fix attempt:** 10 minutes (try v1.2.1 first!)
- **Full GPIO refactor:** 2-3 hours (if PIO fix fails)
- **Testing & debug:** 1-2 hours
- **Total:** Half day maximum

---

## 💭 Recommendation:

### Option A: Try v1.2.1 FIRST (5 minutes)
The sideset base fix might solve the PIO issue. Upload `winder_v1.2.1_SIDESET_FIX.uf2` and test.

**If it works:** Great! Stick with PIO (more efficient)
**If it fails:** Proceed to Option B

### Option B: GPIO Refactor (3-4 hours)
Full Klipper-style implementation. Guaranteed to work, but more work.

---

## 🎯 My Vote:
**Try Option A (v1.2.1) right now!**

The missing `sm_config_set_sideset_base()` call is a likely culprit. If GPIO11 wasn't properly muxed to the PIO sideset, the pulses would go nowhere!

**Upload v1.2.1 and report back in 2 minutes!** 🚀

If that doesn't work, we'll do the GPIO refactor on this new branch.
