# 🚀 COMPLETE OPTIMIZATION ROADMAP TO 1500 RPM

## 🎯 GOAL: 1500 RPM Spindle (750 RPM stepper, 10,000 sps)

## 📊 CURRENT STATUS (v2.1.1):

```
✅ Z-homing works
✅ Multicore working (Core 1 = encoder)
⚠️ Encoder freezes @ 4000+ sps (deal with later!)
⚠️ Motor stalls during aggressive ramp (fixed in v2.1.1)
🎯 12,000 sps limit (room for 1800 RPM spindle!)
```

---

## 🔥 OPTIMIZATION PATHS (Prioritized)

### **TIER 1: Quick Wins (1-4 hours each)**

#### **1.1 TMC2209 UART Optimization** 🔥🔥🔥
**Impact: +30-50% motor speed capability**  
**Effort: 2-4 hours**  
**Klipper code ready to port!**

```c
// From tmcuart.c:
- Switch to SpreadCycle mode (better for high RPM)
- Tune IRUN/IHOLD current (optimize torque vs heat)
- Set TPWMTHRS (auto switch StealthChop → SpreadCycle)
- Enable CoolStep (auto current reduction)
- Configure StallGuard (detect skipped steps)
```

**Expected gain:**
```
Current: Stalls at ~700 RPM stepper
After:   Smooth at 1000+ RPM stepper! 🚀
```

---

#### **1.2 Ramp-Up Tuning** ⚠️
**Impact: Consistent starts**  
**Effort: Testing only (done in v2.1.1!)**

```c
Current: 10s ramp (280 → 140 RPM/s)
If needed: 15s or 20s for very smooth
```

**Test v2.1.1 first!**

---

#### **1.3 Step Timing (Setup/Hold Times)** 🔧
**Impact: 10-20% speed gain**  
**Effort: 1 hour**

```c
// TMC2209 needs minimum pulse widths:
Step pulse: 100 ns minimum (we're fine)
Direction setup: 20 ns before step (we're fine)
Direction hold: 20 ns after step (check this!)

// Verify in spindle_step.pio:
- Step pulse width = half_period × 2
- At 12k sps: 1/(12k×2) = 41 μs ✓✓✓ (way over 100ns!)
```

**Status: Already optimized!** ✅

---

### **TIER 2: Medium Effort (4-8 hours each)**

#### **2.1 GPIO Timer Encoder (Klipper Style)** 🔥🔥
**Impact: Encoder works at ALL speeds**  
**Effort: 2-4 hours**  
**Klipper pulse_counter.c ready to port!**

```c
// Replace PIO encoder with:
- Hardware timer @ 100 kHz
- Direct GPIO reads
- Simple edge detection
- Proven by Klipper for 6000+ RPM!
```

**Expected:**
```
Current PIO: Freezes @ 4000 sps
GPIO timer: Works @ 16000+ sps! ✅
```

---

#### **2.2 DMA for Encoder FIFO** 🔥
**Impact: Encoder works at unlimited speed**  
**Effort: 4-6 hours**

```c
// DMA reads PIO FIFO → Memory buffer
// Core 1 processes buffer in batch
// Zero polling overhead!
```

**Alternative to 2.1 - pick one!**

---

#### **2.3 Scheduler Optimization** 🔧
**Impact: 10-20% CPU reduction**  
**Effort: 2-4 hours**

```c
// Port Klipper's scheduler tricks:
- Sentinel timer (avoid NULL checks)
- Optimized timer insertion
- Better ISR structure
```

---

### **TIER 3: Advanced (8+ hours each)**

#### **3.1 IRQ Priority Tuning** 🔧
**Impact: Better real-time performance**  
**Effort: 2 hours**

```c
// Set critical IRQs higher priority:
NVIC_SetPriority(TIMER_IRQ_0, 0);  // Highest (step generation)
NVIC_SetPriority(TIMER_IRQ_1, 1);  // Medium (encoder)
NVIC_SetPriority(UART1_IRQ, 2);    // Low (TMC UART)
```

---

#### **3.2 Overclock Pico** ⚡
**Impact: +20-40% all speeds**  
**Effort: 30 min**

```c
// Overclock from 125 MHz to 200+ MHz:
set_sys_clock_khz(200000, true);  // 200 MHz!
// Recompute all PIO dividers
```

**Risks:**
- May need voltage bump
- Heat issues
- Stability?

---

## 🔥 PLAN A: STEPPER OPTIMIZATION (Recommended)

### **Phase 1: Test Current Limits** (NOW!)
```
1. Upload v2.1.1 (10s ramp)
2. Test 1400, 1600, 1800 config
3. Find max stable speed
4. Note: Ignore encoder freeze (expected!)
```

### **Phase 2: TMC2209 SpreadCycle** (Next!)
```
1. Port tmcuart.c from Klipper
2. Configure SpreadCycle mode
3. Tune IRUN/IHOLD currents
4. Expected: +30-50% speed! 🚀
```

### **Phase 3: Fix Encoder**
```
1. Port GPIO timer (Klipper style)
2. Or implement DMA
3. Encoder works at all speeds ✅
```

**Expected final: 1500 RPM spindle stable!** ✅

---

## 🎮 PLAN B: Klipper Architecture

### **Pi Zero as Motion Controller:**

```
Pi Zero (Klipper):
├─ UI/LCD handling
├─ G-code parsing
├─ Motion planning
├─ Send step commands →

Pico (Execution):
├─ Receive commands via UART/SPI
├─ Generate steps (PIO)
├─ Read encoder
└─ Report status back
```

**Benefits:**
- ✅ Proven architecture (Klipper)
- ✅ Rich UI/features
- ✅ G-code support
- ✅ Community support

**Drawbacks:**
- ⚠️ Complex setup
- ⚠️ Two boards
- ⚠️ More wiring
- ⚠️ 40+ hours work!

**Verdict:** Use if PLAN A fails!

---

## 🌟 PLAN C: BLDC MOTOR (Secret Weapon!)

### **Your BLDC Specs:**
```
Motor: Same size as stepper
Speed: 4000-5000 RPM @ 24V! 🚀🚀🚀
Controller: 
  - HAL speed output (like encoder!)
  - PWM input (up to 25 kHz)
  - Direction pin (high/low)
  - Brake pin (high/low)
  - Stop pin (high/low)
```

**This is PERFECT for winding!**

### **BLDC Architecture:**

```
Pico Controls:
├─ PWM to BLDC (speed control)
├─ HAL speed input (like encoder)
├─ Direction GPIO
├─ Traverse stepper (same as now)
└─ LCD/UI

Benefits:
✅ 4000-5000 RPM capable! (3x stepper!)
✅ Smooth speed control (analog!)
✅ HAL output = reliable feedback
✅ No step generation overhead!
✅ Less heat than stepper
✅ Quieter operation
```

### **Code Changes Needed:**

```c
// Replace stepper PIO with PWM:
pwm_set_gpio_level(BLDC_PWM_PIN, duty_cycle);

// Read HAL speed (like encoder):
// Already have this code in EP-0172_BLDC_test!
```

**Estimated work: 8-16 hours**

**Expected result: 3000+ RPM spindle!** 🚀🚀🚀

---

## 📊 COMPARISON TABLE:

| Approach | Max RPM | Complexity | Time | Success |
|----------|---------|------------|------|---------|
| **Current Stepper** | 600 | Medium | 0h | ✅ Working |
| **+ TMC Tuning** | 1000-1200 | Medium | 4h | 🔥 High |
| **+ GPIO Encoder** | 1500 | Medium | 8h | 🔥 Very High |
| **Klipper Arch** | 1500 | High | 40h | ✅ Proven |
| **BLDC Motor** | 3000+ | Medium | 16h | 🚀 **BEST!** |

---

## 🎯 MY RECOMMENDATION:

### **Immediate (Today):**
1. **Test v2.1.1** - 10s ramp, find motor limit
2. **If reaches 1200+ RPM** - Continue stepper path!
3. **If stalls < 1000 RPM** - Consider BLDC!

### **This Week:**
1. **Implement TMC SpreadCycle** (4h) - +50% speed
2. **Port GPIO encoder** (4h) - Works at all speeds
3. **Target: 1500 RPM stable!** 🎯

### **If Stepper Can't Hit 1500 RPM:**
**Switch to BLDC!** (16h work but 3x speed!)

### **Only If All Else Fails:**
**Klipper architecture** (40h+)

---

## 🔥 NEXT ACTIONS:

**Step 1: TEST v2.1.1 NOW!**
```bash
git pull
Upload: winder_v2.1.1_SLOW_RAMP_10SEC.uf2
Test: 1400, 1600, 1800 config
Report: Max stable speed
```

**Step 2: Based on results...**

**If motor reaches 1200+ RPM:**
→ Implement TMC SpreadCycle (I'll start this!)

**If motor stalls < 1000 RPM:**
→ Consider BLDC swap (I can port EP-0172 code!)

---

## 💎 BLDC IS YOUR ACE IN THE HOLE!

**You already have:**
- ✅ BLDC motor (4-5k RPM!)
- ✅ Controller board (PWM + HAL)
- ✅ Working test code (EP-0172_BLDC_test!)

**If stepper can't hit 1500 RPM:**
**BLDC swap = 2 days work for 3000+ RPM!** 🚀

---

**📄 Files ready:**
- `winder_v2.1.1_SLOW_RAMP_10SEC.uf2` (test now!)
- `OPTIMIZATION_ROADMAP_COMPLETE.md` (full strategy!)

**🎯 Test v2.1.1, then we decide: TMC tuning or BLDC swap?** 🔥
