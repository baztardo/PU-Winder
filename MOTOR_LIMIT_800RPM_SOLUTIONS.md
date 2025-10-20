# 🎯 MOTOR LIMIT: 800 RPM Spindle Solutions

## **TEST RESULTS (v2.1.1):**

```
✅ 1400 config → 700 RPM spindle → STABLE (even under load!)
✅ 1600 config → 801 RPM spindle → STABLE (even under load!)
❌ 1800 config → 795 RPM spindle → STALLS (motor hums, lost sync)

Motor physical limit: ~800 RPM spindle (1600 RPM stepper @ 10,667 sps)
```

**Diagnosis: Stepper motor at torque-speed curve limit!**

---

## **🔧 SOLUTION OPTIONS (Ranked by Effort):**

---

### **OPTION 1: TMC SpreadCycle Mode** ⚡ (QUICK WIN!)

**What it does:**
- Switches from StealthChop (quiet) to SpreadCycle (performance)
- Higher torque at high speeds
- Better motor response
- May gain 10-20% speed!

**Expected gain:**
```
Current: 800 RPM max
With SpreadCycle: 880-960 RPM (10-20% boost!)
```

**Effort:**
- Code changes: 4 hours
- Testing: 2 hours
- **Total: 6 hours** ✅

**Pros:**
- ✅ Quick to implement
- ✅ Already have code from Klipper review
- ✅ Free (no hardware needed)
- ✅ Worth trying first!

**Cons:**
- ⚠️ Motor will be noisier
- ⚠️ May only gain 100-150 RPM
- ⚠️ Still might not reach 1500 RPM target

**Recommendation:** **DO THIS FIRST!** Easy win! 🔥

---

### **OPTION 2: Reduce Microstepping** ⚡⚡ (MEDIUM WIN!)

**What it does:**
- Switch from 1/4 microstepping to 1/2 or full-step
- Double the torque at high speeds
- Half the resolution (but may be acceptable)

**Expected gain:**
```
Current: 1/4 step @ 800 RPM
1/2 step: ~1000-1200 RPM (25-50% boost!)
Full step: ~1200-1500 RPM (50-100% boost!)
```

**Effort:**
- Change config.h: `SPINDLE_MICROSTEPS 4` → `2` or `1`
- Recompile and test
- **Total: 30 minutes** ✅

**Pros:**
- ✅ Very quick to test
- ✅ Big torque gain
- ✅ May reach 1200+ RPM!

**Cons:**
- ⚠️ Lower resolution (less smooth)
- ⚠️ May not reach full 1500 RPM
- ⚠️ Won't work if smoothness critical

**Recommendation:** **Try after SpreadCycle!** Quick test! 🔥

---

### **OPTION 3: Increase Voltage** ⚡⚡⚡ (BIG WIN!)

**What it does:**
- Steppers perform better at higher voltage
- More current at high speeds = more torque
- Typical: 24V → 36V or 48V

**Expected gain:**
```
24V: 800 RPM current
36V: ~1000-1200 RPM (25-50% boost!)
48V: ~1200-1500 RPM (50-100% boost!)
```

**Effort:**
- Check current PSU voltage
- If 24V, swap to 36V or 48V PSU
- Verify TMC2209 can handle voltage (max 48V)
- **Total: 2-4 hours** ✅

**Pros:**
- ✅ Big performance gain
- ✅ May reach 1200-1500 RPM!
- ✅ Doesn't sacrifice smoothness

**Cons:**
- ⚠️ May need new PSU ($30-50)
- ⚠️ TMC2209 max voltage is 48V
- ⚠️ Check motor voltage rating

**Recommendation:** **If you have 24V PSU, this is a winner!** 🔥

---

### **OPTION 4: BLDC Motor Swap** 🚀🚀🚀 (GUARANTEED WIN!)

**What it does:**
- Replace stepper with BLDC motor
- 4-5k RPM capable!
- Use your existing EP-0172 BLDC controller
- Port existing BLDC test code

**Expected gain:**
```
Stepper: 800 RPM limit
BLDC: 3000-4000 RPM (375-500% boost!) 🚀🚀🚀
```

**Effort:**
- Mechanical mounting: 4 hours
- Port EP-0172 code: 8 hours
- Tuning/testing: 4 hours
- **Total: 16 hours** ⚠️

**Pros:**
- ✅ GUARANTEED 1500+ RPM!
- ✅ Can go 3000+ RPM if needed!
- ✅ More efficient
- ✅ Less heat
- ✅ You already have motor + controller!

**Cons:**
- ⚠️ Bigger time investment (16h)
- ⚠️ Need Hall sensor feedback (has it?)
- ⚠️ More complex control
- ⚠️ May need new mounting

**Recommendation:** **If other options fail, this is your ace!** 🚀

---

### **OPTION 5: Pi Zero + Klipper Architecture** 🤔 (OVERKILL?)

**What it does:**
- Move to Pi Zero running Klipper
- Pico becomes I/O controller
- Professional firmware architecture
- Better scheduling, multicore handling

**Expected gain:**
```
Performance: Same as Pico (doesn't fix motor limit!)
Architecture: Much better organization
```

**Effort:**
- Learn Klipper architecture: 8 hours
- Port code: 24 hours
- Testing: 8 hours
- **Total: 40+ hours** ❌

**Pros:**
- ✅ Professional architecture
- ✅ Better code organization
- ✅ More processing power

**Cons:**
- ❌ DOESN'T FIX MOTOR LIMIT!
- ❌ Huge time investment
- ❌ More complex
- ❌ More hardware to manage

**Recommendation:** **DON'T DO THIS!** Overkill, doesn't solve problem! ❌

---

## **🎯 RECOMMENDED PATH:**

### **IMMEDIATE (This Week):**

**1. Test TMC SpreadCycle (6 hours)** 🔥
```c
// In tmc2209.cpp, add:
tmc2209_write_reg(TMC_REG_GCONF, 0x00000004);  // Enable SpreadCycle
tmc2209_write_reg(TMC_REG_CHOPCONF, 0x000100C3); // Tuned chopper
tmc2209_write_reg(TMC_REG_IHOLD_IRUN, 0x00081F1F); // Max current
```
**Expected: 880-960 RPM spindle**

**2. Test 1/2 Microstepping (30 minutes)** 🔥
```c
// In config.h:
#define SPINDLE_MICROSTEPS 2  // Was 4
```
**Expected: 1000-1200 RPM spindle**

**3. Combine SpreadCycle + 1/2 Microstepping** 🔥🔥
```
Best case: 1100-1400 RPM spindle!
Worst case: 900-1000 RPM spindle
```

**If reaches 1200+ RPM:** ✅ **SUCCESS! Use hardware counter for turns!**

**If still < 1200 RPM:** → Move to BLDC!

---

### **IF STEPPER FAILS (Next Week):**

**BLDC Motor Swap (16 hours)** 🚀
```
Day 1: Mechanical mounting (4h)
Day 2: Port EP-0172 code (8h)
Day 3: Tuning and testing (4h)

Result: 3000-4000 RPM capable! 🚀🚀🚀
```

---

## **📊 EFFORT vs GAIN TABLE:**

| Solution | Effort | Expected Gain | Cost | Recommendation |
|----------|--------|---------------|------|----------------|
| **TMC SpreadCycle** | 6h | +100-150 RPM | $0 | ⭐⭐⭐ **DO THIS!** |
| **1/2 Microstepping** | 30m | +200-400 RPM | $0 | ⭐⭐⭐ **DO THIS!** |
| **36V PSU** | 2-4h | +200-400 RPM | $30-50 | ⭐⭐ If have 24V PSU |
| **BLDC Swap** | 16h | +2200-3200 RPM | $0 (have it!) | ⭐⭐⭐ **Plan B!** |
| **Pi Zero Klipper** | 40h+ | 0 RPM (doesn't fix!) | $15 | ❌ **DON'T DO!** |

---

## **🔥 IMMEDIATE ACTION PLAN:**

### **TODAY:**
```bash
1. Check PSU voltage (24V or 36V?)
2. Test 1/2 microstepping (30 min test!)
   - Change config.h: SPINDLE_MICROSTEPS 2
   - Recompile, test 1600 config
   - Does it hit 1000+ RPM?
```

### **THIS WEEKEND:**
```bash
1. Implement TMC SpreadCycle (6h)
2. Test with 1/4 and 1/2 microstepping
3. Find max stable RPM
4. If > 1200 RPM: Wire hardware counter, DONE! ✅
```

### **NEXT WEEK (if needed):**
```bash
1. Order 36V PSU if needed ($30-50)
2. If still < 1200 RPM: Plan BLDC swap (16h)
3. BLDC guarantees 1500+ RPM! 🚀
```

---

## **💎 HARDWARE COUNTER (Don't Forget!):**

**Once motor speed is solved:**
```
Wire hardware counter:
  - Encoder A/B → Counter inputs
  - Counter NPN → Pico GPIO 26
  - Set target turns on counter
  - Done! Accurate counting at ANY speed! ✅

This solves encoder freeze completely!
```

---

## **🎯 BOTTOM LINE:**

**Your stepper CAN reach 800 RPM (proven!)** ✅

**With optimization:**
- SpreadCycle + 1/2 step: ~1100-1400 RPM likely
- If that fails: BLDC guarantees 1500+ RPM! 🚀

**Test order:**
1. 1/2 microstepping (30 min) 🔥
2. TMC SpreadCycle (6h) 🔥
3. BLDC swap if needed (16h) 🚀

**You're SO close to 1500 RPM!** 🎯
