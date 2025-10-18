# MOTOR TEST - DEFINITIVE HARDWARE VERIFICATION

## Two Firmware Options

### ✅ **OPTION 1: motor_test.uf2 (START HERE!)**
- **Location:** `/workspace/motor_test.uf2`
- **Purpose:** Pure GPIO test - proves if hardware works
- **NO complex logic, NO queues, NO ISR**

### ✅ **OPTION 2: winder_project.uf2 (If Option 1 works)**
- **Location:** `/workspace/SKR-Pico_dev/winder_project/build/winder_project.uf2`
- **Purpose:** Full winder with diagnostics

---

## 📋 **TEST PROCEDURE:**

### **Step 1: Upload motor_test.uf2**
1. Hold BOOTSEL button on Pico
2. Plug in USB
3. Copy `/workspace/motor_test.uf2` to the drive
4. Connect USB serial at 115200 baud

### **Step 2: Observe**

You should see:
```
==================================
  MINIMAL MOTOR TEST
  NO QUEUE, NO ISR, PURE GPIO
==================================

Initializing pins...
Pins initialized.
Enable pins set to 0V (motors enabled)
Direction pins set HIGH (forward)

=== TEST #1 ===
SPINDLE: Sending 200 pulses at 10Hz...
  Spindle: 50/200 pulses
  Spindle: 100/200 pulses
  Spindle: 150/200 pulses
SPINDLE: Done! Motor should have turned 1 revolution.

TRAVERSE: Sending 200 pulses at 10Hz...
  Traverse: 50/200 pulses
  Traverse: 100/200 pulses
  Traverse: 150/200 pulses
TRAVERSE: Done! Motor should have turned 1 revolution.

Waiting 5 seconds before next test...
```

**This repeats FOREVER.**

---

## 🔍 **RESULTS INTERPRETATION:**

### ✅ **RESULT A: Motors Move Slowly**
- **Spindle turns 1 revolution every 20 seconds**
- **Traverse turns 1 revolution every 20 seconds**
- **Conclusion:** HARDWARE IS WORKING!
- **Next:** Upload `winder_project.uf2` and check diagnostics

### ❌ **RESULT B: Motors Don't Move**
- **No movement at all despite serial output**
- **Conclusion:** Hardware issue confirmed
- **Check:**
  1. Power supply voltage (12-24V for TMC2209)
  2. Motor wiring (4 wires per motor)
  3. Enable pin voltage (should be 0V when enabled)
  4. TMC2209 driver LEDs (should be on)
  5. GPIO pin assignments in code match your board

---

## 🔧 **Hardware Checklist**

If motors don't move with motor_test.uf2:

### **Power Supply:**
- [ ] 12-24V DC connected to TMC2209 boards
- [ ] Voltage stable under load
- [ ] Ground connected between Pico and TMC2209

### **Wiring:**
| Signal | Pico GPIO | TMC2209 Pin |
|--------|-----------|-------------|
| Spindle STEP | 11 | STEP |
| Spindle DIR | 10 | DIR |
| Spindle ENA | 12 | EN (active LOW) |
| Traverse STEP | 6 | STEP |
| Traverse DIR | 5 | DIR |
| Traverse ENA | 7 | EN (active LOW) |

### **TMC2209 Drivers:**
- [ ] Red LED on (power)
- [ ] Green LED behavior (stepping activity)
- [ ] UART TX/RX connected (GPIO 8/9)
- [ ] UART address jumpers set (Spindle=0x00, Traverse=0x02)

### **Motors:**
- [ ] 4-wire connection (A+, A-, B+, B-)
- [ ] No shorts between coils
- [ ] Correct coil pairs identified

---

## 📊 **Voltage Measurements**

With motor_test.uf2 running:

| Pin | Expected Voltage | What It Means |
|-----|------------------|---------------|
| GPIO 12 (SPINDLE_ENA) | 0V | Motor enabled (active LOW) |
| GPIO 7 (TRAVERSE_ENA) | 0V | Motor enabled (active LOW) |
| GPIO 11 (SPINDLE_STEP) | Pulses 0V/3.3V | Step signal |
| GPIO 6 (TRAVERSE_STEP) | Pulses 0V/3.3V | Step signal |
| GPIO 10 (SPINDLE_DIR) | 3.3V | Direction HIGH |
| GPIO 5 (TRAVERSE_DIR) | 3.3V | Direction HIGH |

---

## 🎯 **What This Proves**

### **If motor_test.uf2 makes motors move:**
→ Hardware is GOOD  
→ Problem is in winder_project logic/timing  
→ Upload winder_project.uf2 and check serial diagnostics

### **If motor_test.uf2 doesn't make motors move:**
→ Hardware issue exists  
→ Not a software problem  
→ Check wiring, power, TMC2209 config

---

## 🚀 **After Testing**

### **Scenario 1: Minimal test WORKS, winder_project FAILS**
→ The problem is in queue/ISR/timing
→ Report complete serial output from both firmwares
→ We'll fix the specific broken component

### **Scenario 2: Both tests FAIL**
→ Hardware issue confirmed
→ Focus on hardware troubleshooting
→ Likely: wiring, power, or TMC2209 config

---

## 📞 **What to Report**

**For motor_test.uf2:**
1. Do you see serial output? (Copy first 50 lines)
2. Do motors physically move?
3. How many revolutions before stopping (if any)?

**For winder_project.uf2:**
1. Copy complete serial output from power-on to timeout
2. Do motors move during "MOTOR TEST START" phase?
3. At what point does it fail?

---

## ⚡ **Quick Decision Tree**

```
Upload motor_test.uf2
    ↓
Serial output appears?
├─ NO → USB serial not connected or firmware not running
└─ YES → Continue
    ↓
Motors move during test?
├─ YES → Hardware OK! Problem is in winder_project
│        → Upload winder_project.uf2
│        → Report where diagnostics stop
│
└─ NO → Hardware issue
        → Check power supply
        → Check wiring
        → Measure enable pin voltages
        → Check TMC2209 LEDs
```

---

**Start with motor_test.uf2 - it will PROVE if your hardware actually works!**
