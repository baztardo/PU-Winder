# 🚀 HIGH SPEED OPTIMIZATION PLAN

## 📊 YOUR TEST RESULTS SUMMARY:

| Config | Actual Spindle | Stepper SPS | Encoder | Result |
|--------|----------------|-------------|---------|--------|
| 120 | 60 RPM | 800 | ✅ Perfect | No issues |
| 240 | 120 RPM | 1600 | ✅ Perfect | No issues |
| 480 | 240 RPM | 3200 | ⚠️ Good | OOM panic (v1.8.8 fixes!) |
| 840 | 420 RPM | 5600 | ❌ Fail | Encoder frozen |
| 1680 | 840 RPM | 11200 | ❌ Fail | Motor stalled, encoder frozen |

---

## 🎯 CURRENT LIMITS FOUND:

### ✅ WORKING RANGE:
- **60-240 RPM actual** (120-480 config)
- Encoder tracks correctly
- Turn counts accurate
- Motor smooth

### ⚠️ PROBLEMATIC RANGE:
- **300-420 RPM actual** (600-840 config)
- Motor runs but encoder freezes
- Tach confirms motor speed
- Turn counting fails

### ❌ FAILURE RANGE:
- **500+ RPM actual** (1000+ config)
- Motor stalls/recovers
- Encoder completely fails
- System unstable

---

## 🔧 OPTIMIZATION ROADMAP:

### PHASE 1: Fix Encoder at High Speed (300-420 RPM)

#### Option 1A: Reduce Debug Overhead
**Problem:** `printf()` every 500ms blocks ISR
```cpp
// In update_rpm(), change:
if (dt_us < 2000000) return;  // Was 500000 (500ms), now 2s
```
**Impact:** Reduces printf overhead by 75%

#### Option 1B: Faster ISR Rate
**Problem:** Encoder FIFO drains too slowly
```cpp
// config.h:
#define HEARTBEAT_US 25  // Was 50 (20kHz → 40kHz)
```
**Impact:** Drains FIFO 2× faster, prevents overflow

#### Option 1C: Optimize Encoder ISR
**Problem:** Too much work in ISR
```cpp
// Only drain FIFO, don't calculate RPM in ISR
while (!pio_sm_is_rx_fifo_empty()) {
    process_sample();  // Fast!
}
// Calculate RPM later (not in ISR)
```

---

### PHASE 2: TMC2209 Optimization (500+ RPM)

Your motor stalled at 500-600 RPM. To go faster:

#### Option 2A: Switch to SpreadCycle
**StealthChop** (current) = quiet but weak at high speed  
**SpreadCycle** = louder but strong at high speed
```cpp
// In init_motors():
tmc_spindle.writeRegister(TMC_REG_GCONF, 0x00000004);
```

#### Option 2B: Increase Motor Current
```cpp
// config.h:
#define SPINDLE_CURRENT_MA 3000  // Was 2800 (if motor rated for it!)
```

#### Option 2C: Tune Chopper Timing
```cpp
// Optimize CHOPCONF for high speed:
uint32_t chopconf = 
    (1 << 28) |  // TOFF = 1 (faster decay)
    (2 << 15) |  // HSTRT = 2
    (1 << 11) |  // HEND = 1
    (2 << 8)  |  // TBL = 2
    153;         // Base config
tmc_spindle.writeRegister(TMC_REG_CHOPCONF, chopconf);
```

#### Option 2D: Set SpreadCycle Transition Speed
```cpp
// Switch to SpreadCycle at lower speeds:
tmc_spindle.writeRegister(TMC_REG_TPWMTHRS, 500);
```

#### Option 2E: Enable CoolStep
```cpp
// Auto-adjust current based on load:
uint32_t coolconf = 0x00010000;
tmc_spindle.writeRegister(TMC_REG_COOLCONF, coolconf);
```

---

### PHASE 3: Advanced Optimizations

#### Option 3A: Multi-Core Architecture
```cpp
Core 0: Main logic, LCD, UI
Core 1: Encoder reading, PIO management

Benefits:
- Zero ISR jitter
- Encoder never misses counts
- Can handle MUCH higher speeds
```

#### Option 3B: DMA-Driven PIO
```cpp
// Auto-feed PIO from memory buffer
// Zero CPU overhead during motion
dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
channel_config_set_read_increment(&cfg, true);
channel_config_set_dreq(&cfg, pio_get_dreq(pio, sm, true));
```

#### Option 3C: Lower Microstepping
```cpp
// Trade resolution for speed:
#define SPINDLE_MICROSTEPS 2  // Was 4 (doubles max speed!)

Benefits:
- 2× max speed capability
- Still 400 steps/rev (plenty for smoothness)
```

---

## 🧪 IMMEDIATE TESTS FOR v1.8.8:

### Test 1: 480 Config (Should complete now!)
```cpp
#define WINDING_SPINDLE_RPM 480.0f
```
**Expected:**
- Encoder: ~240 RPM
- Turn count: 100 ±3
- **Ramp-down completes!** ✅ (no OOM!)

### Test 2: Push the Encoder Limit
```cpp
#define WINDING_SPINDLE_RPM 600.0f  // 300 actual
#define WINDING_SPINDLE_RPM 800.0f  // 400 actual
```
**Watch for:** When does encoder freeze?

---

## 📋 OPTIMIZATION PRIORITY:

1. **v1.8.8** - Fix OOM panic ✅
2. **Reduce printf overhead** - Test 600-800 config
3. **TMC2209 tuning** - Enable SpreadCycle for 500+ RPM
4. **Multi-core** - If needed for 1000+ RPM

---

## 🎯 YOUR GOALS:

**Target Average:** 1000 RPM spindle (500 actual with gear ratio)  
**Target Maximum:** 1500 RPM spindle (750 actual)

**Current Limit:** ~240 RPM actual (480 config) with encoder tracking  
**Motor Limit:** ~300-400 RPM actual before stalling

**Gap to close:** 2-3× speed increase needed!

**Next optimizations:**
1. Fix encoder (should get to 400-500 RPM)
2. Tune TMC2209 (should get to 600-800 RPM)
3. Consider 2× microstepping (doubles max speed!)

---

## FILE: winder_v1.8.8_FIX_RAMP_DOWN_OOM.uf2

git pull
Test at 480 config - should complete without crash! 🎯
