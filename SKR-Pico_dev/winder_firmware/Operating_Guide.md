# LCD-Based Winding System - Testing & Operation Guide

## 🔌 Hardware Connections

### I2C LCD (20x4)
- **VCC** → 5V
- **GND** → Ground
- **SDA** → GPIO 0 (Pin 1)
- **SCL** → GPIO 1 (Pin 2)

Common I2C addresses: 0x27 or 0x3F (check your LCD module)

### Home Switch (Traverse)
- **Signal** → GPIO 14
- **GND** → Ground
- **Pull-up** enabled in firmware (active-low trigger)

### Encoder (Spindle)
- **A** → GPIO 4
- **B** → GPIO 3
- **Z** → GPIO 25
- **5V** → 5V
- **GND** → Ground

## 📺 LCD Display Sequence

### Startup (First 5 seconds)

```
┌────────────────────┐
│Wire Winder v1.0    │  Line 1: Title
│Initializing...     │  Line 2: Status
│Motors...           │  Line 3: Progress
│Scheduler...        │  Line 4: Progress
└────────────────────┘
```

### Parameters Display (2 seconds)

```
┌────────────────────┐
│Winding Setup:      │  Configuration
│Turns: 1000         │  Total turns
│Layers: 781         │  Calculated layers
│RPM: 300            │  Spindle speed
└────────────────────┘
```

### Ready Screen (3 seconds)

```
┌────────────────────┐
│System Ready        │  
│Auto-starting in 3s │  Countdown
│                    │  
│                    │  
└────────────────────┘
```

### Homing Spindle

```
┌────────────────────┐
│Homing Spindle...   │  Current operation
│Finding Z Index     │  Sub-task
│                    │  
│                    │  
└────────────────────┘

↓ When Z pulse detected ↓

┌────────────────────┐
│Homing Spindle...   │  
│Finding Z Index     │  
│Z Index Found!      │  Success message
│                    │  
└────────────────────┘
```

### Homing Traverse

```
┌────────────────────┐
│Homing Traverse...  │  Current operation
│Moving to switch    │  Moving phase
│                    │  
│                    │  
└────────────────────┘

↓ After switch triggered ↓

┌────────────────────┐
│Homing Traverse...  │  
│Switch found!       │  
│Backing off...      │  Back-off phase
│                    │  
└────────────────────┘

↓ After backoff complete ↓

┌────────────────────┐
│Homing Traverse...  │  
│Switch found!       │  
│Backing off...      │  
│Home complete!      │  Done
└────────────────────┘
```

### Moving to Start

```
┌────────────────────┐
│Moving to Start     │  Operation
│Target: 20.0mm      │  Destination
│                    │  
│                    │  
└────────────────────┘

↓ After reaching position ↓

┌────────────────────┐
│Moving to Start     │  
│Target: 20.0mm      │  
│In position!        │  Confirmation
│                    │  
└────────────────────┘
```

### Ramping Up

```
┌────────────────────┐
│Ramping Up...       │  Operation
│RPM: 125            │  Current RPM (increasing)
│Target: 300         │  Target RPM
│                    │  
└────────────────────┘

↓ Speed reached ↓

┌────────────────────┐
│Ramping Up...       │  
│RPM: 300            │  At target
│Target: 300         │  
│At speed!           │  Confirmation
└────────────────────┘
```

### Active Winding (Updates Every 100ms)

```
┌────────────────────┐
│Winding: 347/1000   │  Progress: current/total turns
│RPM: 300            │  Current spindle speed
│Layer: 1/2          │  Current layer / total layers
│L-Turns: 347/781    │  Turns in current layer
└────────────────────┘
```

### Ramping Down

```
┌────────────────────┐
│Ramping Down...     │  Operation
│RPM: 175            │  Decreasing speed
│                    │  
│                    │  
└────────────────────┘
```

### Completion

```
┌────────────────────┐
│Winding Complete!   │  Success
│Turns: 1000         │  Total turns completed
│Layers: 2           │  Layers wound
│                    │  
└────────────────────┘
```

## 🔧 Troubleshooting

### LCD Shows Nothing

**Problem**: Blank screen, backlight may be on

**Solutions**:
1. Check I2C address:
   ```cpp
   // Try 0x27, 0x3F, 0x20
   LCDDisplay lcd(i2c0, 0x27, 20, 4);  // Try different addresses
   ```

2. Check connections:
   - SDA → GPIO 0
   - SCL → GPIO 1
   - Power (5V and GND)

3. Check contrast adjustment (potentiometer on LCD backpack)

4. Test I2C scan:
   ```cpp
   for (uint8_t addr = 0x01; addr < 0x7F; addr++) {
       uint8_t rxdata;
       if (i2c_read_blocking(i2c0, addr, &rxdata, 1, false) >= 0) {
           // Found device - use LED or other indicator
       }
   }
   ```

### LCD Shows Garbage

**Problem**: Random characters or symbols

**Solutions**:
1. Lower I2C frequency in config.h:
   ```cpp
   #define I2C_FREQ_HZ 100000  // Try 100kHz instead of 400kHz
   ```

2. Add pull-up resistors (4.7kΩ) on SDA and SCL if not on LCD module

3. Check cable length (keep under 30cm for reliable operation)

### Spindle Won't Find Z Index

**Problem**: Stuck on "Finding Z Index"

**Solutions**:
1. Check encoder Z connection (GPIO 25)
2. Verify encoder power (5V)
3. Manually rotate spindle slowly - should trigger within one revolution
4. Check pull-up enabled on encoder pins
5. Test encoder with multimeter - should see voltage pulses

**Timeout**: Firmware will show "Z Index Timeout!" after 10 seconds

### Traverse Won't Home

**Problem**: Stuck on "Moving to switch"

**Solutions**:
1. Check home switch connection (GPIO 14)
2. Verify switch is normally-open, active-low
3. Check direction - should move towards physical switch
4. Test switch manually with multimeter
5. Verify pull-up enabled

### Winding Stops Immediately

**Problem**: Goes through sequence but stops at winding

**Solutions**:
1. Check encoder is counting (rotate spindle manually, watch encoder position)
2. Verify TMC2209 drivers initialized correctly
3. Check motor power supply voltage (12-24V)
4. Verify motor enable pins are LOW (motors enabled)
5. Check for error messages on LCD

### Wrong Number of Turns/Layers

**Problem**: Calculation seems incorrect

**Check parameters** in main.cpp:
```cpp
params.target_turns = 1000;          // Total turns wanted
params.wire_diameter_mm = 0.064f;    // Check wire diameter
params.layer_width_mm = 50.0f;       // Measure actual winding width
```

**Calculation**:
- Turns per layer = layer_width / wire_diameter
- Total layers = (target_turns / turns_per_layer) rounded up

Example for 43 AWG (0.064mm) wire, 50mm width:
- Turns per layer = 50 / 0.064 = 781.25 ≈ 781
- For 1000 turns = 1000 / 781 = 1.28 ≈ 2 layers

## ⚙️ Configuration Guide

### Changing Winding Parameters

Edit `main.cpp` in `setup_winding_parameters()`:

```cpp
void setup_winding_parameters() {
    WindingParams params;
    
    // *** MODIFY THESE VALUES ***
    params.target_turns = 2000;          // Change: total turns
    params.spindle_rpm = 250.0f;         // Change: slower/faster
    params.wire_diameter_mm = 0.080f;    // Change: different wire (AWG 40 = 0.080mm)
    params.layer_width_mm = 60.0f;       // Change: your bobbin width
    params.start_position_mm = 25.0f;    // Change: start position
    params.ramp_time_sec = 5.0f;         // Change: slower ramp = smoother
    
    params.calculate_layers();
    winding_controller.set_parameters(params);
}
```

### Wire Diameter Reference (AWG)

| AWG | Diameter (mm) | Diameter (inches) |
|-----|---------------|-------------------|
| 38  | 0.101         | 0.00398          |
| 39  | 0.089         | 0.00350          |
| 40  | 0.079         | 0.00311          |
| 41  | 0.071         | 0.00279          |
| 42  | 0.063         | 0.00248          |
| **43** | **0.056**  | **0.00220**      |
| 44  | 0.051         | 0.00200          |

Note: Insulation adds ~0.008mm to diameter

### Adjusting Speed

**Slower (for delicate wire):**
```cpp
params.spindle_rpm = 150.0f;    // Half speed
params.ramp_time_sec = 5.0f;    // Gentle ramp
```

**Faster (for production):**
```cpp
params.spindle_rpm = 500.0f;    // Faster
params.ramp_time_sec = 2.0f;    // Quick ramp
```

**Very slow (testing):**
```cpp
params.spindle_rpm = 60.0f;     // 1 rev/second
params.ramp_time_sec = 10.0f;   // Very gentle
```

## 📊 Monitoring During Operation

### Normal Operation Indicators

**RPM Stable**: ±5 RPM variation is normal
```
RPM: 298  ← Good
RPM: 305  ← Good
RPM: 290  ← Good
RPM: 150  ← Problem! Should be at 300
```

**Layer Transitions**: Should be smooth
```
Layer: 1/2
L-Turns: 780/781  ← Near end of layer
L-Turns: 781/781  ← Layer complete
Layer: 2/2        ← New layer started
L-Turns: 1/781    ← Counting in new layer
```

**Turn Counting**: Should increment steadily
```
Winding: 100/1000  (10%)
Winding: 250/1000  (25%)
Winding: 500/1000  (50%)
Winding: 750/1000  (75%)
Winding: 1000/1000 (100%)
```

### Warning Signs

**RPM Fluctuating Wildly:**
- Encoder connection issue
- Mechanical binding
- Power supply problem

**Turns Not Incrementing:**
- Encoder not counting
- Spindle not moving
- TMC2209 issue

**Layer Not Changing:**
- Traverse not moving
- Synchronization problem
- Queue underrun

## 🎯 Success Criteria

You know it's working correctly when:

1. ✅ LCD shows clear, readable text
2. ✅ Spindle homes to Z index successfully
3. ✅ Traverse homes and backs off smoothly
4. ✅ Move to start position completes
5. ✅ Spindle ramps up smoothly to target RPM
6. ✅ RPM reading is stable (±5 RPM)
7. ✅ Turns increment as spindle rotates
8. ✅ Traverse moves in sync with spindle
9. ✅ Layer transitions happen automatically
10. ✅ Direction reverses for each new layer
11. ✅ Winding completes with correct turn count
12. ✅ Spindle ramps down smoothly
13. ✅ Completion message displays

## 🚀 Next Steps After Successful Test

Once basic winding works:

1. **Add pause/resume functionality**
2. **Add emergency stop button**
3. **Add rotary encoder for menu navigation**
4. **Add parameter adjustment via menus**
5. **Add load cell for tension control**
6. **Save/load winding programs**
7. **Add wire break detection**
8. **Log winding statistics**

---