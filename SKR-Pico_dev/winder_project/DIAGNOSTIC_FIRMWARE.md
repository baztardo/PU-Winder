# Diagnostic Firmware - Full Trace Build

## Purpose
This firmware has extensive diagnostics to trace EXACTLY where the code execution stops when motors don't move.

## What You'll See on USB Serial (115200 baud)

### 1. **Scheduler ISR Startup**
```
Starting scheduler ISR at 100 us intervals...
Scheduler ISR started successfully!
```
⚠️ **If you DON'T see this:** Scheduler failed to start - hardware timer issue

### 2. **First ISR Ticks**
```
ISR tick 1
ISR tick 2
ISR tick 3
```
⚠️ **If you DON'T see this:** ISR is not running - critical failure

### 3. **Chunks Being Pushed**
```
PUSH axis=0 interval=5000 add=0 count=3200 (depth now=1)
```
⚠️ **If you DON'T see this:** Chunks aren't being queued - check winding controller

### 4. **Chunks Being Loaded in ISR**
```
Axis 0: Loaded chunk interval=5000 count=3200
```
⚠️ **If you DON'T see this:** ISR is not loading chunks from queue

### 5. **Step Pulses Executing**
```
STEP axis=0 pin=11 count=1
STEP axis=0 pin=11 count=2
STEP axis=0 pin=11 count=3
STEP axis=0 pin=11 count=4
STEP axis=0 pin=11 count=5
```
⚠️ **If you DON'T see this:** Step pulses not being generated

### 6. **Encoder Updates**
```
[ENC] pos=1440 rpm=298.4
```

### 7. **State Transitions**
```
Homing spindle: queuing 1 chunks
Move to start: queued 5 chunks for 640 steps
Refilled spindle: 8 chunks (depth was 3)
```

## Diagnostic Flow Chart

```
START
  ↓
Scheduler ISR starts? 
  NO → Hardware timer failure
  YES ↓
  ↓
ISR ticks printing?
  NO → ISR not running
  YES ↓
  ↓
Chunks being pushed?
  NO → Winding controller not queuing
  YES ↓
  ↓
Chunks being loaded?
  NO → Queue empty or ISR not checking
  YES ↓
  ↓
Step pulses executing?
  NO → Timing issue or GPIO problem
  YES ↓
  ↓
Motors moving?
  NO → Hardware issue (wiring, power, TMC)
  YES → SUCCESS!
```

## Expected Pin Activity (with Oscilloscope/Logic Analyzer)

### Spindle Motor (GPIO 11)
- **Frequency:** ~200 Hz during homing (5000us intervals)
- **Pulse Width:** 2μs high
- **Pattern:** Continuous pulses for 3200 steps

### Traverse Motor (GPIO 6)
- **Frequency:** Variable based on winding speed
- **Pulse Width:** 2μs high
- **Pattern:** Bursts of pulses during traverse moves

### Heartbeat LEDs
- **GPIO 20 (FAN3):** Toggle every 500ms (from ISR)
- **GPIO 18 (FAN2):** Toggle every 250ms (from main loop)

## Failure Analysis Table

| Symptom | What You See | What's Missing | Problem Area |
|---------|--------------|----------------|--------------|
| **Silent Failure** | Nothing on serial | All diagnostics | USB serial not connected OR firmware not running |
| **ISR Won't Start** | "Scheduler ISR started successfully!" | "ISR tick" messages | Hardware timer broken |
| **ISR Running, No Chunks** | ISR ticks | PUSH messages | Winding controller not queuing |
| **Chunks Not Loading** | PUSH messages | "Axis X: Loaded chunk" | Queue mechanism broken |
| **No Step Pulses** | "Loaded chunk" | "STEP axis=" | Timing calculation wrong |
| **Pulses But No Movement** | "STEP axis=" | Motor movement | Hardware (TMC2209, wiring, power) |

## Pin Configuration to Verify

```
SPINDLE_STEP_PIN    = 11  (GPIO 11)
SPINDLE_DIR_PIN     = 10  (GPIO 10)
SPINDLE_ENA_PIN     = 12  (GPIO 12) - Active LOW

TRAVERSE_STEP_PIN   = 6   (GPIO 6)
TRAVERSE_DIR_PIN    = 5   (GPIO 5)
TRAVERSE_ENA_PIN    = 7   (GPIO 7) - Active LOW

ENCODER_A_PIN       = 3   (GPIO 3)
ENCODER_B_PIN       = 4   (GPIO 4)
ENCODER_Z_PIN       = 25  (GPIO 25)

TMC_UART_TX_PIN     = 8   (GPIO 8)
TMC_UART_RX_PIN     = 9   (GPIO 9)

I2C_SDA_PIN         = 0   (GPIO 0)
I2C_SCL_PIN         = 1   (GPIO 1)
```

## How to Use This Firmware

1. **Upload winder_project.uf2**
2. **Connect USB serial at 115200 baud**
   - Windows: PuTTY, TeraTerm
   - Mac/Linux: `screen /dev/ttyACM0 115200`
3. **Power cycle the board**
4. **Watch the serial output**
5. **Report EXACTLY what you see and what's missing**

## Klipper-Style Debugging

This firmware is based on Klipper architecture:
- **Move queue** = command queue (push_chunk = queue_step in Klipper)
- **ISR scheduler** = itersolve.c timing engine
- **Step compression** = trapq.c trapezoidal motion
- **Axis handlers** = stepcompress.c step generation

The key difference: Klipper uses Linux host + MCU, we have everything on MCU.

## Next Steps Based on Output

### If you see "ISR tick" but no "PUSH"
→ home_spindle() isn't running - check state machine

### If you see "PUSH" but no "Loaded chunk"
→ Queue depth check or timing issue - add more debug

### If you see "Loaded chunk" but no "STEP"
→ Interval calculation wrong - check time_us_32()

### If you see "STEP" but motors don't move
→ 100% hardware: check power, wiring, TMC UART, enable pins

## Upload This Build

File: `winder_project.uf2`
Size: ~138 KB
Location: `/workspace/SKR-Pico_dev/winder_project/build/`

**CRITICAL:** Report the COMPLETE serial output from power-on to Z-index timeout. Every line matters for diagnosis.
