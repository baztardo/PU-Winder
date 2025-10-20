# ST7789 TFT + BLDC Motor Driver for SKR-Pico

## 🔌 Pin Wiring Diagram

### ST7789 TFT Display (SPI0)

```
SKR-Pico        ST7789 TFT
─────────────────────────────
GND      ───────→ GND (Black)
3.3V     ───────→ VCC (Red)

SPI0 Bus:
GPIO 18  ───────→ SCK  (Clock) - Orange
GPIO 19  ───────→ MOSI (Data)  - Yellow
GPIO 16  ───────→ MISO (optional, not used)

Control Pins:
GPIO 17  ───────→ CS   (Chip Select) - Green
GPIO 20  ───────→ DC   (Data/Command) - Blue
GPIO 21  ───────→ RST  (Reset) - Purple
```

### BLDC Motor & Hall Sensors

```
SKR-Pico        BLDC Motor Driver
─────────────────────────────────
GPIO 2   ───────→ PWM  (Speed control)
GPIO 26  ───────→ DIR  (Direction)
GPIO 27  ───────→ BRK  (Brake)

Hall Sensors (Input):
GPIO 3   ←───────  HALL_A (Input, 3.3V)
GPIO 4   ←───────  HALL_B (Input, 3.3V)
GPIO 5   ←───────  HALL_C (Input, 3.3V)

12V Supply:
12V Power → Motor power
GND      → Motor GND (common with Pico GND)
```

### Complete Pin Usage Summary

```
GPIO  0: I2C SDA (not used in this project)
GPIO  1: I2C SCL (not used in this project)
GPIO  2: BLDC PWM output          [Output]
GPIO  3: Hall A input              [Input]
GPIO  4: Hall B input              [Input]
GPIO  5: Hall C input              [Input]
GPIO 16: SPI MISO                  [SPI - optional]
GPIO 17: TFT CS                    [Output]
GPIO 18: SPI SCK                   [SPI]
GPIO 19: SPI MOSI                  [SPI]
GPIO 20: TFT DC                    [Output]
GPIO 21: TFT RST                   [Output]
GPIO 26: BLDC Direction            [Output]
GPIO 27: BLDC Brake                [Output]
```

## 📦 Hardware Requirements

- **Raspberry Pi Pico** (SKR-Pico compatible)
- **ST7789 TFT Display** (240×320 pixels, SPI interface)
  - Common modules: 2.4" displays with ST7789 chip
- **BLDC Motor** (with Hall sensors)
- **BLDC Motor Driver** (with PWM, direction, brake inputs)
  - Example: VESC, SimpleFOC, or custom 3-phase driver
- **12V Power Supply** (for motor)
- **Jumper Wires** and breadboard

## 🛠️ Building the Firmware

### Prerequisites

```bash
# Install build tools (if not already done)
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential

# Set Pico SDK path
export PICO_SDK_PATH=~/pico-sdk
```

### Build Steps

```bash
# 1. Create project directory
mkdir bldc_tft_project
cd bldc_tft_project

# 2. Copy all files
cp /path/to/bldc_tft_demo.cpp .
cp /path/to/st7789_pico.h .
cp /path/to/st7789_pico.cpp .
cp /path/to/CMakeLists.txt .
cp /path/to/pico_sdk_import.cmake .

# 3. Create build directory
mkdir build && cd build

# 4. Configure with CMake
cmake ..

# 5. Build the firmware
make -j$(nproc)

# Output file: bldc_tft_demo.uf2
```

### Flashing

```bash
# 1. Connect Pico with BOOTSEL held down
#    (or double-tap BOOTSEL button if already connected)

# 2. Copy UF2 file to RPI-RP2 drive
cp bldc_tft_demo.uf2 /media/$USER/RPI-RP2/
# or on macOS:
cp bldc_tft_demo.uf2 /Volumes/RPI-RP2/

# 3. Pico will reboot automatically and start firmware
```

## 🖥️ Serial Console

Connect to Pico's serial port to see debug output and send commands:

```bash
# Find the serial port
ls /dev/ttyACM* /dev/ttyUSB*

# Connect with picocom
picocom /dev/ttyACM0 -b 115200

# Or with screen
screen /dev/ttyACM0 115200

# Or with minicom
minicom -D /dev/ttyACM0 -b 115200
```

## 🎮 Controls

Once running, use serial console commands:

- **`s`** - Start spindle (ramp to 50% over 2 seconds)
- **`e`** - Stop spindle (ramp down over 2 seconds)
- **`d`** - Toggle direction (forward/reverse)
- **`b`** - Toggle brake (on/off)
- **`+`** - Increase speed by 5%
- **`-`** - Decrease speed by 5%
- **`q`** - Quit (shutdown motor)

### Example Session

```
$ picocom /dev/ttyACM0 -b 115200

BLDC Motor Test with ST7789 TFT Display
SKR-Pico + Pico SDK
============================================================

Initializing TFT display...
[ST7789] Initialized: 240 x 320
TFT initialized successfully
Initializing BLDC...
[BLDC] Initialized

System ready!
Controls:
  's' - Start spindle (ramp to 50%)
  'e' - Stop spindle
  'd' - Toggle direction
  'b' - Toggle brake
  '+' - Increase speed
  '-' - Decrease speed
  'q' - Quit

s           ← User types 's'
Starting spindle...
[BLDC] Set speed: 500/1000

d           ← User types 'd'
Direction: REVERSE

b
Brake: OFF

e
Stopping spindle...
```

## 📊 Display Output

The TFT screen shows:

```
╔════════════════════════╗
║ BLDC Test              │
║                        │
║ Speed: 50%             │  ← Current PWM duty
║ Dir: FWD               │  ← Direction (FWD/REV)
║ Brake: ON              │  ← Brake status (ON/OFF)
║ RPM: 1500              │  ← Calculated RPM
║                        │
║ [████████░░░░░░░░░░]   │  ← Speed bar
║                        │
║ Press 's' to start     │
║ Press 'e' to stop      │  ← Help text
║ Press 'd' to toggle    │
║ Press 'b' for brake    │
└════════════════════════┘
```

## 🔧 Troubleshooting

### TFT Display Not Showing Anything

**Check:**
1. SPI pins connected correctly (SCK, MOSI)
2. CS and DC pins set high/low correctly
3. Power supply (3.3V) to TFT
4. Reset pin pulsed during init
5. Check SPI frequency (try lowering to 30MHz if unstable):
   ```cpp
   tft.init(30000000);  // 30 MHz instead of 62.5 MHz
   ```

**Add Debug Output:**
```cpp
printf("SPI initialized\n");
printf("GPIO initialized\n");
printf("Sending reset...\n");
```

### Motor Not Responding

**Check:**
1. Motor power supply (12V) connected
2. PWM pin (GPIO 2) showing ~5kHz square wave
3. Direction pin voltage (0V or 3.3V)
4. Brake pin logic (0V = free, 3.3V = brake)
5. Motor driver connected properly

**Test with Logic Analyzer:**
```
Should see on GPIO 2:
├─ 0% duty: flat 0V
├─ 50% duty: square wave ~8µs high, ~8µs low
├─ 100% duty: flat 3.3V
```

### Hall Sensors Not Detected

**Check:**
1. Hall sensor power (3.3V)
2. Hall pins have pull-ups enabled (done in code)
3. Sensor logic levels (should be 0-3.3V TTL)
4. Rotation speed fast enough to see pulses

**Test with GPIO Read:**
```cpp
// Add to main loop:
printf("Hall A: %d  Hall B: %d  Hall C: %d\n",
       gpio_get(3), gpio_get(4), gpio_get(5));
```

Should see values change as motor spins.

### SPI Communication Issues

**Symptoms:** TFT shows garbage or no display

**Solutions:**
1. Check cable length (keep under 30cm for SPI)
2. Add pull-up resistors to CS/DC (10kΩ to 3.3V)
3. Lower SPI frequency:
   ```cpp
   tft.init(15000000);  // Try 15 MHz
   ```
4. Verify Pico GPIO voltage (should be 3.3V logic, not 5V)

## 📚 API Reference

### ST7789 Display Functions

```cpp
// Initialization
tft.init(62500000);              // Initialize at 62.5 MHz

// Drawing
tft.fill(ST7789_BLACK);          // Fill entire screen
tft.fill_rect(x1, y1, x2, y2, color);  // Filled rectangle
tft.draw_rect(x1, y1, x2, y2, color);  // Rectangle outline
tft.draw_pixel(x, y, color);     // Single pixel
tft.draw_hline(x, y, len, color); // Horizontal line
tft.draw_vline(x, y, len, color); // Vertical line
tft.draw_circle(x, y, r, color);  // Circle outline
tft.fill_circle(x, y, r, color);  // Filled circle

// Control
tft.display_on(true/false);      // Power on/off
tft.invert(true/false);           // Invert colors
tft.set_rotation(0-3);            // 0°, 90°, 180°, 270°

// Info
tft.get_width();                  // Display width
tft.get_height();                 // Display height
```

### BLDC Functions (in demo)

```cpp
bldc_init();                      // Initialize motor
bldc_set_speed(0-1000);          // Set PWM duty (0-100%)
bldc_set_direction(true/false);  // Forward/reverse
bldc_set_brake(true/false);      // Brake on/off
bldc_ramp_to(target, time_ms);   // Smooth ramp
bldc_update();                    // Call from main loop
```

## 🚀 Next Steps

### After Basic Testing Works:

1. **Add Hall Sensor Interrupt:**
   ```cpp
   // Replaces polling, counts true RPM
   void hall_isr(uint gpio, uint32_t events) {
       // Update RPM from Hall edges
   }
   ```

2. **Implement 6-Step Commutation:**
   - Electronic switching based on Hall state
   - Much smoother torque delivery

3. **Add Closed-Loop Speed Control:**
   - PID controller for consistent RPM
   - Automatic load compensation

4. **Integrate with Stepper Winder:**
   - Use BLDC for spindle instead of stepper
   - Real-time position feedback
   - Sync traverse to BLDC revolutions

## 📝 Notes

- **SPI Frequency:** 62.5 MHz is safe for most ST7789 modules
- **PWM Frequency:** 16 kHz is standard for motor drivers
- **Hall Sensors:** 3.3V TTL levels (Pico native)
- **Display Update:** Every 500ms to avoid flicker
- **Motor Ramping:** Smooth acceleration/deceleration in 50ms steps

## 📞 Support

Check these first:
1. Serial console output (what errors appear?)
2. GPIO voltage levels (multimeter)
3. SPI bus with logic analyzer
4. Motor driver documentation (specs for PWM freq, current)

---

**Ready to build!** Start with the build steps above. 🎯