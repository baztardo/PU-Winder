# BLDC Speed Pulse Test - EP-0172 with LCD UI

Complete test suite for BLDC motor speed pulse input with LCD display and button control.

## Features

✅ **Speed Pulse ISR Handler** - Counts SC output pulses  
✅ **RPM Calculation** - Real-time RPM from pulse frequency  
✅ **LCD Display** - Visual feedback on 3.5" TFT screen  
✅ **Button Control** - Menu navigation with BTN1/BTN2  
✅ **4 Test Modes**:
   - Speed Pulse Monitor (30s continuous monitoring)
   - PWM Ramp with Feedback (0→100→0 over 60s)
   - Direction Change Test (toggle direction)
   - Brake Test (engage/disengage brake)

## Hardware Configuration

### Pins Used (EP-0172)
```
BLDC Control:
  GP10  → PWM Speed Control
  GP11  → Direction Control
  GP18  → Brake Control
  GP19  → Speed Pulse Input (SC output from motor)

Display & Input:
  GP2   → SPI CLK (display)
  GP3   → SPI DIN (display)
  GP5   → SPI CS (display)
  GP6   → SPI DC (display)
  GP7   → SPI RST (display)
  GP8   → I2C SDA (touch)
  GP9   → I2C SCL (touch)
  GP14  → BTN2
  GP15  → BTN1
```

### BLDC Motor Connection
```
Motor Controller:
  PWM Pin → GP10 (0-100% speed)
  DIR Pin → GP11 (Low=Forward, High=Reverse)
  BRK Pin → GP18 (Low=Free, High=Brake)
  Speed Pulse → GP19 (3 pulses per phase from SC output)

Note: Adjust speed pulse pulses_per_revolution if needed
```

## Building

### Prerequisites
```bash
# Install Pico SDK
cd ~
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init
export PICO_SDK_PATH=~/pico-sdk

# Install build tools
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
```

### Build Steps
```bash
# Create and enter build directory
mkdir bldc_test && cd bldc_test
cmake /path/to/source/files ..
make -j4

# Two versions are created:
# - bldc_pulse_test.uf2 (with LCD and button UI)
# - bldc_pulse_test_console.uf2 (console only, for comparison)
```

### Flash to Pico
```bash
# Hold BOOTSEL button and plug in USB
# Copy UF2 file to RPI-RP2 drive
cp bldc_pulse_test.uf2 /Volumes/RPI-RP2/
# (Linux: /media/user/RPI-RP2/)

# Board will reboot automatically
```

## Usage

### Main Menu
```
┌────────────────────────────────────┐
│ BLDC Speed Pulse Test - EP-0172    │
│ LCD Display & Button Control       │
├────────────────────────────────────┤
│ > 1. Speed Pulse Monitor (30s)     │
│   2. PWM Ramp with Feedback        │
│   3. Direction Change Test         │
│   4. Brake Test                    │
│   0. Exit                          │
│                                    │
│ BTN1: Select | BTN2: Start/Back    │
└────────────────────────────────────┘
```

### Navigation
- **BTN1** (GP15): Move cursor up in menu, toggle features in tests
- **BTN2** (GP14): Select menu item, start test, return to menu

### Test Descriptions

#### 1. Speed Pulse Monitor (30s)
- Monitors continuous speed pulse input
- Displays: Pulse count, Revolutions, RPM, Frequency (Hz)
- Updates every 500ms
- Good for basic verification that motor is spinning

#### 2. PWM Ramp with Feedback
- Automatically ramps motor speed 0→100%→0 over 60 seconds
- Shows real-time RPM feedback from speed pulse
- 30 seconds ramp up, 30 seconds ramp down
- Verifies speed control linearity

#### 3. Direction Change Test
- Runs at constant 60% speed
- Use BTN1 to toggle direction
- Press BTN2 to return to menu
- Tests forward/reverse operation

#### 4. Brake Test
- Spins motor at 70%, then apply brake
- Use BTN1 to toggle brake on/off
- Watch RPM drop when brake engaged
- Press BTN2 to exit

## Serial Output

All debug info also prints to USB serial:
```bash
# Monitor with:
picocom /dev/ttyACM0 -b 115200
# or
screen /dev/ttyACM0 115200
```

Output example:
```
[INIT] Initializing BLDC controller...
[BLDC] Initialized
[INIT] Initializing speed pulse handler...
[BLDC-PULSE] Initialized on GPIO 19
[BLDC-PULSE] Pulses per revolution: 6
[INIT] Initializing LCD display...
[LCD] Initialized (320x480)
[INIT] Initializing button control...
[BUTTONS] Initialized BTN1(GP15) and BTN2(GP14)
[INIT] All systems initialized!

[BTN2] Pressed
[MENU] Starting test 1
```

## Customization

### Change Pulses Per Revolution
```cpp
// In bldc_pulse_test_ui_main.cpp
// After speed_pulse.init():
speed_pulse.set_pulses_per_revolution(3);  // for 3-phase BLDC
```

### Adjust Motor Speed
Edit in tests:
```cpp
bldc.set_speed(750);  // 75% speed (0-1000)
```

### Change RPM Smoothing
```cpp
// In display functions:
float rpm = speed_pulse.get_smoothed_rpm(0.2f);  // 0.1-1.0
```

### Modify Test Duration
```cpp
// Change timeout in test functions:
while (test_running && (time_us_32() - start) < 60000000) {
    // 60000000 µs = 60 seconds
}
```

## Troubleshooting

### No Pulses Detected
- Verify motor is spinning (listen for sound)
- Check GP19 connection to motor SC output
- Verify pull-up is enabled (it is, in code)
- Use multimeter to verify 0-3.3V transitions on GP19

### RPM Reading Incorrect
- Verify `set_pulses_per_revolution()` matches your motor
- Check motor connector polarity (backward rotation?)
- Use serial output to verify edge count increments

### LCD Not Responding
- Check SPI connections (GP2, GP3, GP5, GP6, GP7)
- Verify display power (5V)
- Try console-only version first: `bldc_pulse_test_console.uf2`

### Buttons Not Working
- Check GP14 and GP15 connections
- Verify pull-ups are in place (they are, in code)
- Use serial output to see "[BTN1] Pressed" messages

### Motor Won't Start
- Check GP10, GP11, GP18 connections
- Verify direction pin state (GPIO output low)
- Check brake pin state (GPIO output low = not braked)
- Use multimeter to verify PWM on GP10 (should see ~5V square wave)

## File Structure

```
├── bldc_pulse_test_ui_main.cpp    Main program with UI
├── bldc_pulse_test_main.cpp       Console-only version
├── bldc_speed_pulse.h/.cpp        Speed pulse ISR handler
├── ep7172_lcd.h/.cpp              LCD display driver
├── button_control.h/.cpp          Button control handler
├── config.h                        Pin configuration
└── CMakeLists.txt                 Build config
```

## Performance Specs

| Parameter | Value |
|-----------|-------|
| PWM Frequency | 16 kHz |
| ISR Frequency (encoder/buttons) | ~10 kHz (from Pico scheduler) |
| Max Speed Pulse Rate | ~100 kHz (limited by Pico GPIO speed) |
| RPM Update Rate | Every 500ms (display) |
| Debounce Time | 100ms (buttons), 100µs (speed pulse) |
| Max Motor Speed Tested | 5000+ RPM |

## Future Enhancements

- [ ] Add tension control via load cell
- [ ] Store test data to Pico's Flash
- [ ] Multi-motor coordination
- [ ] Closed-loop speed control (PID)
- [ ] WiFi telemetry (Pico W)
- [ ] G-code interpreter for winding patterns
- [ ] Automatic layer detection
- [ ] Emergency stop button (GPIO 25)

## License

Public domain - Use freely for your projects

## Support

For issues, check:
1. Serial output (USB) - Shows initialization status
2. LCD display - Confirms display is working
3. Button presses - Should print "[BTN1/2] Pressed" to serial
4. Speed pulse - Verify edges on GP19 with oscilloscope if available

---

**Happy Motor Testing! 🎯**