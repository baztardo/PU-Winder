╔════════════════════════════════════════════════════════════════════════════╗
║                     ST7789 TFT + BLDC MOTOR DRIVER                        ║
║                    Raspberry Pi Pico - Pico SDK                           ║
║                                                                            ║
║  START HERE! 👇                                                            ║
╚════════════════════════════════════════════════════════════════════════════╝

📦 WHAT YOU HAVE
════════════════════════════════════════════════════════════════════════════

✅ Complete ST7789 TFT Display Driver (Pico SDK native)
✅ BLDC Motor Controller with smooth ramping
✅ Demo application with real-time display updates
✅ Everything you need to get started!

🗂️ FILES YOU NEED
════════════════════════════════════════════════════════════════════════════

Core Implementation:
  • st7789_pico.h          ← Display driver header
  • st7789_pico.cpp        ← Display driver implementation
  • bldc_tft_demo.cpp      ← Main demo application
  • config_bldc_tft.h      ← Customizable pin configuration

Build & Documentation:
  • CMakeLists.txt         ← Build configuration
  • README.md              ← Overview and features
  • ST7789_BLDC_BUILD_GUIDE.md ← Detailed wiring & build instructions

⚡ 5-MINUTE QUICK START
════════════════════════════════════════════════════════════════════════════

1. WIRE IT UP (15 minutes)

   ST7789 TFT Display:
   ┌─────────────────────────────┐
   │ SKR-Pico  →  TFT Display   │
   ├─────────────────────────────┤
   │ GPIO 18   →  SCK (clock)    │ (orange)
   │ GPIO 19   →  MOSI (data)    │ (yellow)
   │ GPIO 17   →  CS (select)    │ (green)
   │ GPIO 20   →  DC (command)   │ (blue)
   │ GPIO 21   →  RST (reset)    │ (purple)
   │ 3.3V      →  VCC            │ (red)
   │ GND       →  GND            │ (black)
   └─────────────────────────────┘

   BLDC Motor:
   ┌─────────────────────────────┐
   │ GPIO 2    →  PWM (speed)    │
   │ GPIO 26   →  DIR (direction)│
   │ GPIO 27   →  BRK (brake)    │
   │ 12V Pwr   →  Motor +        │
   │ GND       →  Motor -        │
   └─────────────────────────────┘

2. BUILD IT (5 minutes)

   mkdir bldc_tft && cd bldc_tft
   cp /path/to/files/* .
   mkdir build && cd build
   cmake ..
   make -j$(nproc)

3. FLASH IT (2 minutes)

   Hold BOOTSEL on Pico + plug in USB
   cp bldc_tft_demo.uf2 /media/$USER/RPI-RP2/
   Pico reboots automatically

4. TEST IT (2 minutes)

   picocom /dev/ttyACM0 -b 115200
   
   Type: s  (start motor - ramps to 50%)
   See TFT display light up!
   
   Other commands:
   - e: Stop
   - d: Toggle direction
   - b: Toggle brake
   - +/−: Adjust speed

🎯 WHAT HAPPENS NEXT
════════════════════════════════════════════════════════════════════════════

1. TFT screen shows up with motor speed, direction, brake status
2. Serial console responds to keyboard commands
3. Motor starts ramping smoothly when you type 's'
4. Speed bar on display updates in real-time

✨ FEATURES
════════════════════════════════════════════════════════════════════════════

✓ Pure Pico SDK (no Arduino dependencies)
✓ 62.5 MHz SPI for crisp, fast display updates
✓ Smooth motor acceleration (50ms ramping)
✓ Professional-quality graphics API
✓ Full command interface over serial
✓ Customizable via config_bldc_tft.h

📖 DETAILED DOCUMENTATION
════════════════════════════════════════════════════════════════════════════

For complete wiring diagrams, troubleshooting, and API reference:

👉 READ: ST7789_BLDC_BUILD_GUIDE.md

This covers:
  • Full pin wiring with diagrams
  • Detailed build instructions
  • SPI bus explanation
  • Troubleshooting guide
  • API reference
  • Next steps for advanced features

🔧 CUSTOMIZATION
════════════════════════════════════════════════════════════════════════════

All pins, speeds, and parameters are in one file:

👉 EDIT: config_bldc_tft.h

Change pins? Update these #defines:
  #define BLDC_PWM_PIN    2      // Change to any GPIO
  #define TFT_CS_PIN      17     // Change to any GPIO
  #define BLDC_DIR_PIN    26     // etc...

Change colors? Update these:
  #define COLOR_RED       0xF800
  #define COLOR_GREEN     0x07E0
  #define COLOR_BLUE      0x001F

🆘 NOT WORKING? FIRST CHECK
════════════════════════════════════════════════════════════════════════════

Display Shows Nothing?
  1. Power: 3.3V on TFT VCC pin?
  2. Pins: GPIO 18, 19, 20, 21, 17 connected?
  3. SPI: Try lower frequency in init:
     tft.init(30000000);  // 30 MHz instead of 62.5 MHz

Motor Not Responding?
  1. GPIO 2: Do you see PWM signal with scope/LED?
  2. Power: 12V supply for motor?
  3. Pins: DIR (GPIO 26) and BRK (GPIO 27) connected?

Garbled Display?
  1. Cable length: Keep SPI wires <30cm
  2. Pull-ups: Add 10kΩ resistors on CS and DC pins
  3. Frequency: Lower SPI speed to 30 MHz

🚀 NEXT STEPS (After Basic Testing Works)
════════════════════════════════════════════════════════════════════════════

1. Add Real RPM Reading
   - Implement Hall sensor ISR
   - Count pulses for accurate RPM

2. Add 6-Step Commutation
   - Electronic switching based on Hall state
   - Much smoother motor operation

3. Add Closed-Loop Speed Control
   - PID controller maintains RPM under load
   - Automatic acceleration compensation

4. Integrate with Your Wire Winder
   - Use BLDC for spindle instead of stepper
   - Connect traverse stepper
   - Synchronized winding!

💡 PRO TIPS
════════════════════════════════════════════════════════════════════════════

✓ SPI works best at 62.5 MHz but 30 MHz is more stable if you have issues
✓ Motor ramping every 50ms feels smooth and professional
✓ Keep the demo simple - add features gradually
✓ Check serial output for error messages (they help!)
✓ Use multimeter to verify GPIO voltage levels (should be 0-3.3V)

📞 SUPPORT
════════════════════════════════════════════════════════════════════════════

Check these first:
  1. Serial console output - error messages are here
  2. Wiring diagram in ST7789_BLDC_BUILD_GUIDE.md
  3. config_bldc_tft.h for pin definitions
  4. README.md for API reference

═══════════════════════════════════════════════════════════════════════════════

🎯 READY TO BUILD?

1. Follow wiring in this document
2. Run the build commands above
3. Flash to Pico (hold BOOTSEL)
4. Connect serial console
5. Type 's' to start motor

THAT'S IT! 🎉

═══════════════════════════════════════════════════════════════════════════════