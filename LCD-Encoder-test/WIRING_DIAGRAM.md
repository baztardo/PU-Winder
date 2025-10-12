# Wiring Diagram

## Complete System Wiring

```
┌─────────────────────────────────────────────────────────────────┐
│                         SKR-PICO BOARD                          │
│                       (RP2040 Processor)                        │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                    I2C Bus (i2c0)                        │  │
│  │                                                          │  │
│  │  GPIO 0 (SDA) ●─────────────────────────────────┐      │  │
│  │  GPIO 1 (SCL) ●──────────────────────────┐      │      │  │
│  │                                          │      │      │  │
│  └──────────────────────────────────────────┼──────┼──────┘  │
│                                             │      │         │
│  ┌──────────────────────────────────────────┼──────┼──────┐  │
│  │                 Encoder Pins             │      │      │  │
│  │                                          │      │      │  │
│  │  GPIO 4  (A) ●                           │      │      │  │
│  │  GPIO 3  (B) ●                           │      │      │  │
│  │  GPIO 25 (Z) ●                           │      │      │  │
│  │                                          │      │      │  │
│  └──────────────────────────────────────────┼──────┼──────┘  │
│                                             │      │         │
│  ┌──────────────────────────────────────────┼──────┼──────┐  │
│  │                  Power                   │      │      │  │
│  │                                          │      │      │  │
│  │  5V  ●───────────────────────────────────┼──────┼──┐   │  │
│  │  GND ●───────────────────────────────────┼──────┼──┼┐  │  │
│  │                                          │      │  ││  │  │
│  └──────────────────────────────────────────┼──────┼──┼┼──┘  │
│                                             │      │  ││     │
└─────────────────────────────────────────────┼──────┼──┼┼─────┘
                                              │      │  ││
                                              │      │  ││
         ┌────────────────────────────────────┘      │  ││
         │                  ┌────────────────────────┘  ││
         │                  │             ┌─────────────┘│
         ↓                  ↓             ↓              ↓
    ┌────────────────────────────────────────────────────┐
    │         TC2004A-01 LCD (20x4 Display)              │
    │              with PCF8574 I2C Backpack             │
    ├────────────────────────────────────────────────────┤
    │ Pin 1  VSS  (GND)      ←───────────────────────────┼── GND
    │ Pin 2  VDD  (5V)       ←───────────────────────────┼── 5V
    │ Pin 3  V0   (Contrast) [Connected to pot on board] │
    │ Pin 4  RS              [Controlled by PCF8574]     │
    │ Pin 5  R/W             [Controlled by PCF8574]     │
    │ Pin 6  E               [Controlled by PCF8574]     │
    │ Pin 7  DB0             [Not used in 4-bit mode]    │
    │ Pin 8  DB1             [Not used in 4-bit mode]    │
    │ Pin 9  DB2             [Not used in 4-bit mode]    │
    │ Pin 10 DB3             [Not used in 4-bit mode]    │
    │ Pin 11 DB4             [Controlled by PCF8574]     │
    │ Pin 12 DB5             [Controlled by PCF8574]     │
    │ Pin 13 DB6             [Controlled by PCF8574]     │
    │ Pin 14 DB7             [Controlled by PCF8574]     │
    │ Pin 15 LED+ (Backlight)←───────────────────────────┼── 5V
    │ Pin 16 LED- (Backlight)←───────────────────────────┼── GND
    │                                                     │
    │ ┌─────────────────────────────────────────────┐   │
    │ │    PCF8574 I2C Backpack (on back of LCD)    │   │
    │ │                                             │   │
    │ │  SDA ←──────────────────────────────────────┼───┼── GPIO 0
    │ │  SCL ←──────────────────────────────────────┼───┼── GPIO 1
    │ │  VCC ←──────────────────────────────────────┼───┼── 5V
    │ │  GND ←──────────────────────────────────────┼───┼── GND
    │ │                                             │   │
    │ │  I2C Address: 0x27 (or 0x3F)                │   │
    │ │  Adjustable via A0, A1, A2 jumpers          │   │
    │ └─────────────────────────────────────────────┘   │
    └────────────────────────────────────────────────────┘


         ┌────────────────────────────────────┐
         │  360 PPR Rotary Encoder            │
         │  (with A, B, Z channels)           │
         ├────────────────────────────────────┤
         │  VCC (5V or 3.3V)  ←───────────────┼── 5V or 3.3V *
         │  GND               ←───────────────┼── GND
         │  A (Channel A)     ────────────────┼── GPIO 4
         │  B (Channel B)     ────────────────┼── GPIO 3
         │  Z (Index)         ────────────────┼── GPIO 25
         └────────────────────────────────────┘

    * Note: If encoder is 5V, you may need level shifters for A/B/Z signals
            to protect the 3.3V GPIO pins on the SKR-Pico


═══════════════════════════════════════════════════════════════════
                      CONNECTION SUMMARY
═══════════════════════════════════════════════════════════════════

SKR-Pico          TC2004A-01 LCD              Function
────────────────────────────────────────────────────────────────────
GPIO 0 (I2C SDA)  → PCF8574 SDA              I2C Data
GPIO 1 (I2C SCL)  → PCF8574 SCL              I2C Clock
5V                → LCD VDD, LED+, PCF8574   Power
GND               → LCD VSS, LED-, PCF8574   Ground


SKR-Pico          Encoder                    Function
────────────────────────────────────────────────────────────────────
GPIO 4            → Channel A                Quadrature A
GPIO 3            → Channel B                Quadrature B
GPIO 25           → Index (Z)                Revolution marker
5V or 3.3V*       → VCC                      Power
GND               → GND                      Ground

* Use 3.3V if encoder supports it, otherwise 5V with level shifters


═══════════════════════════════════════════════════════════════════
                    IMPORTANT NOTES
═══════════════════════════════════════════════════════════════════

1. I2C PULL-UPS
   - Usually built into the PCF8574 backpack (10kΩ typical)
   - No external pull-ups needed
   - If issues, check for 4.7kΩ - 10kΩ pull-ups to 3.3V

2. LCD CONTRAST
   - Most I2C backpacks have a potentiometer on the back
   - Turn clockwise/counter-clockwise to adjust
   - If display is blank or shows blocks, adjust contrast

3. I2C ADDRESS
   - Default: 0x27 (most common)
   - Alternative: 0x3F
   - Adjustable via A0, A1, A2 jumpers on backpack
   - Program auto-scans and shows address on USB serial

4. VOLTAGE LEVELS
   - SKR-Pico GPIO: 3.3V logic (NOT 5V tolerant!)
   - LCD I2C: 5V logic, but PCF8574 usually handles level shifting
   - Encoder: Check if 5V or 3.3V, use level shifters if needed

5. BACKLIGHT CURRENT
   - TC2004A-01: 180mA @ 4.2V (Yellow-Green LED)
   - Powered directly from 5V rail
   - No current limiting resistor needed (built into backpack)

6. USB SERIAL DEBUG
   - Connect micro-USB to computer
   - 115200 baud
   - Shows I2C scan and encoder data
   - Useful for troubleshooting


═══════════════════════════════════════════════════════════════════
                     TYPICAL WIRING
═══════════════════════════════════════════════════════════════════

Minimum 4 wires for LCD:
┌────────────┐         ┌──────────────┐
│ SKR-Pico   │         │ LCD Backpack │
├────────────┤         ├──────────────┤
│ GPIO 0     │────────→│ SDA          │
│ GPIO 1     │────────→│ SCL          │
│ 5V         │────────→│ VCC          │
│ GND        │────────→│ GND          │
└────────────┘         └──────────────┘

Minimum 5 wires for Encoder:
┌────────────┐         ┌──────────────┐
│ SKR-Pico   │         │ Encoder      │
├────────────┤         ├──────────────┤
│ GPIO 4     │←────────│ A            │
│ GPIO 3     │←────────│ B            │
│ GPIO 25    │←────────│ Z            │
│ 5V/3.3V    │────────→│ VCC          │
│ GND        │────────→│ GND          │
└────────────┘         └──────────────┘


═══════════════════════════════════════════════════════════════════
                    LEVEL SHIFTER (If Needed)
═══════════════════════════════════════════════════════════════════

If using 5V encoder with 3.3V GPIO:

Simple Voltage Divider Method:
                                    
    Encoder 5V Signal ──┬── 10kΩ ──┬── GPIO Pin
                        │          │
                        │          └── 10kΩ ── GND
                        │
                     (optional 0.1µF cap to GND for filtering)

This creates: 5V × (10k / 20k) = 2.5V (safe for 3.3V logic)

Or use a proper bi-directional level shifter module (recommended):
- TXS0108E (8-channel)
- BSS138-based level shifter
- Available from Adafruit, SparkFun, etc.


═══════════════════════════════════════════════════════════════════
