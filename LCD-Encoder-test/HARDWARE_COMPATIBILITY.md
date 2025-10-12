# Hardware Compatibility

## Tested Hardware

### LCD Display

#### ✅ TC2004A-01 (Primary Test Display)

**Manufacturer**: Tinsharp Industrial Co., Ltd.  
**Available from**: Adafruit, other distributors  
**Datasheet**: https://cdn-shop.adafruit.com/datasheets/TC2004A-01.pdf

**Specifications:**
- Display: 20 characters × 4 lines
- Controller: SPLC780D1 (HD44780-compatible)
- Interface: I2C via PCF8574 backpack
- Voltage: 5V logic, 4.7V LCD
- Backlight: Yellow-green LED, 4.2V, 180mA
- I2C Address: Typically 0x27 or 0x3F

**Configuration in config.h:**
```c
#define LCD_ADDRESS  0x27    // May need to try 0x3F
#define LCD_COLS     20
#define LCD_ROWS     4
```

**DDRAM Address Map:**
```
Line 1: 0x00-0x13  (addresses 0-19)
Line 2: 0x40-0x53  (addresses 64-83)
Line 3: 0x14-0x27  (addresses 20-39)
Line 4: 0x54-0x67  (addresses 84-103)
```

Our driver uses these exact addresses, so it's fully compatible!

#### Other Compatible LCD Displays

Any LCD with these characteristics should work:
- ✅ HD44780 or compatible controller (ST7066, SPLC780, KS0066, etc.)
- ✅ PCF8574 I2C backpack/adapter
- ✅ 5V logic levels
- ✅ Standard 20x4 or 16x2 character displays

**Common Models:**
- Standard 1602 (16x2)
- Standard 2004 (20x4)
- Most generic I2C character LCDs from Amazon/eBay/AliExpress

**To use 16x2 displays**, just change config.h:
```c
#define LCD_COLS  16
#define LCD_ROWS  2
```

### SKR-Pico Board

**Manufacturer**: BigTreeTech (BTT)  
**Processor**: RP2040 (Raspberry Pi Pico compatible)  
**I2C**: Hardware I2C on GPIO 0/1 (i2c0)

**Pin Mapping:**
```
I2C0 SDA → GPIO 0
I2C0 SCL → GPIO 1
I2C1 SDA → GPIO 6 or 10
I2C1 SCL → GPIO 7 or 11
```

### Encoder

**Type**: Incremental rotary encoder with quadrature output  
**Specification**: 360 PPR (Pulses Per Revolution)  
**Channels**: A, B, and Z (index)  
**Output**: Quadrature signals (90° phase shift)

**Configuration in config.h:**
```c
#define ENCODER_A_PIN    4
#define ENCODER_B_PIN    3
#define ENCODER_Z_PIN    25
#define ENCODER_PPR      360    // Adjust for your encoder
#define ENCODER_CPR      1440   // PPR × 4 for quadrature
```

**Compatible Encoders:**
- Any quadrature encoder with A/B/Z outputs
- Common PPR values: 100, 200, 360, 400, 600, 1000, 1024
- Both 5V and 3.3V encoders (SKR-Pico is 3.3V tolerant)

**To use different PPR encoders**, update config.h:
```c
#define ENCODER_PPR  600     // Your encoder's PPR
#define ENCODER_CPR  2400    // PPR × 4
```

## Finding Your I2C Address

If your LCD doesn't work with 0x27, try these steps:

### Method 1: Check Hardware
Most I2C backpacks have the address printed on them:
- A0, A1, A2 jumpers determine address
- Default (no jumpers): Usually 0x27
- With jumpers: 0x20-0x27 or 0x38-0x3F range

### Method 2: Use I2C Scanner
Our program automatically scans on startup:
```
I2C Bus Scan
   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
20 -- -- -- -- -- -- -- 27 -- -- -- -- -- -- -- --
```

The scan shows your LCD address. Update `config.h` accordingly.

### Method 3: Try Common Addresses
```c
// In config.h, try each:
#define LCD_ADDRESS  0x27   // Most common
#define LCD_ADDRESS  0x3F   // Second most common
#define LCD_ADDRESS  0x20   // Alternative
#define LCD_ADDRESS  0x38   // Alternative
```

## Multiple I2C Devices

Our architecture makes it easy to add more devices:

### Example: Add OLED Display on Same Bus
```c
// Both on i2c0
lcd_init(&lcd, i2c0, 0x27, 20, 4);      // LCD
oled_init(&oled, i2c0, 0x3C);           // OLED at different address
```

### Example: Use Two I2C Buses
```c
// Initialize both I2C buses
i2c_config_t i2c0_cfg = { .port = i2c0, .sda_pin = 0, .scl_pin = 1, .baudrate = 100000 };
i2c_config_t i2c1_cfg = { .port = i2c1, .sda_pin = 6, .scl_pin = 7, .baudrate = 400000 };

i2c_helper_init(&i2c0_cfg);
i2c_helper_init(&i2c1_cfg);

// LCD on i2c0
lcd_init(&lcd, i2c0, 0x27, 20, 4);

// Fast sensors on i2c1
sensor_init(&bme280, i2c1, 0x76);
```

## Voltage Level Compatibility

### SKR-Pico GPIO Levels
- **Logic Level**: 3.3V
- **5V Tolerant**: ⚠️ NO! Do not connect 5V signals directly to GPIO

### LCD with I2C Backpack
- **Logic Level**: 5V
- **Solution**: PCF8574 I2C backpack typically has built-in level shifting
- **Safe**: Most I2C LCD modules with backpacks work directly with 3.3V I2C

### Encoder
- **5V Encoder**: May need level shifters (or use resistor dividers)
- **3.3V Encoder**: Works directly
- **Open Collector**: Works with pull-ups to 3.3V

**Simple Level Shifter (if needed):**
```
Encoder 5V → 10kΩ resistor → GPIO pin
                       ↓
                    10kΩ to GND
```
This creates a voltage divider: 5V × (10k / 20k) = 2.5V (safe for 3.3V GPIO)

## Troubleshooting

### LCD Not Detected
1. Check I2C connections (SDA/SCL not swapped)
2. Check power (5V and GND)
3. Try different I2C address in config.h
4. Check I2C pull-up resistors (usually on backpack)
5. View I2C scan output on USB serial

### LCD Shows Blocks
- Normal on power-up before initialization
- If persistent: adjust contrast (V0) with potentiometer on backpack
- Check initialization sequence completed

### Encoder Not Counting
1. Check power (5V or 3.3V depending on encoder)
2. Verify pin connections (A, B, Z)
3. Check voltage levels (may need level shifters for 5V encoders)
4. Verify encoder outputs are not open-drain without pull-ups

### Wrong Direction
- Swap A and B pins physically, OR
- Swap `ENCODER_A_PIN` and `ENCODER_B_PIN` in config.h

## Expansion Possibilities

With the modular architecture, you can easily add:

### Sensors
- BME280/BMP280 (temperature, pressure, humidity)
- MPU6050 (accelerometer, gyroscope)
- ADS1115 (ADC)
- INA219 (current/voltage sensor)

### Displays
- SSD1306 OLED (128x64 or 128x32)
- ST7735 TFT (if using SPI)

### I/O Expansion
- MCP23017 (16-bit GPIO expander) - example included!
- PCF8574 (8-bit GPIO expander)
- PCA9685 (16-channel PWM driver)

### Storage
- AT24C256 EEPROM
- DS3231 RTC with EEPROM

All using the same `i2c_helper` library!

## References

- [SKR-Pico GitHub](https://github.com/bigtreetech/SKR-Pico)
- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [HD44780 LCD Controller](https://www.sparkfun.com/datasheets/LCD/HD44780.pdf)
- [PCF8574 I2C Expander](https://www.ti.com/lit/ds/symlink/pcf8574.pdf)

## Summary

Your **TC2004A-01** LCD is fully compatible with this project. The configuration is already correct in `config.h`, and the driver uses the exact DDRAM addresses from the TC2004A-01 datasheet. Just build, flash, and it should work perfectly!

If you have any issues, check:
1. I2C address (try 0x27 or 0x3F)
2. Connections (don't swap SDA/SCL)
3. Power (5V to LCD, 3.3V logic on I2C)
4. USB serial output for debug info
