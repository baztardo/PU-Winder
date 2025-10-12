# Project Architecture Guide

## Overview

This project uses a **layered architecture** that separates generic I2C communication from device-specific drivers. This design makes it easy to add new I2C devices without duplicating code.

## Architecture Layers

```
┌─────────────────────────────────────────┐
│         Application Layer               │
│         (main.c, config.h)              │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│       Device Driver Layer               │
│  (LCD, Encoder, MCP23017, etc.)         │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│    Generic I2C Helper Layer             │
│        (i2c_helper.c/h)                 │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│      Pico SDK Hardware Layer            │
│      (hardware/i2c.h)                   │
└─────────────────────────────────────────┘
```

## Directory Structure

```
encoder_monitor/
├── config.h                    # Hardware configuration
├── main.c                      # Application code
│
├── src/
│   ├── i2c/                   # Generic I2C communication
│   │   ├── i2c_helper.h       # - I2C helper API
│   │   └── i2c_helper.c       # - I2C implementations
│   │
│   ├── drivers/               # Device-specific drivers
│   │   ├── lcd_pcf8574.h     # - LCD with PCF8574 backpack
│   │   ├── lcd_pcf8574.c
│   │   ├── mcp23017.h        # - I2C GPIO expander (example)
│   │   └── mcp23017.c
│   │
│   └── encoder/              # Encoder (non-I2C peripheral)
│       ├── encoder.h
│       └── encoder.c
│
├── CMakeLists.txt
└── pico_sdk_import.cmake
```

## Why This Architecture?

### ❌ Old Approach (Monolithic)

```c
// lcd_i2c.c - Everything mixed together
void lcd_init() {
    // I2C initialization code
    i2c_init(i2c0, 100000);
    gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    
    // LCD-specific code
    lcd_send_command(0x33);
    // ...
}

// If you add another I2C device, you duplicate all the I2C code!
```

### ✅ New Approach (Layered)

```c
// i2c_helper.c - Generic I2C functions
bool i2c_helper_init(const i2c_config_t *config) {
    // Initialize I2C (used by ALL devices)
}

// lcd_pcf8574.c - Only LCD logic
void lcd_init(lcd_t *lcd, i2c_inst_t *port, ...) {
    // Just use i2c_helper functions
    i2c_helper_write_byte(port, address, data);
}

// mcp23017.c - Only GPIO expander logic
bool mcp23017_init(mcp23017_t *dev, i2c_inst_t *port, ...) {
    // Just use i2c_helper functions
    i2c_helper_write_register(port, address, reg, val);
}
```

## Benefits

### 1. **No Code Duplication**
- I2C initialization is written once in `i2c_helper.c`
- All devices reuse the same I2C functions
- DRY principle: Don't Repeat Yourself

### 2. **Easy to Add New Devices**
Create a new driver in `src/drivers/`:

```c
// my_sensor.h
typedef struct {
    i2c_inst_t *i2c_port;
    uint8_t address;
} my_sensor_t;

void my_sensor_init(my_sensor_t *dev, i2c_inst_t *port, uint8_t addr);
float my_sensor_read_temperature(my_sensor_t *dev);
```

```c
// my_sensor.c
#include "my_sensor.h"
#include "../i2c/i2c_helper.h"

void my_sensor_init(my_sensor_t *dev, i2c_inst_t *port, uint8_t addr) {
    dev->i2c_port = port;
    dev->address = addr;
    
    // Just use i2c_helper!
    i2c_helper_device_present(port, addr);
}

float my_sensor_read_temperature(my_sensor_t *dev) {
    uint8_t data[2];
    i2c_helper_read_register(dev->i2c_port, dev->address, TEMP_REG, data);
    return convert_to_celsius(data);
}
```

Add to `CMakeLists.txt`:
```cmake
add_executable(encoder_monitor
    main.c
    src/i2c/i2c_helper.c
    src/drivers/lcd_pcf8574.c
    src/drivers/my_sensor.c      # <-- Just add this line!
    src/encoder/encoder.c
)
```

### 3. **Multiple I2C Buses**
The architecture supports multiple I2C buses cleanly:

```c
// Initialize two I2C buses
i2c_config_t i2c0_config = {
    .port = i2c0,
    .sda_pin = 0,
    .scl_pin = 1,
    .baudrate = 100000
};
i2c_helper_init(&i2c0_config);

i2c_config_t i2c1_config = {
    .port = i2c1,
    .sda_pin = 6,
    .scl_pin = 7,
    .baudrate = 400000
};
i2c_helper_init(&i2c1_config);

// LCD on I2C0
lcd_init(&lcd, i2c0, 0x27, 20, 4);

// Sensor on I2C1
sensor_init(&sensor, i2c1, 0x76);
```

### 4. **Built-in Debugging Tools**
The `i2c_helper` provides useful debugging functions:

```c
// Scan for all devices on the bus
uint8_t found_devices[128];
int count = i2c_helper_scan(i2c0, found_devices);
printf("Found %d devices\n", count);

// Check if specific device is present
if (i2c_helper_device_present(i2c0, 0x27)) {
    printf("LCD found!\n");
}
```

### 5. **Device Independence**
Each driver is self-contained and doesn't depend on other drivers:

```c
// lcd_pcf8574.c - Only needs i2c_helper
#include "lcd_pcf8574.h"
#include "../i2c/i2c_helper.h"

// mcp23017.c - Only needs i2c_helper
#include "mcp23017.h"
#include "../i2c/i2c_helper.h"

// They don't know about each other!
```

### 6. **Reusable Across Projects**
Copy the entire `src/` folder to a new project and it just works:

```
new_project/
├── main.c                     # Different application
├── src/                       # Same drivers!
│   ├── i2c/
│   └── drivers/
```

## API Design

### I2C Helper Functions

| Function | Purpose | Use Case |
|----------|---------|----------|
| `i2c_helper_init()` | Initialize I2C bus | Setup at startup |
| `i2c_helper_write()` | Write multiple bytes | Send data block |
| `i2c_helper_read()` | Read multiple bytes | Receive data block |
| `i2c_helper_write_byte()` | Write single byte | Simple command |
| `i2c_helper_read_byte()` | Read single byte | Simple read |
| `i2c_helper_write_register()` | Write to register | Configure device |
| `i2c_helper_read_register()` | Read from register | Query device state |
| `i2c_helper_scan()` | Scan I2C bus | Find devices |
| `i2c_helper_device_present()` | Check if device exists | Validation |

### Driver Pattern

All I2C device drivers follow this pattern:

```c
// 1. Define device structure
typedef struct {
    i2c_inst_t *i2c_port;
    uint8_t address;
    // ... device-specific data
} device_t;

// 2. Initialize device
void device_init(device_t *dev, i2c_inst_t *port, uint8_t addr) {
    dev->i2c_port = port;
    dev->address = addr;
    // ... device-specific initialization using i2c_helper
}

// 3. Device-specific functions
void device_do_something(device_t *dev) {
    // Use i2c_helper functions
    i2c_helper_write_register(dev->i2c_port, dev->address, reg, val);
}
```

## Example: Adding an OLED Display

Let's add an SSD1306 OLED display:

### Step 1: Create driver files

```bash
touch src/drivers/ssd1306.h
touch src/drivers/ssd1306.c
```

### Step 2: Define API (ssd1306.h)

```c
#ifndef SSD1306_H
#define SSD1306_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

typedef struct {
    i2c_inst_t *i2c_port;
    uint8_t address;
    uint8_t width;
    uint8_t height;
} ssd1306_t;

void ssd1306_init(ssd1306_t *oled, i2c_inst_t *port, uint8_t addr);
void ssd1306_clear(ssd1306_t *oled);
void ssd1306_draw_pixel(ssd1306_t *oled, uint8_t x, uint8_t y);
void ssd1306_update(ssd1306_t *oled);

#endif
```

### Step 3: Implement driver (ssd1306.c)

```c
#include "ssd1306.h"
#include "../i2c/i2c_helper.h"

void ssd1306_init(ssd1306_t *oled, i2c_inst_t *port, uint8_t addr) {
    oled->i2c_port = port;
    oled->address = addr;
    
    // Use i2c_helper for all communication
    i2c_helper_write_register(port, addr, 0x00, 0xAE); // Display off
    i2c_helper_write_register(port, addr, 0x00, 0x8D); // Enable charge pump
    // ... more initialization
}

void ssd1306_clear(ssd1306_t *oled) {
    // Use i2c_helper functions
}
```

### Step 4: Add to CMakeLists.txt

```cmake
add_executable(encoder_monitor
    main.c
    src/i2c/i2c_helper.c
    src/drivers/lcd_pcf8574.c
    src/drivers/ssd1306.c          # <-- Add this
    src/encoder/encoder.c
)
```

### Step 5: Use in main.c

```c
#include "src/drivers/ssd1306.h"

ssd1306_t oled;
ssd1306_init(&oled, i2c0, 0x3C);
ssd1306_clear(&oled);
ssd1306_draw_pixel(&oled, 10, 10);
ssd1306_update(&oled);
```

## Testing Strategy

### Unit Test Each Layer

```c
// Test i2c_helper
void test_i2c_scan() {
    uint8_t devices[128];
    int count = i2c_helper_scan(i2c0, devices);
    assert(count > 0);
}

// Test LCD driver
void test_lcd() {
    lcd_t lcd;
    lcd_init(&lcd, i2c0, 0x27, 20, 4);
    lcd_print(&lcd, "Test");
}
```

## Performance Considerations

1. **I2C Speed**: Configure in `i2c_helper_init()`
   - Standard: 100 kHz (default)
   - Fast: 400 kHz
   - Fast Plus: 1 MHz

2. **Function Overhead**: Minimal
   - Helper functions are thin wrappers
   - Compiler can inline them
   - No performance penalty

3. **Memory**: Efficient
   - No global state in helper
   - Each device manages its own data
   - Stack usage is minimal

## Migration from Old Structure

If you have existing monolithic drivers:

1. Extract I2C calls to `i2c_helper`
2. Remove I2C initialization from drivers
3. Pass `i2c_inst_t*` to driver functions
4. Update includes and CMakeLists.txt

## Summary

This architecture provides:
- ✅ Clear separation of concerns
- ✅ Easy to add new I2C devices
- ✅ No code duplication
- ✅ Better testing
- ✅ Reusable across projects
- ✅ Supports multiple I2C buses
- ✅ Built-in debugging tools

**The key insight**: Separate the "how to communicate" (I2C protocol) from the "what to communicate" (device commands).
