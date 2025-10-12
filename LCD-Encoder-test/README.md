# SKR-Pico Encoder Monitor (Pico SDK)

A complete C program for the SKR-Pico board using the Raspberry Pi Pico SDK to monitor a 360 PPR quadrature encoder with Z-index and display real-time information on an I2C LCD.

## Features

- **Quadrature Decoding**: Full 4x resolution (1440 counts per revolution)
- **Direction Detection**: Tracks clockwise (CW) and counter-clockwise (CCW) rotation
- **Z-Index Tracking**: Uses Z-channel for accurate revolution counting
- **Real-time RPM**: Calculates and displays RPM (positive for CW, negative for CCW)
- **Pulse Counting**: Tracks pulses per revolution
- **20x4 LCD Display**: Shows all parameters simultaneously
- **USB Serial Debugging**: Outputs data to USB serial port
- **Modular Architecture**: Layered design with reusable I2C helper library

## Architecture

This project uses a **layered architecture** that separates I2C communication from device drivers:

```
Application (main.c)
        ↓
Device Drivers (LCD, etc.)
        ↓
I2C Helper Library
        ↓
Pico SDK Hardware
```

**Benefits:**
- ✅ Easy to add new I2C devices (LCD, OLED, sensors, GPIO expanders)
- ✅ No code duplication
- ✅ Built-in I2C bus scanning and debugging
- ✅ Supports multiple I2C buses
- ✅ Reusable across projects

See [ARCHITECTURE.md](ARCHITECTURE.md) for detailed information.

## Hardware Requirements

- **SKR-Pico Board** (RP2040-based)
- **I2C LCD** (20x4 with PCF8574 I2C backpack)
- **360 PPR Encoder** with A, B, and Z channels

## Pin Connections

### I2C LCD
- SDA → GPIO 0
- SCL → GPIO 1
- VCC → 5V
- GND → GND

### Encoder
- Channel A → GPIO 4
- Channel B → GPIO 3
- Channel Z → GPIO 25
- VCC → 5V (or 3.3V depending on encoder)
- GND → GND

## File Structure

```
encoder_monitor/
├── CMakeLists.txt              # CMake build configuration
├── pico_sdk_import.cmake       # Pico SDK import script
├── config.h                    # Hardware configuration & pin definitions
├── main.c                      # Main application code
│
├── src/                        # Source code organized by function
│   │
│   ├── i2c/                    # Generic I2C communication layer
│   │   ├── i2c_helper.h        # - I2C helper API
│   │   └── i2c_helper.c        # - I2C implementations
│   │
│   ├── drivers/                # Device-specific I2C drivers
│   │   ├── lcd_pcf8574.h      # - LCD with PCF8574 I2C backpack
│   │   ├── lcd_pcf8574.c
│   │   ├── mcp23017.h         # - MCP23017 GPIO expander (example)
│   │   └── mcp23017.c
│   │
│   └── encoder/               # Encoder library (non-I2C)
│       ├── encoder.h
│       └── encoder.c
│
├── .vscode/                   # VSCode IDE configuration
│   ├── settings.json
│   ├── c_cpp_properties.json
│   └── launch.json
│
├── README.md                  # This file
├── QUICKSTART.md              # Quick start guide
├── ARCHITECTURE.md            # Architecture documentation
└── PROJECT_STRUCTURE.md       # File organization guide
```

## Prerequisites

### Windows

1. **Install CMake**: Download from [cmake.org](https://cmake.org/download/)
   - Add to PATH during installation

2. **Install ARM GCC Compiler**:
   - Download [ARM GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
   - Extract and add `bin` folder to PATH

3. **Install Pico SDK**:
   ```bash
   git clone https://github.com/raspberrypi/pico-sdk.git
   cd pico-sdk
   git submodule update --init
   ```

4. **Set Environment Variable**:
   - Add `PICO_SDK_PATH` pointing to your pico-sdk directory
   - Example: `C:\pico-sdk`

5. **Install Build Tools**:
   - Install [Visual Studio Build Tools](https://visualstudio.microsoft.com/downloads/) or
   - Install [MinGW-w64](https://www.mingw-w64.org/)

6. **Install VSCode Extensions**:
   - C/C++ (Microsoft)
   - CMake Tools (Microsoft)
   - CMake (twxs)

### Linux (Ubuntu/Debian)

```bash
# Install dependencies
sudo apt update
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib build-essential

# Install Pico SDK
cd ~
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init

# Set environment variable
echo 'export PICO_SDK_PATH=~/pico-sdk' >> ~/.bashrc
source ~/.bashrc
```

### macOS

```bash
# Install Homebrew if not already installed
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install dependencies
brew install cmake
brew tap ArmMbed/homebrew-formulae
brew install arm-none-eabi-gcc

# Install Pico SDK
cd ~
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init

# Set environment variable
echo 'export PICO_SDK_PATH=~/pico-sdk' >> ~/.zshrc
source ~/.zshrc
```

## Building the Project

### Method 1: Command Line

```bash
# Navigate to project directory
cd encoder_monitor

# Create build directory
mkdir build
cd build

# Configure CMake
cmake ..

# Build the project
make -j4

# The output file will be: encoder_monitor.uf2
```

### Method 2: VSCode with CMake Tools

1. Open the project folder in VSCode
2. Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac)
3. Type "CMake: Configure" and press Enter
4. Select your compiler (GCC for arm-none-eabi)
5. Press `F7` or click "Build" in the status bar
6. The `.uf2` file will be in the `build` directory

## Flashing to SKR-Pico

1. **Enter Bootloader Mode**:
   - Hold the BOOT button on the SKR-Pico
   - Press and release the RESET button
   - Release the BOOT button
   - The Pico should appear as a USB mass storage device (RPI-RP2)

2. **Flash the Firmware**:
   - Copy `encoder_monitor.uf2` to the RPI-RP2 drive
   - The board will automatically reboot and run your program

## Configuration

All configuration is in `config.h`:

### LCD Address
```c
#define LCD_ADDRESS  0x27  // Try 0x3F if 0x27 doesn't work
```

### LCD Size
```c
#define LCD_COLS  20
#define LCD_ROWS  4
```

### Encoder Parameters
```c
#define ENCODER_PPR  360    // Pulses per revolution
#define ENCODER_CPR  1440   // Counts per revolution (PPR × 4)
```

### Pin Assignments
```c
#define I2C_SDA_PIN     0
#define I2C_SCL_PIN     1
#define ENCODER_A_PIN   4
#define ENCODER_B_PIN   3
#define ENCODER_Z_PIN   25
```

## Display Layout (20x4 LCD)

```
Line 1: Rev: [revolution count]
Line 2: RPM: [rpm value] [CW/CCW]
Line 3: Pulses/Rev: [pulse count]
Line 4: Count: [total encoder count]
```

## USB Serial Debugging

The program outputs debug information via USB serial at 115200 baud:

1. Connect to the USB serial port using:
   - **Windows**: PuTTY, TeraTerm, or Arduino Serial Monitor
   - **Linux/Mac**: `screen /dev/ttyACM0 115200` or `minicom`

2. Example output:
   ```
   Rev: 10 | RPM: 125.3 | Dir: CW | Pulses: 245 | Count: 14400
   ```

## Troubleshooting

### CMake Cannot Find Pico SDK
- Verify `PICO_SDK_PATH` environment variable is set correctly
- Restart VSCode/terminal after setting the variable
- Check that pico-sdk submodules are initialized: `git submodule update --init`

### LCD Not Working
1. Check I2C address (use I2C scanner if needed)
2. Verify connections (SDA/SCL, VCC/GND)
3. Try different I2C address: `0x27` or `0x3F`
4. Check LCD backpack is properly soldered

### Encoder Not Counting
1. Verify encoder power supply (5V or 3.3V)
2. Check A, B, Z pin connections
3. Ensure encoder outputs are 3.3V compatible or use level shifters
4. Check for loose connections

### Incorrect Direction
- Swap encoder A and B pins physically, or
- Swap `ENCODER_A_PIN` and `ENCODER_B_PIN` in `config.h`

### Build Errors
- Ensure ARM GCC compiler is installed and in PATH
- Verify CMake version is 3.13 or higher: `cmake --version`
- Check that all Pico SDK submodules are present
- Clean build: `rm -rf build` and rebuild

### USB Serial Not Working
- Some terminal programs may need the board to be reset after connection
- Try different baud rates if 115200 doesn't work
- On Linux, you may need permissions: `sudo usermod -a -G dialout $USER`

## Advanced Configuration

### Changing I2C Speed
In `config.h`:
```c
#define I2C_BAUDRATE  100000  // 100kHz (standard)
// #define I2C_BAUDRATE  400000  // 400kHz (fast mode)
```

### Adjusting Display Update Rate
In `config.h`:
```c
#define UPDATE_INTERVAL_MS  100   // Update LCD every 100ms
#define RPM_SAMPLE_MS       100   // Calculate RPM every 100ms
```

### Using Different I2C Port
In `config.h`:
```c
#define I2C_PORT  i2c1  // Use I2C1 instead of I2C0
// Also change pins to I2C1 pins (e.g., GPIO 6/7, 10/11, etc.)
```

## VSCode Tasks (Optional)

Create `.vscode/tasks.json` for quick build/flash:

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "Build",
            "type": "shell",
            "command": "cmake --build build",
            "group": {
                "kind": "build",
                "isDefault": true
            }
        }
    ]
}
```

## Performance Notes

- Interrupt-driven encoder reading provides accurate high-speed counting
- RPM calculation updates every 100ms by default
- LCD updates are rate-limited to prevent I2C bus overload
- Quadrature decoding handles up to ~100kHz encoder frequency

## License

This code is provided as-is for educational and commercial use.

## Credits

Developed for SKR-Pico board using Raspberry Pi Pico SDK.
