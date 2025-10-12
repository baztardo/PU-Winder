# Quick Start Guide

Get your SKR-Pico encoder monitor up and running in minutes!

## Prerequisites Checklist

- [ ] Pico SDK installed
- [ ] `PICO_SDK_PATH` environment variable set
- [ ] ARM GCC compiler installed
- [ ] CMake installed (version 3.13+)
- [ ] VSCode with C/C++ and CMake Tools extensions

## Step 1: Setup Pico SDK (One-time)

### Windows
```powershell
# Clone Pico SDK
cd C:\
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init

# Set environment variable (run as Administrator or add via System Properties)
setx PICO_SDK_PATH "C:\pico-sdk"
```

### Linux/Mac
```bash
# Clone Pico SDK
cd ~
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init

# Set environment variable
echo 'export PICO_SDK_PATH=~/pico-sdk' >> ~/.bashrc  # or ~/.zshrc for Mac
source ~/.bashrc  # or source ~/.zshrc for Mac
```

## Step 2: Open Project in VSCode

1. Extract all project files to a folder (e.g., `encoder_monitor`)
2. Open VSCode
3. File → Open Folder → Select `encoder_monitor` folder
4. VSCode should prompt to configure CMake - click "Yes"

## Step 3: Configure the Project

Edit `config.h` if needed:

```c
// Change LCD I2C address if needed
#define LCD_ADDRESS  0x27  // or 0x3F

// Verify your pin connections match these
#define I2C_SDA_PIN         0
#define I2C_SCL_PIN         1
#define ENCODER_A_PIN       4
#define ENCODER_B_PIN       3
#define ENCODER_Z_PIN       25
```

## Step 4: Build the Project

### Using VSCode
1. Press `F7` or click "Build" in the bottom status bar
2. Wait for compilation to complete
3. Find `encoder_monitor.uf2` in the `build` folder

### Using Command Line
```bash
mkdir build
cd build
cmake ..
make -j4
```

## Step 5: Flash to SKR-Pico

1. **Hold** the **BOOT** button on SKR-Pico
2. **Press** and **release** the **RESET** button
3. **Release** the **BOOT** button
4. The board appears as **RPI-RP2** USB drive
5. **Drag** `encoder_monitor.uf2` to the **RPI-RP2** drive
6. Board automatically reboots and runs your program!

## Step 6: Connect Hardware

### I2C LCD (20x4)
```
LCD          SKR-Pico
----         ---------
VCC     →    5V
GND     →    GND
SDA     →    GPIO 0
SCL     →    GPIO 1
```

### Encoder (360 PPR with A/B/Z)
```
Encoder      SKR-Pico
-------      ---------
VCC     →    5V (or 3.3V)
GND     →    GND
A       →    GPIO 4
B       →    GPIO 3
Z       →    GPIO 25
```

## Step 7: Test

1. **Power on** the SKR-Pico
2. LCD should display:
   ```
   Rev: 0
   RPM: 0.0 CW
   Pulses/Rev: 0
   Count: 0
   ```
3. **Rotate encoder** - numbers should change!

## Viewing USB Serial Output (Optional)

### Windows
- Use PuTTY, TeraTerm, or Arduino Serial Monitor
- Port: COM port for SKR-Pico
- Baud rate: 115200

### Linux
```bash
sudo screen /dev/ttyACM0 115200
# Press Ctrl+A then K to exit
```

### Mac
```bash
screen /dev/cu.usbmodem* 115200
# Press Ctrl+A then K to exit
```

## Troubleshooting

### "CMake Error: PICO_SDK_PATH not found"
- Verify environment variable: `echo $PICO_SDK_PATH` (Linux/Mac) or `echo %PICO_SDK_PATH%` (Windows)
- Restart VSCode/terminal after setting the variable
- Check SDK exists at that path

### "LCD shows nothing"
- Check power connections (VCC/GND)
- Verify I2C address in `config.h` (try 0x27 or 0x3F)
- Check SDA/SCL not swapped

### "Encoder doesn't count"
- Verify encoder power (5V or 3.3V depending on encoder type)
- Check A, B, Z connections
- Ensure encoder outputs are 3.3V compatible

### "Wrong direction"
- Swap A and B wires physically, OR
- Swap `ENCODER_A_PIN` and `ENCODER_B_PIN` in `config.h` and rebuild

### "Build fails"
- Ensure ARM GCC is in PATH: `arm-none-eabi-gcc --version`
- Check CMake version: `cmake --version` (needs 3.13+)
- Verify all pico-sdk submodules: `cd $PICO_SDK_PATH && git submodule update --init`

## Next Steps

- Adjust `UPDATE_INTERVAL_MS` in `config.h` for faster/slower display updates
- Modify display layout in `main.c` function `update_display()`
- Add data logging to SD card
- Implement tachometer features
- Connect multiple encoders

## Support

For more detailed information, see the main `README.md` file.

Enjoy your encoder monitor! 🎉
