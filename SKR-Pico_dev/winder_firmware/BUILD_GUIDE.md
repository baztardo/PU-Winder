# Klipper-Style Winder Firmware - Build & Usage Guide

## Hardware Configuration

### Pin Assignments (SKR Pico)
- **Spindle Stepper**: Step=11, Dir=10, Enable=12
- **Traverse Stepper**: Step=6, Dir=5, Enable=7
- **Traverse Home Switch**: GPIO 14 (active low)
- **Encoder**: A=4, B=3, Z=25 (1:1 ratio, 360 PPR → 720 quadrature)
- **TMC2209 UART**: TX=8, RX=9 (shared bus)
- **I2C**: SDA=0, SCL=1

### Hardware Specifications
- **Spindle Motor**: 2.8A RMS current, 16 microsteps
- **Traverse Motor**: 0.5A RMS current, 16 microsteps
- **R_sense**: 0.11Ω
- **Lead Screw**: 5mm/revolution

## Building the Firmware

### Prerequisites
1. Install Pico SDK:
```bash
cd ~
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init
export PICO_SDK_PATH=~/pico-sdk
```

2. Install build tools:
```bash
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
```

### Build Steps

1. **Create project directory**:
```bash
mkdir winder_project
cd winder_project
```

2. **Copy the firmware files**:
   - Save `main.cpp` to the project directory
   - Save `CMakeLists.txt` to the project directory

3. **Build the firmware**:
```bash
mkdir build
cd build
cmake ..
make -j4
```

4. **Flash to SKR Pico**:
   - Hold BOOTSEL button while plugging in USB
   - Copy `winder_firmware.uf2` to the RPI-RP2 drive
   - Board will reboot automatically

## Testing & Verification

### 1. Initial Power-Up Test (No Motors)
- Connect USB to view serial output
- Verify TMC2209 initialization messages
- Check encoder counts update when manually rotating spindle

### 2. TMC2209 UART Verification
- Use logic analyzer on TX=8, RX=9 if available
- Verify baud rate: 115200
- Check for valid CRC in packets

### 3. Motor Test (Low Speed)
Modify the example move parameters in `main()`:
```cpp
uint32_t steps = 200;     // Small test move
double start_v = 0;
double cruise_v = 100;    // Slow speed (100 steps/sec)
double accel = 200;       // Gentle acceleration
```

### 4. Full Speed Test
After verifying low-speed operation:
```cpp
uint32_t steps = 3200;    // Full revolution
double start_v = 0;
double cruise_v = 1000;   // 1000 steps/sec
double accel = 2000;      // 2000 steps/sec²
```

## Architecture Overview

### Step Compression Pipeline
1. **Input**: Target position, velocities, acceleration
2. **Trapezoid Generator**: Creates absolute step times
3. **Bisect Compressor**: Finds optimal chunks with error tolerance
4. **Move Queue**: Stores compressed chunks (128 per axis)
5. **ISR Consumer**: Executes steps at 10 kHz

### Key Classes

#### `StepCompressor`
- Generates trapezoidal velocity profiles
- Compresses steps into `interval/add/count` chunks
- Uses bisect + least-squares fitting (Klipper algorithm)

#### `MoveQueue`
- Fixed-size FIFO per axis
- ISR-safe push/pop operations
- Automatic chunk consumption

#### `TMC2209_UART`
- Register read/write via UART
- Current setting with RMS calculation
- Microstepping configuration

#### `Encoder`
- Quadrature decoding in software
- Updated every ISR cycle (100μs)
- Position tracking for closed-loop control

## Performance Characteristics

### Timing
- **ISR Frequency**: 10 kHz (100μs period)
- **Step Pulse Width**: 2μs
- **Maximum Step Rate**: ~5000 steps/sec per axis (conservative)

### Compression Efficiency
- **Error Tolerance**: 20μs (adjustable)
- **Typical Chunk Size**: 50-200 steps
- **Queue Depth**: 128 chunks = ~6400-25600 steps buffered

## Safety Features

1. **Fixed-Size Queues**: No dynamic allocation, no fragmentation
2. **Wraparound-Safe Timers**: Uses 32-bit unsigned arithmetic
3. **Enable Pins**: Motors can be disabled instantly
4. **Current Limiting**: TMC2209 hardware protection
5. **Home Switch**: Traverse axis has hardware limit

## Customization

### Adjusting Step Compression
In `StepCompressor::compress_trapezoid()`:
```cpp
// Change error tolerance (larger = fewer chunks, more error)
double max_err_us = 20.0;  // Default: 20μs
```

### Changing Motor Currents
In `main()`:
```cpp
#define SPINDLE_CURRENT_MA  2800  // Adjust as needed
#define TRAVERSE_CURRENT_MA 500   // Adjust as needed
```

### Modifying Queue Size
```cpp
#define MOVE_CHUNKS_CAPACITY 128  // Increase for longer buffering
```

### Adjusting ISR Frequency
```cpp
#define HEARTBEAT_US 100  // 100μs = 10 kHz
                          // Lower = faster response, higher CPU load
```

## Troubleshooting

### Motors Don't Move
1. Check enable pins are low (active low)
2. Verify TMC2209 UART initialization succeeded
3. Check power supply voltage (12-24V)
4. Verify step pulse width (should see 2μs pulses)

### Encoder Not Counting
1. Check pull-ups are enabled
2. Verify encoder power supply
3. Test with multimeter: should see voltage transitions
4. Manually rotate and check serial output

### TMC2209 Not Responding
1. Verify RX/TX not swapped
2. Check baud rate (115200)
3. Try increasing delays in `init_driver()`
4. Verify common ground between Pico and drivers

### Steps Missing at High Speed
1. Reduce `cruise_v` and `accel` parameters
2. Increase voltage to motors
3. Check for mechanical binding
4. Verify TMC2209 current settings

## Next Steps for Production

1. **Add closed-loop traverse sync** based on spindle encoder
2. **Implement continuous winding patterns** with synchronized motion
3. **Add USB/Serial command interface** for move queuing
4. **Implement stallGuard** for stall detection
5. **Add multi-layer winding logic** with layer counters
6. **Create host-side step compression** (Python/C++)

## Performance Monitoring

The firmware prints encoder position every second. To add more monitoring:

```cpp
// Add to main loop:
printf("Queue depth: Spindle=%u, Traverse=%u\n", 
    spindle_queue_depth, traverse_queue_depth);
printf("Current position: Spindle=%ld steps\n", 
    spindle_position);
```

## License & Credits

Based on Klipper's motion control algorithms:
- Step compression: `stepcompress.c`
- Trapezoid generation: Similar to Marlin/Klipper
- TMC UART protocol: Trinamic datasheets

---

**Support**: For issues, check the serial console output first. Most problems will be reported during initialization.