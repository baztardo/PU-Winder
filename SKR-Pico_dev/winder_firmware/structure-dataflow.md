# Winder Firmware - Project Structure & Architecture

## 📁 Project Directory Structure

```
winder_project/
├── CMakeLists.txt          # Build configuration
├── config.h                # Hardware configuration & constants
├── main.cpp                # Application logic
├── tmc2209.h/.cpp          # TMC2209 UART driver
├── encoder.h/.cpp          # Quadrature encoder decoder
├── stepcompress.h/.cpp     # Klipper-style step compression
├── move_queue.h/.cpp       # MCU move queue & step consumer
├── scheduler.h/.cpp        # Hardware timer ISR scheduler
└── build/                  # Build output directory
```

## 🏗️ Architecture Overview

### Layer 1: Hardware Abstraction
**Files**: `config.h`
- Central configuration for all hardware pins
- System constants and parameters
- Easy to modify for different hardware setups

### Layer 2: Device Drivers
**Files**: `tmc2209.h/.cpp`, `encoder.h/.cpp`
- **TMC2209**: UART protocol, register management, current control
- **Encoder**: Quadrature decoding, position tracking, velocity estimation

### Layer 3: Motion Control
**Files**: `stepcompress.h/.cpp`, `move_queue.h/.cpp`
- **StepCompressor**: Generates optimized step chunks from motion profiles
- **MoveQueue**: Fixed-size FIFO queue, ISR-driven step execution

### Layer 4: System Management
**Files**: `scheduler.h/.cpp`
- Hardware timer ISR at 10 kHz
- Coordinates encoder updates and step execution
- Provides timing infrastructure

### Layer 5: Application
**Files**: `main.cpp`
- High-level application logic
- Move generation and queuing
- Status monitoring and user interface

## 🔄 Data Flow

```
Motion Command
     ↓
StepCompressor::compress_trapezoid()
     ↓
StepChunk[] (compressed moves)
     ↓
MoveQueue::push_chunk()
     ↓
Fixed-Size Queue (128 chunks)
     ↓
Scheduler ISR (10 kHz)
     ↓
MoveQueue::axis_isr_handler()
     ↓
GPIO Step Pulses
```

## 📚 Module Responsibilities

### config.h
**Purpose**: Centralized configuration
- No code logic, only definitions
- Single place to change hardware setup
- Makes porting to different boards easy

**Key Sections**:
- Pin definitions
- Motor specifications
- Timing parameters
- Queue sizes

### tmc2209.h/.cpp
**Purpose**: TMC2209 stepper driver control

**Key Features**:
- UART packet framing with CRC8
- Register read/write operations
- Current setting (RMS calculation)
- Microstepping configuration
- StealthChop/SpreadCycle modes
- Driver status monitoring

**Usage Example**:
```cpp
TMC2209_UART driver(uart1, 0);
driver.begin(115200);
driver.init_driver(2800, 16);  // 2.8A, 16 microsteps
```

### encoder.h/.cpp
**Purpose**: Quadrature encoder position tracking

**Key Features**:
- Software quadrature decoding
- Position tracking (counts)
- Z-index pulse detection
- Velocity estimation
- Revolution counting

**Usage Example**:
```cpp
Encoder enc;
enc.init();
// In ISR or fast loop:
enc.update();
int32_t pos = enc.get_position();
```

### stepcompress.h/.cpp
**Purpose**: Motion planning and step compression

**Key Features**:
- Trapezoidal velocity profiles
- Bisect algorithm for optimal chunking
- Least-squares fitting
- Configurable error tolerance
- Constant velocity moves

**Algorithm Flow**:
1. Generate absolute step times for trapezoid
2. Binary search for largest chunks within error tolerance
3. Fit chunks using least-squares
4. Output interval/add/count structures

**Usage Example**:
```cpp
auto chunks = StepCompressor::compress_trapezoid(
    3200,    // steps
    0,       // start velocity
    1000,    // cruise velocity
    2000,    // acceleration
    20.0     // max error (µs)
);
```

### move_queue.h/.cpp
**Purpose**: Fixed-size move queue with ISR consumer

**Key Features**:
- Dual-axis queues (spindle + traverse)
- ISR-safe push/pop operations
- Automatic chunk consumption
- Step pulse generation
- Position tracking

**Queue Management**:
- Fixed size: 128 chunks per axis
- Circular buffer (no dynamic allocation)
- Atomic head/tail pointers
- Safe for ISR context

**Usage Example**:
```cpp
MoveQueue queue;
queue.init();

// Queue a chunk
queue.push_chunk(AXIS_SPINDLE, chunk);

// In ISR:
queue.axis_isr_handler(AXIS_SPINDLE);
```

### scheduler.h/.cpp
**Purpose**: High-frequency ISR orchestration

**Key Features**:
- Hardware repeating timer (10 kHz)
- Encoder updates every cycle
- Move queue processing
- User callback support
- Tick counting

**ISR Sequence**:
1. Increment tick counter
2. Update encoder state
3. Process spindle axis steps
4. Process traverse axis steps
5. Call user callback (if registered)

**Usage Example**:
```cpp
Scheduler sched(&move_queue, &encoder);
sched.start(100);  // 100µs = 10 kHz
```

### main.cpp
**Purpose**: Application coordination

**Key Sections**:
- Hardware initialization
- Motor driver setup
- Homing routines
- Move generation
- Status monitoring

**Main Loop**:
- Non-blocking status printing
- Everything else handled by ISR
- Can add user commands, patterns, etc.

## 🎯 Design Principles

### 1. Separation of Concerns
Each module has a single, well-defined responsibility:
- Hardware details → `config.h`
- Device control → Driver modules
- Motion → StepCompressor
- Execution → MoveQueue
- Timing → Scheduler
- Logic → main.cpp

### 2. ISR Safety
- Fixed-size buffers (no malloc in ISR)
- Atomic operations where needed
- Minimal ISR execution time
- No floating point in ISR path

### 3. Testability
Each module can be tested independently:
- Mock GPIO for unit tests
- StepCompressor is pure math
- MoveQueue can be tested without hardware
- Encoder can be simulated

### 4. Extensibility
Easy to add features:
- New axis? Add to config, extend queues
- Different encoder? Swap encoder module
- New driver? Create similar interface
- Custom motion? Extend StepCompressor

## 🔧 Common Modifications

### Change Motor Current
Edit `config.h`:
```cpp
#define SPINDLE_CURRENT_MA  2800
#define TRAVERSE_CURRENT_MA 500
```

### Change ISR Frequency
Edit `config.h`:
```cpp
#define HEARTBEAT_US 100  // 100µs = 10 kHz
```

### Add New Axis
1. Update `config.h`: Add pins, increase `NUM_AXES`
2. Update `move_queue.cpp`: Add axis handling
3. Update `main.cpp`: Initialize new axis

### Change Queue Size
Edit `config.h`:
```cpp
#define MOVE_CHUNKS_CAPACITY 256  // Increase buffer
```

### Tune Step Compression
In `main.cpp` or when calling:
```cpp
auto chunks = StepCompressor::compress_trapezoid(
    steps, v0, v_cruise, accel,
    10.0  // Lower = more chunks, less error
);
```

## 📊 Performance Characteristics

### Memory Usage
- **Code**: ~15-20 KB Flash
- **Static RAM**: ~10 KB (mostly queues)
- **Stack**: ~2 KB typical
- **Total**: Well within RP2040 limits (264 KB RAM, 2 MB Flash)

### Timing
- **ISR Frequency**: 10 kHz (100µs period)
- **ISR Duration**: ~5-15µs typical
- **Step Rate**: Up to 5000 steps/sec per axis
- **Queue Capacity**: 128 chunks × ~100 steps = ~12,800 steps buffered

### Compression Efficiency
- **Input**: Raw step times (1 per step)
- **Output**: Compressed chunks (~50-200 steps each)
- **Compression Ratio**: ~50-200:1
- **Error**: Configurable, typically 20µs

## 🧪 Testing Strategy

### Unit Testing
Each module can be tested independently:

```cpp
// Test StepCompressor
auto chunks = StepCompressor::compress_trapezoid(100, 0, 1000, 1000);
assert(chunks.size() > 0);

// Test Encoder (with mocked GPIO)
encoder.set_position(100);
assert(encoder.get_position() == 100);
```

### Integration Testing
1. **Bench test** (no motors):
   - Verify ISR runs at correct frequency
   - Check encoder counting with hand rotation
   - Monitor queue depth during moves

2. **Motor test** (low speed):
   - Start with slow moves (100 steps/sec)
   - Verify direction control
   - Check for missed steps

3. **Full speed test**:
   - Gradually increase velocity
   - Monitor for errors or stalls
   - Verify smooth motion

## 🚀 Next Steps for Development

### Immediate Enhancements
1. **Add closed-loop sync**: Use encoder to drive traverse
2. **Command interface**: Serial commands for moves
3. **Safety limits**: Software position limits
4. **Error handling**: Detect and report failures

### Advanced Features
1. **Multi-layer winding**: Layer counting and patterns
2. **Tension control**: Closed-loop tension feedback
3. **Variable pitch**: Dynamic pitch adjustment
4. **Web interface**: WiFi control (add Pico W support)

### Optimization
1. **DMA for steps**: Use DMA for ultra-high rates
2. **FPU in compression**: Use hardware FPU
3. **Dual-core**: Offload compression to core 1
4. **Profile ISR**: Optimize critical paths

## 📝 Documentation Standards

Each header file includes:
- Purpose statement
- Key features
- Usage examples
- Parameter descriptions
- Return value meanings

Each implementation file includes:
- Algorithm descriptions
- Important notes
- Performance considerations

## 🐛 Debugging Tips

### ISR Not Running
- Check `scheduler.is_running()`
- Verify timer initialization
- Look for ISR conflicts

### Steps Not Executing
- Check enable pins (active low!)
- Verify queue has chunks
- Monitor `is_active()` status
- Check TMC2209 initialization

### Encoder Not Counting
- Verify pull-ups enabled
- Check wiring (A, B, Z)
- Test with manual rotation
- Monitor voltage levels

### Chunks Not Compressing
- Check error tolerance (too strict?)
- Verify input parameters (accel > 0)
- Debug with small move first

---
