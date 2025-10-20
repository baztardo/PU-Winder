# How to Build Updated Firmware

## Step 1: Pull Latest Code
```bash
cd /path/to/PU-Winder
git checkout cursor/integrate-pio-encoder-and-lcd-into-winder-project-bb8c
git pull origin cursor/integrate-pio-encoder-and-lcd-into-winder-project-bb8c
```

## Step 2: Clean Build
```bash
cd SKR-Pico_dev/winder_project
rm -rf build
mkdir build
cd build
```

## Step 3: Configure
```bash
cmake ..
```

## Step 4: Build
```bash
make -j4
```

## Step 5: Get Firmware
```bash
ls -lh winder_project.uf2
# Should be ~138 KB
```

---

## Files That Changed (Commit 9e06bd4):

### ✅ SKR-Pico_dev/winder_project/src/winding_controller.cpp
**Line 165-170:** Fixed Z-index homing to use PIO instead of move_queue

### Other Files Already in Branch:
- ✅ src/spindle_step.pio (PIO program for spindle)
- ✅ src/spindle_step_pio.cpp (PIO wrapper)
- ✅ src/spindle_step_pio.h (PIO header)
- ✅ src/encoder.pio (PIO encoder reader)
- ✅ CMakeLists.txt (includes PIO compilation)

---

## What the Fix Does:

**Problem:** During spindle homing, code called `move_queue->push_chunk()` but spindle uses PIO, so it never moved → Z-index timeout.

**Solution:** Changed to call `spindle_step_pio_queue_cv()` directly → spindle ACTUALLY rotates → Z-index detected!

---

## Quick Test Without Rebuilding:

If you just want to test, use the pre-built firmware at repo root:
```bash
cd /path/to/PU-Winder
ls -lh UPLOAD_THIS.uf2
# Upload this to Pico
```

But for future development, rebuild from source!
