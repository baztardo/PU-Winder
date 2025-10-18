# Build Instructions - Z-Index Fix Applied

## What Changed

### Source Code Fix (Commit 9e06bd4):
**File:** `src/winding_controller.cpp` (Line 165-170)

**Problem:** Z-index homing called `move_queue->push_chunk()` but spindle uses PIO, so motor never moved.

**Solution:** Changed to `spindle_step_pio_queue_cv()` to directly control PIO → spindle rotates → Z-index detected!

---

## Rebuild From Source

```bash
# 1. Navigate to project
cd SKR-Pico_dev/winder_project

# 2. Clean build
rm -rf build
mkdir build
cd build

# 3. Configure
cmake ..

# 4. Build
make -j4

# 5. Get firmware
ls -lh winder_project.uf2  # Should be ~138 KB
```

---

## Quick Test (Pre-built Firmware)

Pre-built firmware available at repo root:
- `UPLOAD_THIS.uf2` (138 KB) - Ready to upload!

---

## All Files in This Branch

### PIO Programs:
- ✅ `src/spindle_step.pio` - Hardware step generator
- ✅ `src/encoder.pio` - Hardware encoder reader

### PIO Wrappers:
- ✅ `src/spindle_step_pio.cpp`
- ✅ `src/spindle_step_pio.h`

### Fixed Files:
- ✅ `src/winding_controller.cpp` - Z-index homing fix
- ✅ `CMakeLists.txt` - PIO compilation enabled

---

## What's Fixed

✅ **Z-index homing** - Spindle actually rotates now  
✅ **PIO spindle stepping** - Hardware timing  
✅ **PIO encoder reading** - Hardware quadrature  
⚠️ **Traverse still software** - Can fix if needed

---

## Next Issue to Fix

**Traverse uses software ISR stepping** while spindle uses PIO. This causes timing mismatch during winding.

To fix: Add PIO stepping for traverse axis (uses same `spindle_step.pio` program on different state machine).
