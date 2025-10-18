# 🎯 UPLOAD THIS FIRMWARE: v1.2.0

## File to Upload:
**`winder_v1.2.0_DEBUG.uf2`** (141 KB)

---

## What's New in v1.2.0:

### ✅ Version Tracking
You'll now see this on boot:
```
=====================================
  Winder Firmware v1.2.0
  Build: 2025-10-18
  PIO Stepper with Debug
=====================================
```

### ✅ Comprehensive PIO Debug
Every step queue operation now reports:
```
Ramp slice 1: queuing 43 steps at 347.2 sps
  [PIO_QUEUE] 43 steps @ 347.2 sps -> half_cycles=89928
  [PIO_FIFO] TX level=0/4 before push
  [PIO_FIFO] TX level=2/4 after push - SUCCESS!
```

This will tell us:
1. ✅ If `spindle_step_pio_queue_cv()` is being called
2. ✅ The exact timing values being calculated
3. ✅ If the PIO FIFO is accepting data
4. ✅ If there are any errors (NULL ctx, invalid params, etc.)

---

## Expected Output:

### On Boot:
```
=====================================
  Winder Firmware v1.2.0
  Build: 2025-10-18
  PIO Stepper with Debug
=====================================

[PIO] raw=0x80000000 lsb=0 msb=2
[ENC] PIO0 ready. A=3 B=4 base=3 a_bit=1 b_bit=0 sm=0
PIO: Spindle step initialized on PIO0 SM2, GPIO11, offset=18
```

### During Ramp Up:
```
Starting spindle ramp up to 300.0 RPM over 3.0 seconds
Spindle motor enabled, direction: 1
Ramp slice 1: queuing 43 steps at 347.2 sps
  [PIO_QUEUE] 43 steps @ 347.2 sps -> half_cycles=89928
  [PIO_FIFO] TX level=0/4 before push
  [PIO_FIFO] TX level=2/4 after push - SUCCESS!
Ramp slice 2: queuing 53 steps at 428.9 sps
  [PIO_QUEUE] 53 steps @ 428.9 sps -> half_cycles=72833
  [PIO_FIFO] TX level=2/4 before push
  [PIO_FIFO] TX level=4/4 after push - SUCCESS!
...
```

---

## What This Tells Us:

### If PIO Debug Shows "SUCCESS" but Motor Doesn't Turn:
The problem is **hardware-level**, not software:
- ✅ PIO is working
- ✅ Step pulses are being generated on GPIO11
- ❌ TMC2209 not receiving pulses, OR
- ❌ Motor not powered, OR
- ❌ Enable/direction pins wrong

### If PIO Debug Shows Errors:
We'll see exactly what's failing:
- `PIO_QUEUE ERROR: ctx is NULL!` → Initialization problem
- `PIO_QUEUE ERROR: step_count is 0!` → Math error
- `PIO_QUEUE ERROR: steps_per_sec=X invalid!` → Timing calculation error
- FIFO level stuck at 4/4 → PIO state machine hung

---

## Next Steps After Upload:

1. **Upload `winder_v1.2.0_DEBUG.uf2`**
2. **Open serial terminal** (`screen /dev/tty.usbmodem314101 115200`)
3. **Copy and paste ALL output** starting from the version banner
4. **Report back!**

---

## If You Still Don't See New Messages:

**Double-check file timestamp:**
```bash
ls -lh /path/to/file/you/dragged/to/Pico
```

Should show: **Oct 18 11:44** (or later)

If older → You uploaded the wrong file!

---

**This firmware WILL reveal what's happening with the PIO!** 🎯
