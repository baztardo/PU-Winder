# Wire Winder Firmware - Execution Flowchart

## 1. STARTUP & INITIALIZATION FLOW

```
┌─────────────────────────────────┐
│   main() Starts                 │
│ (stdio_init_all, sleep 100ms)   │
└──────────────┬──────────────────┘
               │
               ▼
        ┌──────────────────┐
        │  init_hardware() │
        └────────┬─────────┘
                 │
        ┌────────┴────────┬──────────────┐
        ▼                 ▼              ▼
   ┌─────────┐    ┌─────────────┐  ┌──────────┐
   │MoveQueue│    │  I2C Setup  │  │LCD Init  │
   │.init()  │    │ (GPIO 0,1)  │  │(0x27)    │
   └────┬────┘    └─────────────┘  └──────────┘
        │
        └─► GPIO_OUT on:
            • SPINDLE_STEP_PIN (11)
            • SPINDLE_DIR_PIN (10)
            • SPINDLE_ENA_PIN (12) = 0 (enabled)
            • TRAVERSE_STEP_PIN (6)
            • TRAVERSE_DIR_PIN (5)
            • TRAVERSE_ENA_PIN (7) = 0 (enabled)
               │
               ▼
┌──────────────────────────────────┐
│   LCD.clear()                    │
│   LCD.print("Wire Winder v1.0")  │
│   sleep_ms(500)                  │
└──────────┬───────────────────────┘
           │
           ▼
    ┌──────────────────┐
    │ init_motors()    │
    └────────┬─────────┘
             │
    ┌────────┴─────────┬────────────┐
    ▼                  ▼            ▼
┌────────┐      ┌──────────┐   ┌──────────┐
│TMC2209 │      │TMC2209   │   │Set MST   │
│Spindle │      │Traverse  │   │steps(16) │
│testRead│      │testRead  │   └──────────┘
└────────┘      └──────────┘
    │                │
    └────────┬───────┘
             ▼
    ┌────────────────────┐
    │ set_rms_current()  │
    │ (2800mA spindle)   │
    │ (250mA traverse)   │
    └────────┬───────────┘
             │
             ▼
    ┌────────────────────┐
    │  sleep_ms(1000)    │
    └────────┬───────────┘
             │
             ▼
┌──────────────────────────────────┐
│ Encoder.init()                   │
│ • GPIO 3,4,25 = INPUT            │
│ • Pull-ups enabled               │
│ • PIO1 initialized (or polling)  │
└──────────┬───────────────────────┘
           │
           ▼
┌──────────────────────────────────┐
│ Scheduler.start(HEARTBEAT_US)    │
│ • Initialize repeating timer     │
│ • ISR frequency: 10kHz (100µs)   │
└──────────┬───────────────────────┘
           │
           ▼
┌──────────────────────────────────┐
│ WindingController.init()         │
│ • State = IDLE                   │
│ • Reset counters                 │
└──────────┬───────────────────────┘
           │
           ▼
┌──────────────────────────────────┐
│ setup_winding_parameters()       │
│ • target_turns = 1000            │
│ • spindle_rpm = 300              │
│ • wire_diameter_mm = 0.064       │
│ • layer_width_mm = 50.0          │
│ • Calculate layers (2 layers)    │
└──────────┬───────────────────────┘
           │
           ▼
┌──────────────────────────────────┐
│ LCD Display Parameters           │
│ • Winding Setup                  │
│ • Turns: 1000                    │
│ • Layers: 2                      │
│ • RPM: 300                       │
│ sleep_ms(2000)                   │
└──────────┬───────────────────────┘
           │
           ▼
┌──────────────────────────────────┐
│ LCD: "System Ready"              │
│ "Auto-starting in 3s"            │
│ sleep_ms(3000)                   │
└──────────┬───────────────────────┘
           │
           ▼
┌──────────────────────────────────┐
│ winding_controller.start()       │
│ State → HOMING_SPINDLE           │
└──────────┬───────────────────────┘
           │
           ▼
    ╔═══════════════════════════╗
    ║  MAIN LOOP BEGINS         ║
    ║  while(true) { ...update()│
    ╚═══════════════════════════╝
```

---

## 2. HARDWARE ISR LOOP (Parallel, 10 kHz)

```
┌─────────────────────────────────────┐
│ Scheduler Timer Interrupt           │
│ (Every 100µs)                       │
└─────────────┬───────────────────────┘
              │
              ▼
┌─────────────────────────────────────┐
│ Scheduler::handle_isr()             │
│ tick_count++                        │
└─────────────┬───────────────────────┘
              │
       ┌──────┴──────┬──────────────┐
       ▼             ▼              ▼
┌────────────┐ ┌──────────┐  ┌────────────┐
│Encoder     │ │MoveQueue │  │Heartbeat   │
│.update()   │ │.handle   │  │LED toggle  │
│ Quadrature │ │_isr_tick │  │(500ms)     │
│ decoding   │ └──────┬───┘  └────────────┘
└────────────┘        │
                      │
          ┌───────────┴────────────┐
          ▼                        ▼
    ┌──────────────┐        ┌──────────────┐
    │Spindle Axis  │        │Traverse Axis │
    │axis_isr_     │        │axis_isr_     │
    │handler(0)    │        │handler(1)    │
    └──────┬───────┘        └──────┬───────┘
           │                       │
      ┌────┴──────────────────┬────┘
      ▼                       ▼
   [Check if chunk done] [Check if chunk done]
      │                       │
      NO ─┐                ┌─ NO
         │                │
         ▼                ▼
   [Pop next chunk]  [Pop next chunk]
   from queue        from queue
      │                  │
      └────────┬─────────┘
               ▼
   ┌───────────────────────────┐
   │ Check if time for step:   │
   │ if (now - last_time) >=   │
   │    interval_us            │
   └───────┬───────────────────┘
           │
          YES
           │
           ▼
   ┌───────────────────────────┐
   │ Execute Step Pulse:       │
   │ • GPIO_PUT step pin = 1   │
   │ • busy_wait_us(2)         │
   │ • GPIO_PUT step pin = 0   │
   │ • last_step_time = now    │
   │ • step_count++            │
   └───────┬───────────────────┘
           │
           ▼
   ┌───────────────────────────┐
   │ Update Interval:          │
   │ interval +=  add_us       │
   │ count--                   │
   └───────┬───────────────────┘
           │
           ▼
   ┌───────────────────────────┐
   │ Check if chunk done:      │
   │ if (count == 0)           │
   │   active_running = false  │
   └───────────────────────────┘

[Returns to main loop]
```

---

## 3. MAIN LOOP WINDING STATE MACHINE

```
┌────────────────────────────────────────────┐
│ while(true) in main()                      │
│ Call: winding_controller.update()          │
│ Call: sleep_ms(10)                         │
└────────────┬───────────────────────────────┘
             │
             ▼
     ┌───────────────────┐
     │ update_rpm()      │
     │ Calc RPM from     │
     │ encoder every     │
     │ 500ms             │
     └───────┬───────────┘
             │
             ▼
  ╔══════════════════════════════════╗
  ║  STATE MACHINE SWITCH            ║
  ╚══════════════════════════════════╝
             │
    ┌────────┼────────┬──────┬──────┬─────┐
    │        │        │      │      │     │
    ▼        ▼        ▼      ▼      ▼     ▼
  IDLE  HOMING  HOMING MOVING RAMPING WINDING
        SPINDLE TRAVERSE START   UP


═══════════════════════════════════════════════════════════════════════════════
│ STATE: HOMING_SPINDLE                                                       │
═══════════════════════════════════════════════════════════════════════════════

┌────────────────────────────────────────┐
│ home_spindle()                         │
└────────┬───────────────────────────────┘
         │
         ▼
    ┌─────────────────┐
    │ First call?     │ ← Initialize variables
    │ waiting_for_z   │   Move spindle at 200 sps
    │    = true       │   1 revolution (3200 steps)
    └────────┬────────┘
             │
    ┌────────┴────────────────┐
    │                         │
    ▼                         ▼
┌─────────────┐        ┌──────────────┐
│ Check Z     │        │ Timeout after│
│ pulse:      │        │ 10 seconds?  │
│             │        └──────┬───────┘
│ encoder.    │               │
│check_z_     │              YES
│pulse()      │               │
└────┬────────┘               ▼
     │                   ┌──────────┐
    YES                  │ ERROR    │
     │                   │ state    │
     ▼                   └──────────┘
┌──────────────────┐
│ Z Found!         │
│ encoder.reset()  │
│ Clear queue      │
│ State →          │
│HOMING_TRAVERSE   │
└──────────────────┘


═══════════════════════════════════════════════════════════════════════════════
│ STATE: HOMING_TRAVERSE                                                      │
═══════════════════════════════════════════════════════════════════════════════

┌──────────────────────────────────────┐
│ home_traverse() Sub-State Machine    │
└────────┬─────────────────────────────┘
         │
    ┌────┴────┬────────┬──────┐
    ▼         ▼        ▼      ▼
  INIT    MOVING   BACKING  DONE
          TO       OFF
          SWITCH

INIT:
  ▼
  Initialize GPIO 14 (home switch)
  Set direction = false (toward home)
  Sub-state → MOVING_TO_SWITCH

MOVING_TO_SWITCH:
  ▼
  ┌─────────────────────────┐
  │ Home switch triggered?  │
  │ gpio_get(GPIO14) == 0   │
  │ (active low)            │
  └────┬────────────────────┘
       │
       ├─ NO: Queue more steps (1000 steps at 1500 sps)
       │
       └─ YES:
           Queue back-off move (2mm at 1500 sps)
           Set direction = true (away from switch)
           Sub-state → BACKING_OFF

BACKING_OFF:
  ▼
  ┌────────────────────────────┐
  │ Move complete?             │
  │ !is_active && !has_chunk   │
  └────┬───────────────────────┘
       │
       └─ YES:
           current_traverse_pos = 0mm
           Sub-state → DONE
           State → MOVING_TO_START

═══════════════════════════════════════════════════════════════════════════════
│ STATE: MOVING_TO_START                                                      │
═══════════════════════════════════════════════════════════════════════════════

┌──────────────────────────────────────┐
│ move_to_start()                      │
└────────┬─────────────────────────────┘
         │
┌────────┴────────┐
│ First call?     │
│ move_queued     │
│ = false         │
└────┬───────────┐
     │ YES       │ NO
     ▼           │
  Convert start position to steps
  Compress with trapezoid profile
  Queue all chunks
  move_queued = true
              │
              └──────┐
                     ▼
             ┌────────────────┐
             │ Move complete? │
             │ !is_active &&  │
             │ !has_chunk     │
             └────┬───────────┘
                  │
                  └─ YES:
                     current_traverse_pos = start_pos_mm
                     State → RAMPING_UP

═══════════════════════════════════════════════════════════════════════════════
│ STATE: RAMPING_UP                                                           │
═══════════════════════════════════════════════════════════════════════════════

┌──────────────────────────────────────┐
│ ramp_up_spindle()                    │
└────────┬─────────────────────────────┘
         │
    ┌────┴────────────────┐
    │ First call?         │
    │ ramp_started = F    │
    └────┬───────────────┐
         │ YES           │ NO
         ▼               │
  Calculate target SPS:
    RPM / 60 × steps_per_rev
    (300 RPM × 6400 = 32,000 SPS)
                 │
  Divide ramp into 24 slices
  Each slice = ramp_time / 24
                 │
  For each slice (i = 1..24):
    frac = i / 24
    v = v_min + (v_target - v_min) × frac²
    steps = v × slice_time
    Queue constant velocity chunk
                 │
  ramp_started = true
                 │
                 └────┐
                      ▼
        ┌──────────────────────────┐
        │ Ramp complete?           │
        │ elapsed_time >=          │
        │   ramp_time              │
        │ AND queue_depth < 3      │
        └────┬─────────────────────┘
             │
             └─ YES:
                Prefill spindle queue with 1s of motion
                State → WINDING
                ramp_started = false

═══════════════════════════════════════════════════════════════════════════════
│ STATE: WINDING                                                              │
═══════════════════════════════════════════════════════════════════════════════

┌──────────────────────────────────────┐
│ execute_winding()                    │
└────────┬─────────────────────────────┘
         │
         ▼
  ┌────────────────────────────┐
  │ Keep Spindle Running       │
  │ Check queue_depth < 20     │
  └────┬───────────────────────┘
       │
       └─ YES:
           Calculate steps for 1s more motion
           Queue constant velocity chunk

         │
         ▼
  ┌────────────────────────────┐
  │ sync_traverse_to_spindle() │
  └────┬───────────────────────┘
       │
       ▼
  Get current encoder position
  delta = pos - last_encoder_pos
       │
       ├─ delta <= 0: No progress, exit
       │
       └─ delta > 0:
           │
           ▼
           new_turns = delta / ENCODER_CPR
           │
           ├─ new_turns == 0: Not a full rev, exit
           │
           └─ new_turns > 0:
               │
               turns_completed += new_turns
               turns_this_layer += new_turns
               last_encoder_pos += (new_turns × CPR)
               │
               ▼
               ┌──────────────────────┐
               │ Layer complete?      │
               │ turns_this_layer >=  │
               │ turns_per_layer      │
               └────┬─────────────────┘
                    │
                    └─ YES:
                        current_layer++
                        turns_this_layer = 0
                        traverse_direction = !direction (toggle)
                    │
                    ▼
               Calculate traverse motion:
               traverse_mm = new_turns × wire_pitch
               traverse_steps = mm_to_steps(traverse_mm)
               │
               ▼
               Get spindle RPM
               spindle_rps = current_rpm / 60
               traverse_mmps = spindle_rps × wire_pitch
               traverse_sps = traverse_mmps × steps_per_mm
               │
               ▼
               Set direction
               Queue constant velocity traverse chunks
               │
               ▼
  ┌──────────────────────────┐
  │ Check completion:        │
  │ turns_completed >=       │
  │ target_turns             │
  └────┬─────────────────────┘
       │
       └─ YES: State → RAMPING_DOWN

═══════════════════════════════════════════════════════════════════════════════
│ STATE: RAMPING_DOWN                                                         │
═══════════════════════════════════════════════════════════════════════════════

┌──────────────────────────────────────┐
│ ramp_down_spindle()                  │
└────────┬─────────────────────────────┘
         │
    ┌────┴────────────────┐
    │ First call?         │
    │ ramp_started = F    │
    └────┬───────────────┐
         │ YES           │ NO
         ▼               │
  Clear all queues
  Calculate trapezoid ramp down
    start_v = current_rpm_sps
    cruise_v = 0
    accel = negative (deceleration)
  Queue ramp-down chunks
  ramp_started = true
                 │
                 └────┐
                      ▼
        ┌──────────────────────────┐
        │ Motor stopped?           │
        │ !is_active &&            │
        │ !has_chunk               │
        └────┬─────────────────────┘
             │
             └─ YES:
                State → COMPLETE
                ramp_started = false

═══════════════════════════════════════════════════════════════════════════════
│ STATE: COMPLETE                                                             │
═══════════════════════════════════════════════════════════════════════════════

Display completion message on LCD:
  "Winding Complete!"
  "Turns: 1000"
  "Layers: 2"

State → IDLE
```

---

## 4. ENCODER POSITION SYNC FLOW

```
┌─────────────────────────────────────────┐
│ ISR: encoder.update() (every 100µs)     │
└──────────────┬──────────────────────────┘
               │
        ┌──────┴──────┐
        │ PIO or GPIO │
        │ read A, B   │
        └──────┬──────┘
               │
               ▼
        ┌──────────────────┐
        │ Quadrature       │
        │ Transition Table │
        │                  │
        │ last_state →     │
        │ current_state    │
        │ = Δposition      │
        └────────┬─────────┘
                 │
                 ├─ (0→1) = +1 count
                 ├─ (1→3) = +1 count
                 ├─ (3→2) = +1 count
                 ├─ (2→0) = +1 count
                 │
                 ├─ (0→2) = -1 count
                 ├─ (1→0) = -1 count
                 ├─ (3→1) = -1 count
                 ├─ (2→3) = -1 count
                 │
                 └─ other = 0 (noise)
                    │
                    ▼
            position += delta
            isr_hits++
                    │
                    ▼
            ┌──────────────────┐
            │ Check Z pulse    │
            │ (Index)          │
            │ Rising → Falling │
            │ edge detection   │
            └──────┬───────────┘
                   │
                   ▼
            z_pulse_detected = true

┌─────────────────────────────────────────┐
│ Main Loop: update_rpm()                 │
│ (Called every 500ms typically)          │
└──────────────┬──────────────────────────┘
               │
               ▼
        ┌──────────────────┐
        │ Get encoder pos  │
        │ delta = pos -    │
        │   last_rpm_pos   │
        └────────┬─────────┘
                 │
                 ▼
        ┌──────────────────┐
        │ Calculate RPM    │
        │ turns = delta/   │
        │   ENCODER_CPR    │
        │ time_s = dt_us/  │
        │   1e6            │
        │ rps = turns/     │
        │   time_s         │
        │ rpm = rps × 60   │
        └────────┬─────────┘
                 │
                 ▼
        current_rpm = rpm
        (Used for traverse sync)
```

---

## 5. MOVE QUEUE EXECUTION TIMELINE

```
Time 0:        Main thread
               StepCompressor::compress_trapezoid(3200 steps, ...)
                    │
                    ▼
               Returns ~32 StepChunk objects:
               ┌─────────────────┐
               │ Chunk 0: {      │
               │  interval: 8000 │ (start slow)
               │  add: -50       │ (accelerate)
               │  count: 50      │
               │ }               │
               ├─────────────────┤
               │ Chunk 1: {      │
               │  interval: 7950 │
               │  add: -50       │
               │  count: 50      │
               │ }               │
               ├─────────────────┤
               │  ...            │
               │ (accel phase)   │
               ├─────────────────┤
               │ Chunk 16: {     │
               │  interval: 2000 │ (cruise)
               │  add: 0         │ (constant speed)
               │  count: 100     │
               │ }               │
               ├─────────────────┤
               │  ...            │
               │ (cruise phase)  │
               ├─────────────────┤
               │ Chunk 30: {     │
               │  interval: 7950 │ (slow down)
               │  add: +50       │ (decelerate)
               │  count: 50      │
               │ }               │
               └─────────────────┘
                    │
                    ▼
               push_chunk(AXIS_SPINDLE, chunk0)
               push_chunk(AXIS_SPINDLE, chunk1)
               ...
               (Add all 32 chunks to queue)
                    │
                    ▼
               Queue state:
               ┌────────────────────────────┐
               │ queues[SPINDLE][]          │
               │ [0]  = {8000, -50, 50}     │
               │ [1]  = {7950, -50, 50}     │
               │ ...                        │
               │ [31] = {2000,  0,100}      │
               │ head = 32, tail = 0        │
               └────────────────────────────┘


ISR Time 0µs:  axis_isr_handler(AXIS_SPINDLE)
               │
               active_running[SPINDLE] = false
               │
               ▼ Pop chunk from queue
               active[SPINDLE] = {8000, -50, 50}
               tail = 1
               active_running = true
               last_step_time = 0


ISR Time 100µs:  axis_isr_handler(AXIS_SPINDLE)
                 │
                 active_running = true
                 │
                 now = 100
                 time_diff = 100 - 0 = 100
                 │
                 ├─ interval_us = 8000
                 ├─ time_diff (100) < interval (8000)
                 │
                 └─ Too soon, no step yet


ISR Time 8000µs:  axis_isr_handler(AXIS_SPINDLE)
                  │
                  now = 8000
                  time_diff = 8000 - 0 = 8000
                  │
                  ├─ 8000 >= 8000 ✓
                  │
                  ▼ Execute step
                  gpio_put(SPINDLE_STEP_PIN, 1)
                  busy_wait_us(2)
                  gpio_put(SPINDLE_STEP_PIN, 0)
                  last_step_time = 8000
                  step_count++
                  │
                  ▼ Update interval
                  interval = 8000 + (-50) = 7950
                  count = 50 - 1 = 49
                  │
                  ▼ count != 0, chunk continues


ISR Time 15950µs:  Step 2
                   interval = 7950 + (-50) = 7900
                   count = 48


ISR Time ~100ms:  Chunk 0 done (50 steps × 7975µs avg ≈ 400ms)
                  count = 0, active_running = false
                  │
                  ▼ Pop next chunk
                  active[SPINDLE] = Chunk 1
                  tail = 2
                  active_running = true
                  ...continue stepping...


Queue Depth Monitor:
┌────────────────────────────────────────┐
│ Initially:  head=32, tail=0            │
│ queue_depth = 32 chunks                │
│                                        │
│ After ISR processes chunk 0:           │
│ head=32, tail=1                        │
│ queue_depth = 31 chunks                │
│                                        │
│ Every 400ms, another chunk consumed    │
│ Rate: ~1 chunk per 400ms               │
│ Refill rate: Main thread adds more     │
│            (depends on frequency)      │
│                                        │
│ Risk: If main thread too slow,         │
│       queue_depth → 0 = starvation     │
└────────────────────────────────────────┘
```

---

## 6. TRAVERSE SYNCHRONIZATION DETAIL

```
Main Loop Update (runs ~100 Hz)

  sync_traverse_to_spindle()
       │
       ▼
  pos = encoder.get_position()
  delta = pos - last_encoder_position
       │
       ├─ delta ≤ 0: No progress yet, EXIT
       │
       └─ delta > 0:
           │
           ▼
  new_turns = delta / ENCODER_CPR
           │
           ├─ new_turns == 0: Fractional turn, EXIT
           │           (Wait until 1+ full turns)
           │
           └─ new_turns > 0:
               │
               ▼
         Example: new_turns = 3
         (Spindle made 3 complete revolutions)
               │
               ▼
         turns_completed += 3
         turns_this_layer += 3
         last_encoder_pos += 3 × ENCODER_CPR
               │
               ▼
         Check layer:
         if (turns_this_layer >= turns_per_layer)
           current_layer++
           turns_this_layer = 0
           traverse_direction = !traverse_direction
                 │
                 └─ Zig-zag: Forward layer 1,
                            Backward layer 2, etc.
               │
               ▼
         Calculate traverse motion:
         traverse_mm = 3 × wire_pitch_mm
                   = 3 × 0.064mm
                   = 0.192mm
               │
               ▼
         traverse_steps = 0.192mm / steps_per_mm
                        = 0.192 / 0.0049 (approx)
                        = 39 steps
               │
               ▼
         Get current spindle speed:
         spindle_rps = current_rpm / 60
         (measured from encoder 500ms average)
               │
               ▼
         Calculate traverse speed:
         traverse_mmps = spindle_rps × wire_pitch
                       = 5 rev/s × 0.064mm
                       = 0.32 mm/s
               │
               ▼
         traverse_sps = 0.32 mm/s × steps_per_mm
                      = 0.32 × 204.8
                      = ~65 steps/sec
               │
               ▼
         Clamp to minimum:
         if (traverse_sps < TRAVERSE_MIN_WINDING_SPEED)
           traverse_sps = TRAVERSE_MIN_WINDING_SPEED (1000)
               │
               ▼
         Queue constant velocity chunk:
         StepCompressor::compress_constant_velocity(
           39,      // steps
           65       // steps/sec
         )
               │
               ▼
         Returns: [{interval: 15385, add: 0, count: 39}]
         (15385µs ≈ 1/65 sec)
               │
               ▼
         move_queue.push_chunk(AXIS_TRAVERSE, chunk)
               │
               ▼
         ISR will execute 39 traverse steps
         over ~600ms (39 / 65 = 0.6s)
               │
               ▼
         Wire moves 0.192mm along bobbin
         (Perfect sync to spindle motion!)
```

---

## 7. COMPLETE TIMING DIAGRAM

```
Wall Clock      Main Loop          ISR (10 kHz)           Encoders      Motors
────────────────────────────────────────────────────────────────────────────────
T=0s           startup            initialize              idle           idle
               init hardware       timer ready
               init motors

T=1s           LCD setup          HEARTBEAT ON            idle           idle
               encoder init
               scheduler start

T=2s           winding params     tick_count=20k          idle           idle
               display params

T=5s           winding.start()    ┌──────────────────┐    idle           idle
               state=HOMING       │ HOMING_SPINDLE   │
                                  │ spindle rotating │
                                  │ slowly 200 sps   │
                                  └──────────────────┘

               update()           encoder counts      enc_pos       spindle
               check_z_pulse()    increasing         goes 0→3200   spins slow
               Z detected!        delta > 0                        1 rev
               state=HOMING_TRAV

T=6s           update()           ┌──────────────────┐    idle           Z trig
               traverse moves     │ HOMING_TRAVERSE  │
               toward home        │ traverse moving  │
               state=MOVING_START │ toward switch    │
                                  └──────────────────┘

                                  switch triggered!
                                  back off 2mm

T=7s           update()           ┌──────────────────┐    idle           moving
               traverse at home   │ MOVING_TO_START  │
               state=RAMPING_UP   │ traverse to 20mm │
                                  └──────────────────┘

T=8s           update()           ┌──────────────────┐    idle           ramping
               ramp_up()          │ RAMPING_UP       │
               24 slices queued   │ spindle speed    │
               state→WINDING      │ increasing       │
               (queue prefilled)  │ 100→300 RPM      │
                                  └──────────────────┘

T=13s          update()           ┌──────────────────┐    spinning       running
               sync spindle       │ WINDING          │    encoder       full speed
               sync traverse      │ active motion    │    counts        spindle &
                                  │ both axes        │    spins up      traverse
                                  └──────────────────┘

               while turns < 1000:
                 read encoder
                 every 3-5 revs:
                   queue traverse
                   motion

               update_rpm()       encoder velocity    rpm=300       continues
               (every 500ms)      calculated

T=600s         turns reach 1000   ┌──────────────────┐    spindle       both axes
               state=RAMPING_DOWN │ RAMPING_DOWN     │    decelerates  slowing
               queue ramp-down    │ spindle speed    │
                                  │ decreasing       │
                                  └──────────────────┘

T=605s         all queues empty   ┌──────────────────┐    idle           idle
               state=COMPLETE     │ COMPLETE         │
               display msg        │ all motors off   │
               state=IDLE         │ tick count done  │
                                  └──────────────────┘

T=606s         main loop waiting  ┌──────────────────┐    idle           idle
               no new commands    │ ISR still runs   │
               (IDLE state)       │ but nothing to   │
                                  │ do               │
                                  └──────────────────┘
```

---

## 8. KEY DECISION POINTS & LOOPS

```
┌─────────────────────────────────────────────────────────┐
│ MAIN DECISIONS IN CONTROL FLOW                          │
└─────────────────────────────────────────────────────────┘

1. Encoder Loop Detection
   ├─ Check: encoder.check_z_pulse()
   ├─ Action: Reset position, state advance
   └─ Frequency: Every ISR (10 kHz)

2. Layer Completion Loop
   ├─ Check: turns_this_layer >= turns_per_layer
   ├─ Action: Toggle traverse direction, increment layer
   └─ Frequency: Every sync (when turns increment)

3. Winding Completion Loop
   ├─ Check: turns_completed >= target_turns
   ├─ Action: Start ramping down
   └─ Frequency: Every sync update

4. Queue Starvation Loop
   ├─ Check: queue_depth < threshold
   ├─ Action: Refill with more chunks
   └─ Frequency: Every winding update (~10 Hz)

5. Traverse Synchronization Loop
   ├─ Check: encoder position delta
   ├─ Action: Calculate & queue traverse motion
   └─ Frequency: Every main loop (10 Hz)

6. ISR Stepping Loop
   ├─ Check: time >= next_step_time
   ├─ Action: Execute step pulse
   └─ Frequency: Variable per chunk (10-5000 steps/sec)

┌─────────────────────────────────────────────────────────┐
│ TIMING CONSTRAINTS                                      │
└─────────────────────────────────────────────────────────┘

ISR Jitter:
  ├─ Period: 100µs ±5µs
  ├─ Tolerance: ±5% acceptable
  └─ Sources: Other interrupts, flash access

Step Accuracy:
  ├─ Pulse width: 2µs (fixed)
  ├─ Interval jitter: <100µs typical
  └─ Over 3200 steps: ~0.1% error

Encoder Lag:
  ├─ Update rate: 10 kHz (100µs)
  ├─ Typical lag: 1-2 ISR cycles
  └─ Impact: Minor (≈1-2 steps)

Queue Refill:
  ├─ Spindle: Must refill before queue empty
  ├─ Traverse: Can be sparse (every 3-5 turns)
  └─ Priority: Keep spindle running!

╔═════════════════════════════════════════════════════════╗
│ CRITICAL PATH (If this breaks, motion stops)           │
╠═════════════════════════════════════════════════════════╣
│                                                         │
│  ISR Timer ──▶ Step Pulses ──▶ Motors Rotate           │
│                 (every 100µs)      (continuous)        │
│                                                         │
│  Main Loop ──▶ Queue Refill ──▶ ISR Fed               │
│   (every 10ms)   (when needed)                         │
│                                                         │
│  Encoder ──▶ Position Sync ──▶ Traverse Motion        │
│  (10 kHz)    (every 1-2s)        (reactive)           │
│                                                         │
╚═════════════════════════════════════════════════════════╝
```

---

## 9. ERROR & RECOVERY PATHS

```
┌─────────────────────────────────────────────────────┐
│ ERROR HANDLING FLOW                                 │
└─────────────────────────────────────────────────────┘

┌─ Z Index Not Found (10s timeout)
│  ├─ HOMING_SPINDLE state
│  ├─ Action: state = ERROR
│  ├─ Display: "Z Index Timeout!"
│  └─ Recovery: Manual reset required

┌─ Queue Underrun (queue empty during winding)
│  ├─ Critical in WINDING state
│  ├─ Action: ISR has nothing to do
│  ├─ Effect: Spindle stops momentarily
│  └─ Recovery: Main loop refills queue (lag ~10ms)

┌─ TMC2209 Communication Failure
│  ├─ init_motors() detects during startup
│  ├─ Display: "Spindle: FAIL" or "Traverse: FAIL"
│  ├─ Action: System continues (but no current control)
│  └─ Recovery: Check UART wiring (GPIO 8, 9)

┌─ Encoder Counting Fails
│  ├─ Effect: RPM = 0, traverse doesn't sync
│  ├─ Symptom: Only spindle moves, no traverse
│  ├─ Cause: Pull-up failure, wire break, etc.
│  └─ Recovery: Check GPIO 3, 4, 25 voltages

┌─ LCD Display Failure
│  ├─ Effect: No visual feedback
│  ├─ Cause: I2C address mismatch or wire fault
│  ├─ Display tries: 0x27, 0x3F
│  └─ Recovery: Adjust I2C_ADDR in config

┌─ Motor Stall (mechanical jam)
│  ├─ Effect: Steps continue but motor stuck
│  ├─ Detection: TMC2209 stallGuard (not implemented)
│  ├─ Current Behavior: System doesn't detect
│  └─ Recovery: Manual stop + inspection

┌─ Memory Overflow (queue full)
│  ├─ Effect: push_chunk() returns false
│  ├─ Cause: ISR too slow (unlikely)
│  ├─ Prevention: Fixed 128-chunk buffers
│  └─ Monitor: Check queue_depth occasionally

Emergency Stop:
  ├─ Action: emergency_stop()
  ├─ Effect: state = ERROR
  ├─ Motors: Disabled (ENA pins = 1)
  ├─ Queues: Cleared
  ├─ Display: "EMERGENCY STOP!"
  └─ Recovery: Power cycle or manual reset
```

---

## 10. SUMMARY: EXECUTION PHASES

```
┌──────────────────────────────────────────────────────────┐
│  PHASE 1: BOOT & INIT (0-5 seconds)                     │
│  ───────────────────────────────────────────────────────  │
│  • Initialize GPIO, I2C, UART, ISR                      │
│  • Load hardware parameters (current, microsteps)       │
│  • Set default winding parameters                       │
│  • Wait for auto-start or user input                    │
│  • Entry State: IDLE                                    │
└──────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────┐
│  PHASE 2: HOMING (5-10 seconds)                         │
│  ───────────────────────────────────────────────────────  │
│  • Spindle rotates slowly until Z pulse                 │
│  • Traverse moves to physical home switch               │
│  • Position references cleared (home = 0)              │
│  • Entry State: HOMING_SPINDLE                          │
│  • Exit State: MOVING_TO_START                          │
└──────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────┐
│  PHASE 3: POSITIONING (10-12 seconds)                  │
│  ───────────────────────────────────────────────────────  │
│  • Traverse moves to start position (20mm)              │
│  • Spindle queues ramp-up chunks (24 slices)           │
│  • Waits for spindle to reach target RPM                │
│  • Entry State: MOVING_TO_START                         │
│  • Exit State: WINDING                                  │
└──────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────┐
│  PHASE 4: WINDING (12s to ~600s depending on turns)   │
│  ───────────────────────────────────────────────────────  │
│  • Spindle runs constant speed (300 RPM)                │
│  • Traverse synchronizes every 3-5 spindle turns       │
│  • Layer transitions auto-reverse traverse direction    │
│  • Turn counter increments continuously                 │
│  • Entry State: WINDING                                 │
│  • Exit Trigger: turns_completed >= 1000                │
│  • Exit State: RAMPING_DOWN                             │
└──────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────┐
│  PHASE 5: SHUTDOWN (last 5 seconds)                    │
│  ───────────────────────────────────────────────────────  │
│  • Spindle decelerates over 3 seconds                   │
│  • Traverse stopped (not ramped)                        │
│  • Both motors disabled when done                       │
│  • Completion message displayed                        │
│  • Entry State: RAMPING_DOWN                            │
│  • Exit State: IDLE                                     │
│  • ISR continues running (but idles)                    │
└──────────────────────────────────────────────────────────┘

PARALLEL ISR THREAD (Always Running):
  ├─ Update encoder position (every 100µs)
  ├─ Consume step chunks (as scheduled)
  ├─ Emit GPIO step pulses (as needed)
  ├─ Update heartbeat LED (every 500ms)
  └─ Return to main loop when done
```

---

## Legend

```
┌───┐
│ A │  = Process/Function
└───┘

   ▼    = Flow downward
   │    = Continuity
   ►    = Input/Output
   
 ┌──┐  = Decision (if/else)
 └──┘
 
[  ]  = State
{ }   = Data structure
***   = Critical section

YES/NO = Branch outcomes
```

---

**End of Flowchart Document**
