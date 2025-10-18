# Critical Logic Bugs Found in Winder Project

## Bug #1: Dead Code After Infinite Loop (main.cpp:222-226)
**Severity:** Medium
**Impact:** Heartbeat LED code never executes

Lines 222-226 are AFTER the `while(true)` loop and will never execute.

## Bug #2: Static Variable Persistence (winding_controller.cpp)
**Severity:** CRITICAL
**Impact:** State machine cannot reset properly

Three functions use static locals that don't reset:
- `home_spindle()` - line 154: `static bool waiting_for_z = true`
- `move_to_start()` - line 268: `static bool move_queued = false`
- `ramp_down_spindle()` - line 452: `static bool ramp_started = false`

If you re-enter these states, they won't execute properly!

## Bug #3: No Queue Overflow Protection
**Severity:** CRITICAL
**Impact:** Queue can overflow, corrupting memory

No checks before `push_chunk()` calls. If queue is full (128 chunks), push_chunk returns false but code ignores it.

## Bug #4: Encoder Only Tracks Forward Motion
**Severity:** HIGH
**Impact:** Reverse spindle rotation not detected

Line 395: `if (delta <= 0) return;` 
This means if encoder goes backwards, turns aren't counted (could be intentional for winding).

## Bug #5: Spindle Queue Refill Race Condition
**Severity:** MEDIUM
**Impact:** Could overfill spindle queue

`execute_winding()` refills spindle queue every call if depth < 10. 
With 10ms main loop, could push 1 second of moves 100 times!

## Bug #6: Multiple Encoder Position Cursors
**Severity:** MEDIUM  
**Impact:** Confusion between RPM calculation and turn counting

- `last_encoder_position` - used for turn counting
- `enc_last_rpm` - used for RPM calculation
- `enc_last_sync` - declared but never used

These can get out of sync.

## Bug #7: No Protection Against Queue Full
**Severity:** HIGH
**Impact:** Silent failure when queue is full

`push_chunk()` returns false when full, but return value is ignored everywhere.

## Bug #8: Missing Error State Exit
**Severity:** MEDIUM
**Impact:** Once in ERROR state, system is permanently stuck

No way to exit ERROR state except power cycle.
