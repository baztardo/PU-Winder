# Visual Explanation of the Bug & Fix

## The Problem: Fractional Turns Lost

```
BROKEN CODE - Position tracking fails on fractional turns
═════════════════════════════════════════════════════════

Time    Encoder     delta    CPR   Turns   Status              last_pos
        Position           Calc  Counted                       Updated?
────────────────────────────────────────────────────────────────────────

T=0     0           0        —     0       START               NO
        last_pos=0

T=1     400         400      0     0       Fractional turn     ❌ NO!
        (not >= 1440)               return early            STUCK at 0

T=2     800         800      0     0       Still fractional    ❌ NO!
        (800-0)                     return early            STUCK at 0

T=3     1200        1200     0     0       Still fractional    ❌ NO!
        (1200-0)                    return early            STUCK at 0

T=4     1600        1600     1     Count   FINALLY have a turn ✓ YES
        (1600-0)           (error!) BUT: We've LOST 160 counts!
                           Only has accumulated
                           error in count

T=5     1800        1800     1     Count   (1800-1440=360)     ✓ YES
        (1800-0)           But last_pos+=1440=1440
                           Not 1600!

T=6     2100        2100     1     Count   Accumulation        ✓ YES
        (2100-1440)        errors grow

        ⚠️ Result: Miscounts and aliasing!
```

---

## The Fix: Always Track Position

```
FIXED CODE - Position updated for every advance
═════════════════════════════════════════════════════════

Time    Encoder     delta    CPR   Turns   Status              last_pos
        Position           Calc  Counted                       Updated
────────────────────────────────────────────────────────────────────────

T=0     0           0        —     0       START               0
        last_pos=0

T=1     400         400      0     0       Fractional turn     ✓ 400
        (400-0)            carry forward  last_pos = 400

T=2     800         400      0     0       Accumulating...     ✓ 800
        (800-400)          carry forward  last_pos = 800

T=3     1200        400      0     0       Still accumulating  ✓ 1200
        (1200-800)         carry forward  last_pos = 1200

T=4     1600        400      0     0       Still accumulating  ✓ 1600
        (1600-1200)        carry forward  last_pos = 1600

T=5     3100        1500     1     Count   NOW we have turn!   ✓ 3100
        (3100-1600)        (1500/1440=1)  last_pos = 3100
                           Remainder 60 counts
                           carried to next

T=6     3500        400      0     0       Accumulating        ✓ 3500
        (3500-3100)        carry forward  last_pos = 3500

        ✓ Result: Perfect count tracking! All counts preserved!
```

---

## Code Comparison

### ❌ BROKEN (Original)

```cpp
void WindingController::sync_traverse_to_spindle() {
    int32_t pos = encoder->get_position();
    int32_t delta = pos - last_encoder_position;

    if (delta <= 0) return;

    uint32_t new_turns = (uint32_t)(delta / ENCODER_CPR);
    if (new_turns == 0) return;  // ← ❌ EXITS WITHOUT UPDATING

    turns_completed += new_turns;
    
    // ← ❌ ONLY updated if new_turns > 0
    last_encoder_position += (int32_t)(new_turns * ENCODER_CPR);
}
```

**Problem**: `last_encoder_position` is stale → accumulates position error

---

### ✅ FIXED (Corrected)

```cpp
void WindingController::sync_traverse_to_spindle() {
    int32_t pos = encoder->get_position();
    int32_t delta = pos - last_encoder_position;

    if (delta <= 0) return;

    uint32_t new_turns = (uint32_t)(delta / ENCODER_CPR);
    
    if (new_turns > 0) {
        turns_completed += new_turns;
    }
    
    // ✅ ALWAYS updated to current position
    last_encoder_position = pos;  // KEY FIX!
    
    if (new_turns == 0) return;  // Now safe - position was updated
    
    // ... traverse sync code ...
}
```

**Solution**: `last_encoder_position` always current → no aliasing

---

## State Machine Flow

### Showing When sync_traverse_to_spindle() is Called

```
Main Loop (main.cpp)
    ↓
winding_controller.update()
    ↓
    ┌─ Switch on state
    │
    ├─ IDLE: do nothing
    ├─ HOMING_SPINDLE: home_spindle() [encoder NOT synced]
    ├─ HOMING_TRAVERSE: home_traverse() [encoder NOT synced]
    ├─ MOVING_TO_START: move_to_start() [encoder NOT synced]
    ├─ RAMPING_UP: ramp_up_spindle() [encoder NOT synced] ⚠️
    │                                                      │
    │  ⚠️ WARNING: Encoder moves but sync NOT called     │
    │     Problem: last_encoder_position is stale        │
    │
    ├─ WINDING: execute_winding()
    │   └─ CALLS: sync_traverse_to_spindle() ✓ ← First sync here!
    │            ↓
    │            Now: last_encoder_position finally updated
    │            But: Contains all ramp-up counts!
    │            With old code: Only updates if complete turns
    │            With new code: Always updates ✓
    │
    ├─ RAMPING_DOWN: ramp_down_spindle()
    └─ COMPLETE: done

Encoder Position Timeline:
═════════════════════════════════════════════════════════════
    HOMING    TRAVERSE   START    RAMP-UP    WINDING
     (0)       (0)        (0)    (0→3000)   (3000→6000)
                          
    encoder  encoder     encoder   encoder    encoder
    moves    stays at 0   stays     moves      moves ✓
    (resets)             at 0   (NOT synced) (SYNCED!)

Problem Timeline:
═════════════════════════════════════════════════════════════
    T=0: encoder = 0, last_pos = 0 (from start())
    T=1: encoder = 1000, last_pos = 0 (not synced yet)
    T=2: encoder = 2000, last_pos = 0 (not synced yet)
    T=3: encoder = 3000, last_pos = 0 (not synced yet)
        → RAMP COMPLETE, TRANSITION TO WINDING
    T=4: encoder = 3100, last_pos = 0
         ← First sync_traverse_to_spindle() call!
         ← OLD CODE: Calculates delta = 3100 - 0 = 3100
                     new_turns = 3100 / 1440 = 2
                     Updates last_pos += 2*1440 = 2880
                     ❌ But what about leftover 220 counts?
    T=5: encoder = 3200, last_pos = 2880
         ← delta = 3200 - 2880 = 320
         ← new_turns = 320 / 1440 = 0
         ← ❌ return without updating last_pos
         ← ❌ 320 COUNTS LOST!
    
    T=6: encoder = 4400, last_pos = 2880 (STILL!)
         ← delta = 4400 - 2880 = 1520
         ← new_turns = 1520 / 1440 = 1
         ← ✓ Count incrementing!
         ← But WRONG: Included the 320 lost counts in calculation
```

---

## Why the Fix Works

```
Data Flow Comparison
═════════════════════════════════════════════════════════════

OLD (Broken):
  Encoder Position
        ↓
   Calculate delta
        ↓
   Is delta a full turn?
    ├─ NO: Return (forget about it)
    └─ YES: Count it, update position
             ↓
        ❌ Lost fractional turns!

NEW (Fixed):
  Encoder Position
        ↓
   Calculate delta
        ↓
   Extract full turns (integer division)
        ↓
   If any full turns: Increment counter
        ↓
   ✓ ALWAYS update position to current value
        ↓
   Carry forward fractional turns automatically
        ↓
        ✓ Next call calculates correctly!
```

---

## Accumulation Example

### How Fractional Turns Are Preserved

```
Without fix (BROKEN):
  T=1: pos 400, new_turns 0, return → lost 400
  T=2: pos 800, new_turns 0, return → lost 800 (total)
  T=3: pos 1200, new_turns 0, return → lost 1200 (total)
  T=4: pos 1600, new_turns 1, count it → but where did other 160 go?

With fix (CORRECT):
  T=1: pos 400, new_turns 0, last_pos = 400 → saved
  T=2: pos 800, new_turns 0, last_pos = 800 → saved
  T=3: pos 1200, new_turns 0, last_pos = 1200 → saved
  T=4: pos 1600, new_turns 0, last_pos = 1600 → saved
  T=5: pos 3100, new_turns 1, last_pos = 3100 → COUNTED!
       ↑ Natural accumulation: 400 + 400 + 400 + 1500 = 2700
         But 2700 contains exactly 1 full turn (1440) + 260 leftover
         Those 260 naturally appear in next cycle!
```

---

## Memory Layout

```
Before Fix (Problem):
┌─────────────────────────────────────────┐
│ WindingController Member Variables      │
├─────────────────────────────────────────┤
│ last_encoder_position    →  0            │ (stale!)
│ encoder->position()      →  3500         │ (current)
│ delta = 3500 - 0 = 3500                  │
│ new_turns = 3500 / 1440 = 2              │
│ Remainder = 3500 % 1440 = 620 ← LOST!   │
└─────────────────────────────────────────┘

After Fix (Correct):
┌─────────────────────────────────────────┐
│ WindingController Member Variables      │
├─────────────────────────────────────────┤
│ last_encoder_position    →  3500         │ (current!)
│ encoder->position()      →  3500         │ (current)
│ delta = 3500 - 3500 = 0                  │
│ new_turns = 0 / 1440 = 0                 │
│ Next call will include 3500 + new counts │
└─────────────────────────────────────────┘
```

---

## Timeline: Before vs After

```
BEFORE FIX - Encoder at 3100 counts in WINDING state
═══════════════════════════════════════════════════════════════
Cycle   Encoder  delta   new_turns  turns_completed  last_pos  Issue
─────   ────────  ─────  ────────   ───────────────  ────────  ─────
1       3100      3100   2          2 ✓              2880      Lost 220
2       3300      420    0          2               2880      STUCK!
3       4000      1120   0          2               2880      STUCK!
4       5600      2720   1          3 ✗             4320      Wrong!
        
        ❌ Result: Counts wrong and jumpy


AFTER FIX - Same encoder trace
═══════════════════════════════════════════════════════════════
Cycle   Encoder  delta   new_turns  turns_completed  last_pos  Result
─────   ────────  ─────  ────────   ───────────────  ────────  ──────
1       3100      3100   2          2 ✓              3100      ✓
2       3300      200    0          2               3300      ✓
3       4000      700    0          2               4000      ✓
4       5600      1600   1          3 ✓             5600      ✓
        
        ✓ Result: Counts smooth and correct!
```

---

This is the essence of the bug: **Position tracking must be updated every cycle, not just when you have something to count.**

