// =============================================================================
// Diagnostic Test Code for Encoder & Count Tracking
// Add these tests to main.cpp to verify the fix
// =============================================================================

#include "pico/stdlib.h"
#include <cstdio>

// =============================================================================
// TEST 1: Verify Encoder Position Updates
// =============================================================================
// Run this BEFORE starting winding to verify encoder is working
void test_encoder_updates() {
    printf("\n=== TEST 1: Encoder Position Updates ===\n");
    printf("Manually rotate spindle clockwise slowly...\n");
    
    int32_t last_pos = encoder.get_position();
    uint32_t samples = 0;
    
    for (int i = 0; i < 50; i++) {
        sleep_ms(100);
        int32_t pos = encoder.get_position();
        int32_t delta = pos - last_pos;
        
        printf("[%2d] pos=%5ld  delta=%5ld  rots=%.3f\n", 
               i, (long)pos, (long)delta, (float)pos / ENCODER_CPR);
        
        if (delta != 0) {
            samples++;
        }
        last_pos = pos;
    }
    
    printf("Encoder samples changed: %u/50\n", samples);
    if (samples > 40) {
        printf("✓ PASS: Encoder is updating\n\n");
    } else {
        printf("✗ FAIL: Encoder not responding! Check GPIO 3, 4, 25\n\n");
    }
}

// =============================================================================
// TEST 2: Verify Position Tracking in sync_traverse_to_spindle()
// =============================================================================
// Add this as a modified version during testing
void test_position_tracking() {
    printf("\n=== TEST 2: Position Tracking (Manual Simulation) ===\n");
    
    // Simulate the sequence of encoder positions
    int32_t encoder_positions[] = {
        0,      // Starting position
        100,    // Partial turn
        200,    // More accumulation
        300,
        500,
        700,
        900,
        1100,
        1300,
        1440,   // Complete turn 1
        1540,
        1700,
        2100,
        2800,
        2900,
        3100,
        3200,
        3300,
        3400,
        3500,
    };
    
    int32_t last_encoder_position = 0;
    uint32_t turns_completed = 0;
    
    printf("Simulating encoder->get_position() values:\n");
    printf("Enc_Pos  Delta   New_Turns  Completed  Last_Pos_Updated\n");
    printf("-------  -----   ---------  ---------  ----------------\n");
    
    for (size_t i = 0; i < sizeof(encoder_positions) / sizeof(encoder_positions[0]); i++) {
        int32_t pos = encoder_positions[i];
        int32_t delta = pos - last_encoder_position;
        uint32_t new_turns = (uint32_t)(delta / ENCODER_CPR);
        
        if (new_turns > 0) {
            turns_completed += new_turns;
        }
        
        // ✓ FIXED: Update to current position
        last_encoder_position = pos;
        
        printf("%7ld  %5ld   %9lu  %9lu  %7ld\n",
               (long)pos, (long)delta, (unsigned long)new_turns,
               (unsigned long)turns_completed, (long)last_encoder_position);
    }
    
    printf("\nExpected: turns_completed = 2 (one at 1440, one at ~2880)\n");
    printf("Actual: turns_completed = %u\n", turns_completed);
    if (turns_completed >= 2) {
        printf("✓ PASS: Position tracking works\n\n");
    } else {
        printf("✗ FAIL: Turns not counting correctly\n\n");
    }
}

// =============================================================================
// TEST 3: Live Winding Test with Diagnostics
// =============================================================================
// Modify update_display() temporarily to show diagnostics
void diagnostic_update_display() {
    static uint32_t last_update_ms = 0;
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    
    // Print every 500ms
    if (now_ms - last_update_ms < 500) {
        return;
    }
    last_update_ms = now_ms;

    if (state != WindingState::WINDING) {
        return;
    }

    const int32_t enc_counts = encoder.get_position();
    const float turns_f = (float)enc_counts / (float)ENCODER_CPR;
    
    // Print diagnostic info
    printf("[WIND] enc=%6ld  turns_completed=%4lu  L_enc_pos=%6ld  delta=%6ld\n",
           (long)enc_counts,
           (unsigned long)turns_completed,
           (long)last_encoder_position,
           (long)(enc_counts - (int32_t)last_encoder_position));
    
    // Also update LCD as before
    lcd->printf_at(0, 0, "Turns:%4lu/%4lu", turns_completed, params.target_turns);
    lcd->printf_at(0, 1, "RPM:%.0f", current_rpm);
    lcd->printf_at(0, 2, "Layer:%lu/%lu", current_layer, params.total_layers);
    lcd->printf_at(0, 3, "Enc:%ld", (long)enc_counts);
}

// =============================================================================
// TEST 4: Verify Encoder Direction
// =============================================================================
void test_encoder_direction() {
    printf("\n=== TEST 4: Encoder Direction ===\n");
    printf("Rotate spindle CLOCKWISE. Position should INCREASE.\n");
    printf("If position DECREASES, set ENCODER_INVERT = 1 in config.h\n\n");
    
    int32_t start_pos = encoder.get_position();
    printf("Starting position: %ld\n", (long)start_pos);
    printf("Rotate now...\n");
    sleep_ms(3000);
    
    int32_t end_pos = encoder.get_position();
    printf("Ending position: %ld\n", (long)end_pos);
    
    int32_t delta = end_pos - start_pos;
    printf("Delta: %ld\n\n", (long)delta);
    
    if (delta > 100) {
        printf("✓ PASS: Clockwise rotation increases position (correct)\n\n");
    } else if (delta < -100) {
        printf("✗ FAIL: Clockwise rotation decreases position\n");
        printf("  → Set ENCODER_INVERT = 1 in config.h\n\n");
    } else {
        printf("? UNCLEAR: Small movement. Rotate more next time\n\n");
    }
}

// =============================================================================
// TEST 5: ISR and Scheduler Verification
// =============================================================================
void test_isr_running() {
    printf("\n=== TEST 5: ISR Running ===\n");
    
    if (!scheduler.is_running()) {
        printf("✗ FAIL: Scheduler ISR not running!\n");
        printf("  → Check scheduler.start() in main.cpp\n\n");
        return;
    }
    
    uint32_t tick_start = scheduler.get_tick_count();
    uint32_t freq = scheduler.get_frequency_hz();
    
    printf("Scheduler running: YES\n");
    printf("ISR frequency: %lu Hz\n", (unsigned long)freq);
    printf("Expected: 10000 Hz (or whatever HEARTBEAT_US is set to)\n");
    
    sleep_ms(1000);
    
    uint32_t tick_end = scheduler.get_tick_count();
    uint32_t ticks = tick_end - tick_start;
    
    printf("Ticks in 1 second: %lu\n", (unsigned long)ticks);
    printf("Expected: %lu (approximately)\n\n", (unsigned long)freq);
    
    if (ticks > (freq * 0.9) && ticks < (freq * 1.1)) {
        printf("✓ PASS: ISR frequency is correct\n\n");
    } else {
        printf("✗ FAIL: ISR frequency is wrong\n");
        printf("  → Check timer configuration\n\n");
    }
}

// =============================================================================
// MAIN DIAGNOSTIC SUITE
// =============================================================================
// Call this from main.cpp before winding_controller.start()
void run_diagnostic_suite() {
    printf("\n\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║        WIRE WINDER DIAGNOSTIC TEST SUITE                  ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");
    
    test_isr_running();
    test_encoder_updates();
    test_encoder_direction();
    test_position_tracking();
    
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║             DIAGNOSTICS COMPLETE                          ║\n");
    printf("║      Ready to start winding sequence. Press START.        ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n\n");
    
    sleep_ms(5000);
}

// =============================================================================
// USAGE IN main.cpp
// =============================================================================
/*

In main.cpp, add before auto-start:

    setup_winding_parameters();
    
    // ↓ ADD THIS LINE FOR DIAGNOSTICS ↓
    run_diagnostic_suite();
    // ↑ REMOVE AFTER TESTING ↑
    
    if (winding_controller.start()) {
        // ...
    }

Or for minimal testing, just add:

    test_encoder_direction();      // 3-5 seconds
    test_isr_running();            // 1 second

*/

