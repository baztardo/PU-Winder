// =============================================================================
// winding_controller.h - Winding Process State Machine
// Purpose: High-level winding sequence control and coordination
// =============================================================================

#pragma once

#include "move_queue.h"
#include "encoder.h"
#include "stepcompress.h"
#include "lcd_display.h"
#include <cstdint>

// =============================================================================
// Winding States
// =============================================================================
enum class WindingState {
    IDLE,
    HOMING_SPINDLE,
    HOMING_TRAVERSE,
    MOVING_TO_START,
    RAMPING_UP,
    WINDING,
    RAMPING_DOWN,
    COMPLETE,
    ERROR
};

// =============================================================================
// Winding Parameters
// =============================================================================
struct WindingParams {
    uint32_t target_turns;      // Total number of turns to wind
    float spindle_rpm;           // Target spindle RPM
    float wire_diameter_mm;      // Wire diameter in mm (43 AWG ≈ 0.064mm)
    float layer_width_mm;        // Width of winding area in mm
    float start_position_mm;     // Starting position from home (mm)
    float ramp_time_sec;         // Time for spindle ramp up/down
    
    // Calculated values
    float wire_pitch_mm;         // Pitch = wire diameter (for tight winding)
    uint32_t turns_per_layer;    // Turns that fit in layer_width_mm
    uint32_t total_layers;       // Total layers needed
    
    WindingParams() 
        : target_turns(1000)
        , spindle_rpm(300.0f)
        , wire_diameter_mm(0.064f)  // 43 AWG
        , layer_width_mm(50.0f)
        , start_position_mm(20.0f)
        , ramp_time_sec(3.0f)
        , wire_pitch_mm(0.064f)
        , turns_per_layer(0)
        , total_layers(0) {
        calculate_layers();
    }
    
    void calculate_layers() {
        wire_pitch_mm = wire_diameter_mm;  // Tight winding
        turns_per_layer = (uint32_t)(layer_width_mm / wire_pitch_mm);
        if (turns_per_layer == 0) turns_per_layer = 1;
        total_layers = (target_turns + turns_per_layer - 1) / turns_per_layer;
    }
};

// =============================================================================
// WindingController Class
// =============================================================================
class WindingController {
public:
    /**
     * @brief Constructor
     * @param mq Move queue instance
     * @param enc Encoder instance
     * @param lcd LCD display instance
     */
    WindingController(MoveQueue* mq, Encoder* enc, LCDDisplay* lcd);

    void init();
    void set_parameters(const WindingParams& params);
    bool start();
    void stop();
    void update();
    WindingState get_state() const { return state; }
    uint32_t get_current_layer() const { return current_layer; }
    uint32_t get_turns_completed() const { return turns_completed; }
    float get_current_rpm() const { return current_rpm; }
    void emergency_stop();


private:
    MoveQueue* move_queue;
    Encoder* encoder;
    LCDDisplay* lcd;
    
    WindingState state;
    WindingParams params;
    
    uint32_t current_layer;
    uint32_t turns_completed;
    uint32_t turns_this_layer;
    float current_rpm;
    
    int32_t last_encoder_position;
    uint32_t last_rpm_update_time;
    
    bool traverse_direction;  // true = forward, false = reverse
    float current_traverse_position_mm;

    bool ramp_started = false;
    uint32_t ramp_start_time = 0;
    // encoder/traverse sync state
    double   turn_accum = 0.0;            // total turns (fractional) since start
    int8_t   encoder_sign = 0;            // learned sign of "forward" (+1 or -1)
    double   traverse_steps_emitted = 0.0;// how many traverse steps we have already sent
    // Use two independent cursors so RPM math doesn’t fight sync
    int32_t  enc_last_sync = 0;               // for sync_traverse_to_spindle()
    int32_t  enc_last_rpm  = 0;               // for update_rpm()

    /**
     * @brief Home spindle to Z index
     */
    void home_spindle();
    
    /**
     * @brief Home traverse axis
     */
    void home_traverse();
    
    /**
     * @brief Move traverse to start position
     */
    void move_to_start();
    
    /**
     * @brief Ramp up spindle to target RPM
     */
    void ramp_up_spindle();
    
    /**
     * @brief Execute winding process
     */
    void execute_winding();
    
    /**
     * @brief Ramp down spindle
     */
    void ramp_down_spindle();
    
    /**
     * @brief Update RPM calculation
     */
    void update_rpm();
    
    /**
     * @brief Synchronize traverse to spindle position
     */
    void sync_traverse_to_spindle();
    
    /**
     * @brief Update LCD display
     */
    void update_display();
    
    /**
     * @brief Convert mm to steps
     * @param mm Distance in millimeters
     * @return Steps
     */
    uint32_t mm_to_steps(float mm);
    
    /**
     * @brief Convert steps to mm
     * @param steps Number of steps
     * @return Distance in millimeters
     */
    float steps_to_mm(uint32_t steps);
};