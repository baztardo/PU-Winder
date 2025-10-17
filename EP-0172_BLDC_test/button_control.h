// =============================================================================
// button_control.h - Button handler for EP-0172
// Buttons: GP14 (BTN2), GP15 (BTN1)
// =============================================================================

#pragma once

#include <cstdint>
#include "hardware/gpio.h"

typedef void (*ButtonCallback)(void);

class ButtonControl {
public:
    ButtonControl();
    
    void init();
    
    // Register callbacks for button press events
    void register_btn1_callback(ButtonCallback cb);
    void register_btn2_callback(ButtonCallback cb);
    
    // Get button states
    bool btn1_pressed() const { return btn1_state; }
    bool btn2_pressed() const { return btn2_state; }
    
    uint32_t get_btn1_press_count() const { return btn1_press_count; }
    uint32_t get_btn2_press_count() const { return btn2_press_count; }
    
    void reset_counters();

private:
    static constexpr uint BTN1_PIN = 15;  // GP15
    static constexpr uint BTN2_PIN = 14;  // GP14
    
    volatile bool btn1_state;
    volatile bool btn2_state;
    volatile uint32_t btn1_press_count;
    volatile uint32_t btn2_press_count;
    volatile uint32_t btn1_last_press_time;
    volatile uint32_t btn2_last_press_time;
    
    ButtonCallback btn1_callback;
    ButtonCallback btn2_callback;
    
    static void gpio_callback_wrapper(uint gpio, uint32_t events);
    void handle_button_press(uint gpio);
};