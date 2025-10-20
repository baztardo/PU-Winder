// =============================================================================
// button_control.cpp - Button handler implementation
// =============================================================================

#include "button_control.h"
#include "pico/stdlib.h"
#include <cstdio>

static ButtonControl* g_button_instance = nullptr;

ButtonControl::ButtonControl()
    : btn1_state(false)
    , btn2_state(false)
    , btn1_press_count(0)
    , btn2_press_count(0)
    , btn1_last_press_time(0)
    , btn2_last_press_time(0)
    , btn1_callback(nullptr)
    , btn2_callback(nullptr)
{
    g_button_instance = this;
}

void ButtonControl::init() {
    // Configure BTN1 (GP15)
    gpio_init(BTN1_PIN);
    gpio_set_dir(BTN1_PIN, GPIO_IN);
    gpio_pull_up(BTN1_PIN);
    
    // Configure BTN2 (GP14)
    gpio_init(BTN2_PIN);
    gpio_set_dir(BTN2_PIN, GPIO_IN);
    gpio_pull_up(BTN2_PIN);
    
    // Enable interrupts on falling edge (button press)
    gpio_set_irq_enabled_with_callback(
        BTN1_PIN,
        GPIO_IRQ_EDGE_FALL,
        true,
        &ButtonControl::gpio_callback_wrapper
    );
    
    gpio_set_irq_enabled(BTN2_PIN, GPIO_IRQ_EDGE_FALL, true);
    
    printf("[BUTTONS] Initialized BTN1(GP15) and BTN2(GP14)\n");
}

void ButtonControl::register_btn1_callback(ButtonCallback cb) {
    btn1_callback = cb;
}

void ButtonControl::register_btn2_callback(ButtonCallback cb) {
    btn2_callback = cb;
}

void ButtonControl::reset_counters() {
    btn1_press_count = 0;
    btn2_press_count = 0;
}

void ButtonControl::gpio_callback_wrapper(uint gpio, uint32_t events) {
    if (g_button_instance) {
        g_button_instance->handle_button_press(gpio);
    }
}

void ButtonControl::handle_button_press(uint gpio) {
    uint32_t now = time_us_32();
    
    if (gpio == BTN1_PIN) {
        // Debounce: ignore presses faster than 100ms
        if (now - btn1_last_press_time < 100000) return;
        
        btn1_last_press_time = now;
        btn1_state = true;
        btn1_press_count++;
        
        printf("[BTN1] Pressed (count: %lu)\n", btn1_press_count);
        
        if (btn1_callback) {
            btn1_callback();
        }
        
    } else if (gpio == BTN2_PIN) {
        // Debounce: ignore presses faster than 100ms
        if (now - btn2_last_press_time < 100000) return;
        
        btn2_last_press_time = now;
        btn2_state = true;
        btn2_press_count++;
        
        printf("[BTN2] Pressed (count: %lu)\n", btn2_press_count);
        
        if (btn2_callback) {
            btn2_callback();
        }
    }
}