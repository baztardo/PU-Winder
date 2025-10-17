// =============================================================================
// bldc_pulse_test_ui_main.cpp - BLDC Speed Pulse Test Program with LCD UI
// Test the speed pulse input on EP-0172 with LCD display and button control
// =============================================================================

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "bldc_speed_pulse.h"
#include "button_control.h"
#include <cstdio>
#include <cstring>
#include "ST7796S_TFT.hpp"

// Create display object
ST7796_TFT lcd(spi0, 2, 3, 4, 5, 6, 7, 10000000);  // CLK, MOSI, MISO, CS, DC, RST, 10MHz

// =============================================================================
// Configuration for EP-0172
// =============================================================================
#define BLDC_PWM_PIN          18    // PWM speed control
#define BLDC_DIR_PIN          19    // Direction control
#define BLDC_BRAKE_PIN        20    // Brake control
#define BLDC_SPEED_PULSE_PIN  21    // Speed pulse input

// =============================================================================
// Global instances
// =============================================================================
BLDCSpeedPulse speed_pulse(BLDC_SPEED_PULSE_PIN);

ButtonControl buttons;

// Test state machine
enum TestState {
    STATE_MENU,
    STATE_PULSE_MONITOR,
    STATE_PWM_RAMP,
    STATE_DIRECTION_TEST,
    STATE_BRAKE_TEST
};

TestState current_state = STATE_MENU;
TestState selected_test = STATE_MENU;
int menu_selection = 0;
bool test_running = false;

// =============================================================================
// Simple BLDC controller for PWM testing
// =============================================================================
class SimpleBLDC {
public:
    SimpleBLDC() : current_duty(0), current_dir(true), brake_on(false) {}
    
    void init() {
        // Configure PWM on GPIO 10 (slice 5, channel A)
        uint slice = pwm_gpio_to_slice_num(BLDC_PWM_PIN);
        uint channel = pwm_gpio_to_channel(BLDC_PWM_PIN);
        
        gpio_set_function(BLDC_PWM_PIN, GPIO_FUNC_PWM);
        
        // PWM config: 16 kHz frequency, 1000 steps resolution
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv(&cfg, 8.0f);
        pwm_config_set_wrap(&cfg, 1000);
        
        pwm_init(slice, &cfg, true);  // Added 'true' to start PWM
        pwm_set_chan_level(slice, channel, 0);
        
        // Configure direction and brake pins
        gpio_init(BLDC_DIR_PIN);
        gpio_set_dir(BLDC_DIR_PIN, GPIO_OUT);
        gpio_put(BLDC_DIR_PIN, 0);
        
        gpio_init(BLDC_BRAKE_PIN);
        gpio_set_dir(BLDC_BRAKE_PIN, GPIO_OUT);
        gpio_put(BLDC_BRAKE_PIN, 0);
        
        printf("[BLDC] Initialized\n");
    }
    
    void set_speed(uint16_t duty_permil) {
        if (duty_permil > 1000) duty_permil = 1000;
        
        uint slice = pwm_gpio_to_slice_num(BLDC_PWM_PIN);
        uint channel = pwm_gpio_to_channel(BLDC_PWM_PIN);
        
        current_duty = duty_permil;
        pwm_set_chan_level(slice, channel, duty_permil);
    }
    
    void set_direction(bool forward) {
        current_dir = forward;
        gpio_put(BLDC_DIR_PIN, forward ? 1 : 0);
    }
    
    void set_brake(bool enable) {
        brake_on = enable;
        gpio_put(BLDC_BRAKE_PIN, enable ? 1 : 0);
    }
    
    uint16_t get_duty() const { return current_duty; }
    bool get_direction() const { return current_dir; }
    bool get_brake() const { return brake_on; }
    
private:
    uint16_t current_duty;
    bool current_dir;
    bool brake_on;
};

SimpleBLDC bldc;
void setup_display() {
    stdio_init_all();
    lcd.init();
    printf("[INIT] Display ready!\n");
}
// =============================================================================
// UI Display Functions
// =============================================================================

void display_menu() {
    printf("\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║  BLDC Speed Pulse Test - EP-0172       ║\n");
    printf("║  LCD Display & Button Control          ║\n");
    printf("╠════════════════════════════════════════╣\n");
    printf("║                                        ║\n");
    
    const char* options[] = {
        "1. Speed Pulse Monitor (30s)",
        "2. PWM Ramp with Feedback",
        "3. Direction Change Test",
        "4. Brake Test",
        "0. Exit"
    };
    
    for (int i = 0; i < 5; i++) {
        printf("║ %c %s\n", (menu_selection == i) ? '>' : ' ', options[i]);
    }
    
    printf("║                                        ║\n");
    printf("║ BTN1: Select | BTN2: Start/Back       ║\n");
    printf("╚════════════════════════════════════════╝\n");
}

void display_pulse_monitor() {
    printf("\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║  Speed Pulse Monitor                   ║\n");
    printf("╠════════════════════════════════════════╣\n");
    printf("║ Pulses:     %8lu                    ║\n", speed_pulse.get_pulse_count());
    printf("║ Revs:       %8.2f                    ║\n", speed_pulse.get_revolutions());
    printf("║ RPM:        %8.1f                    ║\n", speed_pulse.get_rpm());
    printf("║ Frequency:  %8.1f Hz                 ║\n", speed_pulse.get_frequency());
    printf("║                                        ║\n");
    printf("║ BTN2: Stop test                       ║\n");
    printf("╚════════════════════════════════════════╝\n");
}

void display_pwm_ramp() {
    printf("\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║  PWM Ramp with Feedback                ║\n");
    printf("╠════════════════════════════════════════╣\n");
    printf("║ Duty:       %4u/1000 (%3u%%)         ║\n", bldc.get_duty(), bldc.get_duty() / 10);
    printf("║ RPM:        %8.1f                    ║\n", speed_pulse.get_rpm());
    printf("║ Pulses:     %8lu                    ║\n", speed_pulse.get_pulse_count());
    printf("║ Direction:  %s                         ║\n", bldc.get_direction() ? "Forward " : "Reverse");
    printf("║                                        ║\n");
    printf("║ BTN2: Stop test                       ║\n");
    printf("╚════════════════════════════════════════╝\n");
}

void display_direction_test() {
    printf("\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║  Direction Change Test                 ║\n");
    printf("╠════════════════════════════════════════╣\n");
    printf("║ Speed:      600/1000 (60%%)            ║\n");
    printf("║ Direction:  %s                         ║\n", bldc.get_direction() ? "Forward " : "Reverse");
    printf("║ RPM:        %8.1f                    ║\n", speed_pulse.get_rpm());
    printf("║ Pulses:     %8lu                    ║\n", speed_pulse.get_pulse_count());
    printf("║                                        ║\n");
    printf("║ BTN1: Toggle Direction | BTN2: Stop   ║\n");
    printf("╚════════════════════════════════════════╝\n");
}

void display_brake_test() {
    printf("\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║  Brake Test                            ║\n");
    printf("╠════════════════════════════════════════╣\n");
    printf("║ Speed:      %4u/1000                 ║\n", bldc.get_duty());
    printf("║ Brake:      %s                         ║\n", bldc.get_brake() ? "ON " : "OFF");
    printf("║ RPM:        %8.1f                    ║\n", speed_pulse.get_rpm());
    printf("║ Pulses:     %8lu                    ║\n", speed_pulse.get_pulse_count());
    printf("║                                        ║\n");
    printf("║ BTN1: Toggle Brake | BTN2: Stop       ║\n");
    printf("╚════════════════════════════════════════╝\n");
}

// =============================================================================
// Button Callbacks
// =============================================================================

void btn1_callback() {
    printf("[BTN1] Pressed\n");
    
    if (current_state == STATE_MENU) {
        menu_selection = (menu_selection + 1) % 5;
        display_menu();
    } else if (current_state == STATE_DIRECTION_TEST) {
        bldc.set_direction(!bldc.get_direction());
    } else if (current_state == STATE_BRAKE_TEST) {
        bldc.set_brake(!bldc.get_brake());
    }
}

void btn2_callback() {
    printf("[BTN2] Pressed\n");
    
    if (current_state == STATE_MENU) {
        if (menu_selection < 4) {
            selected_test = (TestState)(menu_selection + 1);
            current_state = selected_test;
            test_running = true;
            speed_pulse.reset();
            printf("[MENU] Starting test %d\n", menu_selection + 1);
        } else {
            printf("[MENU] Exit selected\n");
        }
    } else {
        // Stop current test
        test_running = false;
        current_state = STATE_MENU;
        menu_selection = 0;
        bldc.set_speed(0);
        bldc.set_brake(false);
        display_menu();
    }
}

// =============================================================================
// Test Functions
// =============================================================================

void test_pulse_monitor() {
    bldc.set_speed(0);
    bldc.set_brake(false);
    
    uint32_t start = time_us_32();
    uint32_t last_display = start;
    
    while (test_running && (time_us_32() - start) < 30000000) {
        uint32_t now = time_us_32();
        
        if (now - last_display > 500000) {  // Update display every 500ms
            display_pulse_monitor();
            last_display = now;
        }
        
        sleep_ms(10);
    }
    
    test_running = false;
    current_state = STATE_MENU;
    display_menu();
}

void test_pwm_ramp() {
    bldc.set_direction(true);
    bldc.set_brake(false);
    speed_pulse.reset();
    
    uint32_t start = time_us_32();
    uint32_t last_display = start;
    
    while (test_running && (time_us_32() - start) < 60000000) {
        uint32_t elapsed_ms = (time_us_32() - start) / 1000;
        
        // Ramp up for 30s, then ramp down for 30s
        uint16_t target_duty;
        if (elapsed_ms < 30000) {
            target_duty = (uint16_t)((elapsed_ms / 30000.0f) * 1000);
        } else {
            uint32_t elapsed_since_peak = elapsed_ms - 30000;
            target_duty = (uint16_t)(1000 - (elapsed_since_peak / 30000.0f) * 1000);
        }
        
        bldc.set_speed(target_duty);
        
        uint32_t now = time_us_32();
        if (now - last_display > 500000) {
            display_pwm_ramp();
            last_display = now;
        }
        
        sleep_ms(10);
    }
    
    bldc.set_speed(0);
    test_running = false;
    current_state = STATE_MENU;
    display_menu();
}

void test_direction_change() {
    bldc.set_speed(600);
    bldc.set_direction(true);
    bldc.set_brake(false);
    speed_pulse.reset();
    
    uint32_t last_display = time_us_32();
    
    while (test_running) {
        uint32_t now = time_us_32();
        
        if (now - last_display > 500000) {
            display_direction_test();
            last_display = now;
        }
        
        sleep_ms(10);
    }
    
    bldc.set_speed(0);
    test_running = false;
    current_state = STATE_MENU;
    display_menu();
}

void test_brake() {
    bldc.set_direction(true);
    bldc.set_speed(700);
    bldc.set_brake(false);
    speed_pulse.reset();
    
    uint32_t last_display = time_us_32();
    
    while (test_running) {
        uint32_t now = time_us_32();
        
        if (now - last_display > 500000) {
            display_brake_test();
            last_display = now;
        }
        
        sleep_ms(10);
    }
    
    bldc.set_speed(0);
    bldc.set_brake(false);
    test_running = false;
    current_state = STATE_MENU;
    display_menu();
}

// =============================================================================
// Main
// =============================================================================

int main() {
stdio_init_all();
sleep_ms(2000);

printf("\n=== LCD DIAGNOSTIC TEST ===\n");
setup_display();

sleep_ms(2000);

    printf("\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║  BLDC Speed Pulse Test - EP-0172       ║\n");
    printf("║  with LCD Display & Button Control     ║\n");
    printf("║  Pico SDK                              ║\n");
    printf("╚════════════════════════════════════════╝\n\n");
    
    // Initialize hardware
    printf("[INIT] Initializing BLDC controller...\n");
    bldc.init();
    sleep_ms(500);
    
    printf("[INIT] Initializing speed pulse handler...\n");
    speed_pulse.init();
    sleep_ms(500);
    
    printf("[INIT] Initializing LCD display...\n");
    lcd.init();
    sleep_ms(500);
    
    printf("[INIT] Initializing button control...\n");
    buttons.init();
    buttons.register_btn1_callback(btn1_callback);
    buttons.register_btn2_callback(btn2_callback);
    sleep_ms(500);
    
    printf("[INIT] All systems initialized!\n\n");
    
    // Display menu
    current_state = STATE_MENU;
    display_menu();
    
    // Main loop
    while (true) {
        lcd.fillScreen(COLOR_BLACK);
        lcd.fillRect(10, 10, 300, 50, COLOR_BLUE);

        switch (current_state) {
            case STATE_MENU:
                sleep_ms(100);
                break;
                
            case STATE_PULSE_MONITOR:
                test_pulse_monitor();
                break;
                
            case STATE_PWM_RAMP:
                test_pwm_ramp();
                break;
                
            case STATE_DIRECTION_TEST:
                test_direction_change();
                break;
                
            case STATE_BRAKE_TEST:
                test_brake();
                break;
        }
        
        sleep_ms(10);
    }
    
    return 0;
}