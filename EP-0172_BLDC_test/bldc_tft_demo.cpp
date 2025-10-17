// =============================================================================
// bldc_tft_demo.cpp - BLDC Motor Control with ST7789 TFT Display
// =============================================================================
// Tests BLDC control while displaying status on TFT screen
// =============================================================================

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/uart.h"
#include "st7789_pico.h"
#include <cstdio>
#include <cmath>

// =============================================================================
// Pin Configuration for SKR-Pico
// =============================================================================

// SPI Pins (SPI0)
#define SPI_PORT        spi0
#define SPI_SCK_PIN     18    // GPIO 18 (SPI0 SCK)
#define SPI_MOSI_PIN    19    // GPIO 19 (SPI0 TX)
#define SPI_MISO_PIN    16    // GPIO 16 (SPI0 RX)  - optional
#define SPI_FREQ_HZ     62500000  // 62.5 MHz

// TFT Control Pins
#define TFT_CS_PIN      17    // Chip select
#define TFT_DC_PIN      20    // Data/Command
#define TFT_RST_PIN     21    // Reset

// BLDC Motor Pins (same as before)
#define BLDC_PWM_PIN        2       // PWM output for speed
#define BLDC_PWM_SLICE      1       // PWM slice number
#define BLDC_PWM_CHANNEL    PWM_CHAN_A
#define BLDC_PWM_FREQ_HZ    16000   // 16 kHz PWM frequency

#define BLDC_DIR_PIN        26      // Direction control
#define BLDC_BRAKE_PIN      27      // Brake control

#define BLDC_HALL_A_PIN     3       // Hall A
#define BLDC_HALL_B_PIN     4       // Hall B
#define BLDC_HALL_C_PIN     5       // Hall C

// =============================================================================
// Global Objects
// =============================================================================

ST7789 tft(SPI_PORT,
           SPI_SCK_PIN, SPI_MOSI_PIN, SPI_MISO_PIN,
           TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN,
           240, 320);

// BLDC State
struct {
    uint16_t current_duty;      // 0-1000 (0-100%)
    bool direction;             // true = forward
    bool brake_enabled;
    
    volatile uint32_t hall_edge_count;
    volatile uint32_t last_hall_time_us;
    volatile float measured_rpm;
    
    bool ramping;
    uint16_t target_duty;
    uint16_t ramp_rate;
    uint32_t next_ramp_time;
    
} bldc;

// =============================================================================
// Font Data (Simple 5x7 bitmap font)
// =============================================================================

// Simple ASCII font data (simplified - just numbers and basic chars)
static const uint8_t font5x7[96][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // space
    {0x5F, 0x00, 0x00, 0x00, 0x00}, // !
    {0x03, 0x00, 0x03, 0x00, 0x00}, // "
    {0x14, 0x3E, 0x14, 0x3E, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x1C, 0x22, 0x41, 0x00, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    
    // 0-9
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
};

// =============================================================================
// Simple Text Drawing
// =============================================================================

void draw_char(int x, int y, char c, uint16_t color) {
    if (c < 32 || c > 126) return;
    
    const uint8_t* glyph = font5x7[c - 32];
    
    for (int col = 0; col < 5; col++) {
        uint8_t byte = glyph[col];
        for (int row = 0; row < 8; row++) {
            if (byte & (1 << row)) {
                tft.draw_pixel(x + col, y + row, color);
            }
        }
    }
}

void draw_string(int x, int y, const char* str, uint16_t color) {
    while (*str) {
        draw_char(x, y, *str, color);
        x += 6;  // 5 pixels + 1 space
        str++;
    }
}

// Simplified printf-style for display
void draw_number(int x, int y, int num, uint16_t color) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", num);
    draw_string(x, y, buf, color);
}

// =============================================================================
// BLDC Functions
// =============================================================================

void bldc_init() {
    // PWM Setup
    uint slice = pwm_gpio_to_slice_num(BLDC_PWM_PIN);
    uint channel = pwm_gpio_to_channel(BLDC_PWM_PIN);
    
    gpio_set_function(BLDC_PWM_PIN, GPIO_FUNC_PWM);
    
    uint wrap = 1000;
    float divider = (float)(clock_get_hz(clk_sys)) / (BLDC_PWM_FREQ_HZ * (wrap + 1));
    
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, divider);
    pwm_config_set_wrap(&cfg, wrap);
    
    pwm_init(slice, &cfg);
    pwm_set_chan_level(slice, channel, 0);
    
    // GPIO Setup
    gpio_init(BLDC_DIR_PIN);
    gpio_set_dir(BLDC_DIR_PIN, GPIO_OUT);
    
    gpio_init(BLDC_BRAKE_PIN);
    gpio_set_dir(BLDC_BRAKE_PIN, GPIO_OUT);
    
    // Hall sensor setup (simple GPIO inputs)
    gpio_init(BLDC_HALL_A_PIN);
    gpio_set_dir(BLDC_HALL_A_PIN, GPIO_IN);
    gpio_pull_up(BLDC_HALL_A_PIN);
    
    // Initialize state
    bldc.current_duty = 0;
    bldc.direction = true;
    bldc.brake_enabled = true;
    bldc.hall_edge_count = 0;
    bldc.measured_rpm = 0.0;
    bldc.ramping = false;
    
    printf("[BLDC] Initialized\n");
}

void bldc_set_speed(uint16_t duty) {
    if (duty > 1000) duty = 1000;
    
    uint slice = pwm_gpio_to_slice_num(BLDC_PWM_PIN);
    uint channel = pwm_gpio_to_channel(BLDC_PWM_PIN);
    
    pwm_set_chan_level(slice, channel, duty);
    bldc.current_duty = duty;
}

void bldc_set_direction(bool forward) {
    gpio_put(BLDC_DIR_PIN, forward ? 1 : 0);
    bldc.direction = forward;
}

void bldc_set_brake(bool enable) {
    gpio_put(BLDC_BRAKE_PIN, enable ? 1 : 0);
    bldc.brake_enabled = enable;
}

void bldc_ramp_to(uint16_t target, uint32_t time_ms) {
    bldc.target_duty = target;
    bldc.ramping = true;
    
    uint32_t total_steps = (target > bldc.current_duty) ? 
                           (target - bldc.current_duty) : 
                           (bldc.current_duty - target);
    uint32_t time_50ms_slots = (time_ms + 24) / 50;
    bldc.ramp_rate = (total_steps + time_50ms_slots - 1) / time_50ms_slots;
    bldc.next_ramp_time = time_us_32();
}

void bldc_update() {
    if (!bldc.ramping) return;
    
    uint32_t now = time_us_32();
    if (now - bldc.next_ramp_time < 50000) return;  // 50ms
    
    bldc.next_ramp_time = now;
    
    if (bldc.current_duty < bldc.target_duty) {
        uint16_t new_duty = bldc.current_duty + bldc.ramp_rate;
        if (new_duty > bldc.target_duty) new_duty = bldc.target_duty;
        bldc_set_speed(new_duty);
    } else if (bldc.current_duty > bldc.target_duty) {
        int16_t new_duty = bldc.current_duty - bldc.ramp_rate;
        if (new_duty < (int16_t)bldc.target_duty) new_duty = bldc.target_duty;
        bldc_set_speed((uint16_t)new_duty);
    } else {
        bldc.ramping = false;
    }
}

// =============================================================================
// Display Update
// =============================================================================

void update_display() {
    static uint32_t last_update = 0;
    uint32_t now = time_us_32();
    
    if (now - last_update < 500000) return;  // Update every 500ms
    last_update = now;
    
    // Clear display (draw black background)
    tft.fill(ST7789_BLACK);
    
    // Title
    draw_string(20, 20, "BLDC Test", ST7789_WHITE);
    
    // Speed display
    draw_string(20, 50, "Speed:", ST7789_YELLOW);
    draw_number(80, 50, (bldc.current_duty * 100) / 1000, ST7789_YELLOW);
    draw_string(110, 50, "%", ST7789_YELLOW);
    
    // Direction
    draw_string(20, 70, "Dir:", ST7789_CYAN);
    const char* dir_str = bldc.direction ? "FWD" : "REV";
    draw_string(65, 70, dir_str, ST7789_CYAN);
    
    // Brake status
    draw_string(20, 90, "Brake:", ST7789_RED);
    const char* brake_str = bldc.brake_enabled ? "ON" : "OFF";
    draw_string(75, 90, brake_str, bldc.brake_enabled ? ST7789_RED : ST7789_GREEN);
    
    // RPM (simulated from duty cycle)
    float sim_rpm = (bldc.current_duty / 1000.0f) * 3000.0f;
    draw_string(20, 110, "RPM:", ST7789_GREEN);
    draw_number(75, 110, (int)sim_rpm, ST7789_GREEN);
    
    // Status bar
    uint16_t bar_width = (bldc.current_duty * 200) / 1000;
    tft.fill_rect(20, 140, 20 + bar_width, 155, ST7789_WHITE);
    tft.draw_rect(20, 140, 220, 155, ST7789_WHITE);
    
    // Instructions
    draw_string(20, 180, "Press 's' to start", ST7789_WHITE);
    draw_string(20, 200, "Press 'e' to stop", ST7789_WHITE);
    draw_string(20, 220, "Press 'd' to toggle dir", ST7789_WHITE);
    draw_string(20, 240, "Press 'b' for brake", ST7789_WHITE);
}

// =============================================================================
// Main Program
// =============================================================================

int main() {
    stdio_init_all();
    sleep_ms(1000);
    
    printf("\n" "=" "*60 "\n");
    printf("BLDC Motor Test with ST7789 TFT Display\n");
    printf("SKR-Pico + Pico SDK\n");
    printf("=" "*60 "\n\n");
    
    // Initialize TFT
    printf("Initializing TFT display...\n");
    if (!tft.init(SPI_FREQ_HZ)) {
        printf("ERROR: TFT initialization failed!\n");
        while (true) tight_loop_contents();
    }
    
    printf("TFT initialized successfully\n");
    tft.fill(ST7789_BLACK);
    draw_string(40, 120, "Initializing BLDC...", ST7789_WHITE);
    
    // Initialize BLDC
    printf("Initializing BLDC...\n");
    bldc_init();
    bldc_set_brake(true);  // Start with brake on
    
    printf("\nSystem ready!\n");
    printf("Controls:\n");
    printf("  's' - Start spindle (ramp to 50%)\n");
    printf("  'e' - Stop spindle\n");
    printf("  'd' - Toggle direction\n");
    printf("  'b' - Toggle brake\n");
    printf("  '+' - Increase speed\n");
    printf("  '-' - Decrease speed\n");
    printf("  'q' - Quit\n\n");
    
    uint32_t last_serial_check = 0;
    
    while (true) {
        // Update BLDC ramping
        bldc_update();
        
        // Update display
        update_display();
        
        // Check for serial input
        uint32_t now = time_us_32();
        if (now - last_serial_check > 100000) {  // Check every 100ms
            last_serial_check = now;
            
            if (uart_is_readable(uart0)) {
                char cmd = uart_getc(uart0);
                
                switch (cmd) {
                    case 's':
                        printf("Starting spindle...\n");
                        bldc_set_brake(false);
                        bldc_ramp_to(500, 2000);  // Ramp to 50% over 2 seconds
                        break;
                        
                    case 'e':
                        printf("Stopping spindle...\n");
                        bldc_ramp_to(0, 2000);
                        break;
                        
                    case 'd':
                        bldc_set_direction(!bldc.direction);
                        printf("Direction: %s\n", bldc.direction ? "FORWARD" : "REVERSE");
                        break;
                        
                    case 'b':
                        bldc_set_brake(!bldc.brake_enabled);
                        printf("Brake: %s\n", bldc.brake_enabled ? "ON" : "OFF");
                        break;
                        
                    case '+':
                        if (bldc.current_duty < 1000) {
                            bldc_set_speed(bldc.current_duty + 50);
                            printf("Speed: %u\n", bldc.current_duty);
                        }
                        break;
                        
                    case '-':
                        if (bldc.current_duty > 0) {
                            bldc_set_speed(bldc.current_duty - 50);
                            printf("Speed: %u\n", bldc.current_duty);
                        }
                        break;
                        
                    case 'q':
                        printf("Shutting down...\n");
                        bldc_set_speed(0);
                        bldc_set_brake(true);
                        tft.display_on(false);
                        return 0;
                        
                    default:
                        break;
                }
            }
        }
        
        sleep_ms(10);
    }
    
    return 0;
}