// =============================================================================
// encoder.cpp - Quadrature Encoder Implementation
// =============================================================================

#include "encoder.h"
#include "config.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/time.h"
#include "encoder.pio.h"
#include <cstdio>

static volatile uint32_t g_isr_hits = 0;

Encoder::Encoder()
    : position(0)
    , last_velocity_position(0)
    , last_a(false)
    , last_b(false)
    , last_z(false)
    , z_pulse_detected(false) {
}

void Encoder::init() {
    // Configure Z pin for index pulse
    gpio_init(ENCODER_Z_PIN);
    gpio_set_dir(ENCODER_Z_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_Z_PIN);

    // Determine base pin for PIO sampling: PIO reads base and base+1
    // Use base=A so mapping matches test code (A=bit1, B=bit0)
    const uint8_t a_pin = ENCODER_A_PIN;
    const uint8_t b_pin = ENCODER_B_PIN;
    pio_base_pin = a_pin;

    // Bit mapping to match working PIO test: A in bit1, B in bit0
    a_bit_index = 1;
    b_bit_index = 0;

    // Ensure pulls enabled (works with PIO as well)
    gpio_pull_up(a_pin);
    gpio_pull_up(b_pin);

    // If PIO disabled via config, use GPIO polling immediately
    #if defined(ENCODER_USE_PIO) && (ENCODER_USE_PIO==0)
    pio_initialized = false;
    gpio_init(ENCODER_A_PIN);
    gpio_set_dir(ENCODER_A_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_A_PIN);
    gpio_init(ENCODER_B_PIN);
    gpio_set_dir(ENCODER_B_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_B_PIN);
    last_a = gpio_get(ENCODER_A_PIN);
    last_b = gpio_get(ENCODER_B_PIN);
    last_z = gpio_get(ENCODER_Z_PIN);
    return;
    #endif

    // Try to initialize PIO program; if any step fails, fall back to GPIO polling
    pio = pio0;
    bool pio_ok = true;
    uint local_offset = 0;
    if (!pio_can_add_program(pio, &quadrature_encoder_program)) {
        pio_ok = false;
    } else {
        local_offset = pio_add_program(pio, &quadrature_encoder_program);
    }

    int claimed = -1;
    if (pio_ok) {
        claimed = pio_claim_unused_sm(pio, false);
        if (claimed < 0) pio_ok = false;
    }

    if (pio_ok) {
        sm = (uint)claimed;
        offset = local_offset;
        quadrature_encoder_program_init(pio, sm, offset, pio_base_pin);
        pio_initialized = true;

        // Initialize last state from FIFO if available
        last_state_bits = 0;
        if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
            uint32_t data = pio_sm_get(pio, sm);
            uint8_t raw = data & 0x3;
            bool a = (raw >> a_bit_index) & 0x1;
            bool b = (raw >> b_bit_index) & 0x1;
            last_state_bits = (uint8_t)((a << 1) | b);
        }

        last_a = (last_state_bits >> 1) & 1;
        last_b = (last_state_bits & 1);
        last_z = gpio_get(ENCODER_Z_PIN);
        printf("[ENC] PIO0 ready. A=%u B=%u base=%u a_bit=%u b_bit=%u sm=%u\n",
               (unsigned)a_pin, (unsigned)b_pin, (unsigned)pio_base_pin,
               (unsigned)a_bit_index, (unsigned)b_bit_index, (unsigned)sm);
        return;
    }

    // Fallback: Initialize GPIO pins as inputs with pull-ups and use polling
    pio_initialized = false;
    gpio_init(ENCODER_A_PIN);
    gpio_set_dir(ENCODER_A_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_A_PIN);

    gpio_init(ENCODER_B_PIN);
    gpio_set_dir(ENCODER_B_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_B_PIN);

    // Read initial state for polling
    last_a = gpio_get(ENCODER_A_PIN);
    last_b = gpio_get(ENCODER_B_PIN);
    last_z = gpio_get(ENCODER_Z_PIN);
}

void Encoder::update() {
    // Transition table identical to previous implementation
    static const int8_t table[4][4] = {
        {  0, -1,  1,  0 },
        {  1,  0,  0, -1 },
        { -1,  0,  0,  1 },
        {  0,  1, -1,  0 }
    };

    if (pio_initialized) {
        // Drain RX FIFO; apply transitions for each sample (cap per tick)
        int samples_processed = 0;
        while (!pio_sm_is_rx_fifo_empty(pio, sm) && samples_processed++ < 16) {    // cap per tick
            uint32_t data = pio_sm_get(pio, sm);
            // Latest sample is in LSBs (matches test implementation)
            uint8_t raw = (uint8_t)(data & 0x3);
            bool a = (raw >> a_bit_index) & 0x1;
            bool b = (raw >> b_bit_index) & 0x1;

            uint8_t state = (uint8_t)((a << 1) | b);
            uint8_t last_state = (uint8_t)((last_a << 1) | last_b);
            position += table[last_state][state] * (ENCODER_INVERT ? -1 : 1);
            last_a = a;
            last_b = b;
        }
        // Also sample Z (index) and latch rising->low edge
        bool z_now = gpio_get(ENCODER_Z_PIN);
        if (!z_now && last_z) {
            z_pulse_detected = true;
        }
        last_z = z_now;
        isr_hits++;
        return;
    }

    // GPIO polling fallback
    bool a = gpio_get(ENCODER_A_PIN);
    bool b = gpio_get(ENCODER_B_PIN);
    uint8_t state = (uint8_t)((a << 1) | b);
    uint8_t last_state = (uint8_t)((last_a << 1) | last_b);
    position += table[last_state][state] * (ENCODER_INVERT ? -1 : 1);
    last_a = a;
    last_b = b;
    // Also sample Z (index) and latch edge
    bool z_now = gpio_get(ENCODER_Z_PIN);
    if (!z_now && last_z) {
        z_pulse_detected = true;
    }
    last_z = z_now;
    isr_hits++;
}

int32_t Encoder::get_position() const {
    return position;
}

void Encoder::set_position(int32_t pos) {
    position = pos;
}

void Encoder::reset() {
    position = 0;
    last_velocity_position = 0;
}

float Encoder::get_revolutions() const {
    return (float)position / (float)ENCODER_CPR;
}

bool Encoder::check_z_pulse() {
    if (z_pulse_detected) {
        z_pulse_detected = false;
        return true;
    }
    return false;
}

float Encoder::get_velocity(float dt_seconds) {
    if (dt_seconds <= 0.0f) return 0.0f;
    
    int32_t delta = position - last_velocity_position;
    last_velocity_position = position;
    
    return (float)delta / dt_seconds;
}

void Encoder::debug_status() const {
    printf("Encoder position: %ld  ISR hits: %lu\n",
           (long)position, (unsigned long)get_isr_hits());
}
uint32_t Encoder::get_isr_hits() const {
    return isr_hits;
}
// No global instance; main owns Encoder instances