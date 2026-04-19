#pragma once
#include <Arduino.h>
#include "hardware/pwm.h"

#define EMITTER_PIN    14
#define RECEIVER_PIN   11
#define MAX_NIBBLES    64

#define START_MARK_US  1000
#define DATA_MARK_US    500
#define GAP_BASE_US     500
#define GAP_STEP_US     300
#define STOP_US        6000
#define MARK_TOL        250   // ±µs

struct ir_packet_t {
    uint8_t nibbles[MAX_NIBBLES];
    uint8_t count    = 0;
    bool    complete = false;
    bool    error    = false;
};

void phy_init();

// TX
void phy_send_nibble(uint8_t nibble);
void phy_send_byte(uint8_t b);
void phy_send_raw(const uint8_t *bytes, uint8_t len);

// RX
// phy_update() é no-op — stop-silence detectado por mbed::Ticker HW.
// Mantida na API para compatibilidade.
void phy_update();

bool phy_packet_ready(ir_packet_t *out);