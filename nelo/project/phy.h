#pragma once
#include <Arduino.h>
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "ir_rx.pio.h"

#define RECEIVER_PIN  11
#define EMITTER_PIN   14
#define SERVO_PIN     0

#define MAX_NIBBLES   64

#define START_MARK_US 1000
#define DATA_MARK_US  500
#define GAP_BASE_US   500
#define GAP_STEP_US   300
#define STOP_US       5000

struct ir_packet_t {
    uint8_t nibbles[MAX_NIBBLES];
    uint8_t count    = 0;
    bool    complete = false;
    bool    error    = false;
};

void phy_init();
void phy_update();
void phy_send_raw(const uint8_t *bytes, uint8_t len);
void phy_send_byte(uint8_t b);
void phy_send_nibble(uint8_t nibble);
bool phy_packet_ready(ir_packet_t *out);