#include "phy.h"

typedef enum {
    STATE_IDLE,
    STATE_START_MARK,
    STATE_GAP,
    STATE_MARK,
} ir_rx_state_t;

static volatile ir_rx_state_t _state          = STATE_IDLE;
static volatile uint32_t      _last_edge_time = 0;
static volatile bool          _packet_ready   = false;

static ir_packet_t _packet;
static ir_packet_t _ready_packet;

static uint8_t  _tx_slice;
static bool     _transmitting = false;

// ── TX helpers ────────────────────────────────────────────────
static void _ir_on()  { pwm_set_enabled(_tx_slice, true);  }
static void _ir_off() { pwm_set_enabled(_tx_slice, false); }

static void _mark(uint32_t us) {
    _ir_on();
    delayMicroseconds(us);
    _ir_off();
}

static void _gap(uint32_t us) {
    delayMicroseconds(us);
}

// ── ISR ───────────────────────────────────────────────────────
static void _on_edge() {
    bool pin     = digitalRead(RECEIVER_PIN);
    uint32_t now = micros();
    uint32_t elapsed = now - _last_edge_time;

    bool falling = (pin == LOW);
    bool rising  = (pin == HIGH);

    if (_transmitting) return;  // crosstalk guard

    switch (_state) {
        case STATE_IDLE:
            if (falling) {
                _last_edge_time = now;
                _state = STATE_START_MARK;
            }
            break;

        case STATE_START_MARK:
            if (rising) {
                if (elapsed >= 750 && elapsed <= 1250) {
                    _packet = ir_packet_t{};
                    _last_edge_time = now;
                    _state = STATE_GAP;
                } else {
                    _state = STATE_IDLE;
                }
            }
            break;

        case STATE_GAP:
            if (falling) {
                if (elapsed >= 375) {
                    if (_packet.count < MAX_NIBBLES) {
                        uint8_t nibble = (elapsed - GAP_BASE_US + GAP_STEP_US / 2) / GAP_STEP_US;
                        if (nibble > 0xF) {
                            _packet.error = true;
                            _state = STATE_IDLE;
                            return;
                        }
                        _packet.nibbles[_packet.count++] = nibble;
                    } else {
                        _packet.error = true;
                    }
                    _last_edge_time = now;
                    _state = STATE_MARK;
                } else {
                    _packet.error = true;
                    _state = STATE_IDLE;
                }
            }
            break;

        case STATE_MARK:
            if (rising) {
                if (elapsed >= 375 && elapsed <= 625) {
                    _last_edge_time = now;
                    _state = STATE_GAP;
                } else {
                    _packet.error = true;
                    _state = STATE_IDLE;
                }
            }
            break;
    }
}

// ── public API ────────────────────────────────────────────────
void phy_init() {
    pinMode(RECEIVER_PIN, INPUT);

    gpio_set_function(EMITTER_PIN, GPIO_FUNC_PWM);
    _tx_slice = pwm_gpio_to_slice_num(EMITTER_PIN);
    pwm_set_wrap(_tx_slice, 3289);
    pwm_set_clkdiv(_tx_slice, 1.0f);
    pwm_set_gpio_level(EMITTER_PIN, 3289 / 2);
    pwm_set_enabled(_tx_slice, false);

    attachInterrupt(digitalPinToInterrupt(RECEIVER_PIN), _on_edge, CHANGE);

}

void phy_send_nibble(uint8_t nibble) {
    _mark(DATA_MARK_US);
    _gap(GAP_BASE_US + nibble * GAP_STEP_US);
}

void phy_send_byte(uint8_t b) {
    phy_send_nibble((b >> 4) & 0x0F);
    phy_send_nibble(b & 0x0F);
}

void phy_send_raw(const uint8_t *bytes, uint8_t len) {
    _transmitting = true;
    noInterrupts();

    _mark(START_MARK_US);
    for (uint8_t i = 0; i < len; i++)
        phy_send_byte(bytes[i]);

    interrupts();
    _transmitting = false;
}

// chama no loop — deteção de stop silence sem depender de edge
void phy_update() {
    if (_state == STATE_GAP || _state == STATE_MARK) {
        if (micros() - _last_edge_time > STOP_US) {
            if (_packet.count > 0 && !_packet.error) {
                _packet.complete = true;
                noInterrupts();
                _ready_packet = _packet;
                _packet_ready = true;
                interrupts();
            }
            _state = STATE_IDLE;
        }
    }
}

bool phy_packet_ready(ir_packet_t *out) {
    if (!_packet_ready) return false;
    noInterrupts();
    *out = _ready_packet;
    _packet_ready = false;
    interrupts();
    return true;
}