#include "phy.h"
#include <mbed.h>

// ── RX state machine ─────────────────────────────────────────
typedef enum {
    STATE_IDLE,
    STATE_START_MARK,
    STATE_GAP,
    STATE_MARK,
} ir_rx_state_t;

static volatile ir_rx_state_t _state        = STATE_IDLE;
static volatile uint32_t      _last_edge_us = 0;
static volatile bool          _packet_ready = false;

static volatile ir_packet_t _packet;
static          ir_packet_t _ready_packet;

// ── TX ───────────────────────────────────────────────────────
static uint8_t       _tx_slice;
static volatile bool _transmitting = false;

// ── Stop-silence ticker ──────────────────────────────────────
// Dispara a cada 500µs numa ISR de hardware Mbed.
// Completamente independente do loop() — funciona mesmo com
// Serial.read() ou delay() a bloquear o core.
#define TICKER_PERIOD_US 500

static mbed::Ticker _stop_ticker;

static void _stop_ticker_cb() {
    if (_transmitting)       return;
    if (_state == STATE_IDLE) return;

    uint32_t silence = micros() - _last_edge_us;

    if (silence >= STOP_US) {
        if (_packet.count > 0 && !_packet.error) {
            _packet.complete = true;
            memcpy((void*)&_ready_packet, (const void*)&_packet, sizeof(ir_packet_t));
            _packet_ready = true;
        }
        // reset para próximo pacote
        _packet.count    = 0;
        _packet.complete = false;
        _packet.error    = false;
        _state = STATE_IDLE;
    }
}

// ── TX helpers ────────────────────────────────────────────────
static inline void _ir_on()  { pwm_set_enabled(_tx_slice, true);  }
static inline void _ir_off() { pwm_set_enabled(_tx_slice, false); }

static void _mark(uint32_t us) { _ir_on();  delayMicroseconds(us); _ir_off(); }
static void _gap (uint32_t us) { delayMicroseconds(us); }

// ── GPIO ISR ─────────────────────────────────────────────────
static void _on_edge() {
    if (_transmitting) return;

    bool     pin     = digitalRead(RECEIVER_PIN);
    uint32_t now     = micros();
    uint32_t elapsed = now - _last_edge_us;
    bool     falling = (pin == LOW);
    bool     rising  = (pin == HIGH);

    switch (_state) {

        case STATE_IDLE:
            if (falling) {
                _last_edge_us = now;
                _state = STATE_START_MARK;
            }
            break;

        case STATE_START_MARK:
            if (rising) {
                if (elapsed >= 750 && elapsed <= 1250) {
                    _packet.count    = 0;
                    _packet.complete = false;
                    _packet.error    = false;
                    _last_edge_us    = now;
                    _state           = STATE_GAP;
                } else {
                    _state = STATE_IDLE;
                }
            }
            break;

        case STATE_GAP:
            if (falling) {
                if (elapsed >= (uint32_t)(GAP_BASE_US - GAP_STEP_US / 2)) {
                    if (_packet.count < MAX_NIBBLES) {
                        uint8_t nibble;
                        if (elapsed < (uint32_t)GAP_BASE_US) {
                            nibble = 0;
                        } else {
                            nibble = (uint8_t)((elapsed - GAP_BASE_US + GAP_STEP_US / 2)
                                               / GAP_STEP_US);
                        }
                        if (nibble > 0xF) {
                            _packet.error = true;
                            _state = STATE_IDLE;
                            return;
                        }
                        _packet.nibbles[_packet.count++] = nibble;
                    } else {
                        _packet.error = true;
                    }
                    _last_edge_us = now;
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
                    _last_edge_us = now;
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
    attachInterrupt(digitalPinToInterrupt(RECEIVER_PIN), _on_edge, CHANGE);

    // Ticker dispara a cada 500µs — detecta stop-silence sem polling
    _stop_ticker.attach_us(_stop_ticker_cb, TICKER_PERIOD_US);

    // PWM 38 kHz: 125 MHz / 3290 / 1.0 ≈ 38 kHz
    gpio_set_function(EMITTER_PIN, GPIO_FUNC_PWM);
    _tx_slice = pwm_gpio_to_slice_num(EMITTER_PIN);
    pwm_set_wrap(_tx_slice, 3289);
    pwm_set_clkdiv(_tx_slice, 1.0f);
    pwm_set_gpio_level(EMITTER_PIN, 3289 / 2);
    pwm_set_enabled(_tx_slice, false);
}

// ── TX ────────────────────────────────────────────────────────

void phy_send_nibble(uint8_t nibble) {
    _gap(GAP_BASE_US + (uint32_t)nibble * GAP_STEP_US);
    _mark(DATA_MARK_US);
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

    // Pausa para garantir stop-silence antes de voltar a aceitar RX
    delayMicroseconds(STOP_US + 500);
    _transmitting = false;
}

// ── RX ────────────────────────────────────────────────────────

void phy_update() { /* no-op — ticker trata do stop-silence */ }

bool phy_packet_ready(ir_packet_t *out) {
    if (!_packet_ready) return false;
    noInterrupts();
    *out          = _ready_packet;
    _packet_ready = false;
    interrupts();
    return true;
}