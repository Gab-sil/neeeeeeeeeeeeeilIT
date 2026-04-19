#include "phy.h"

// ── PIO RX state ──────────────────────────────────────────────
static PIO  _pio       = pio0;
static uint _sm        = 0;

// ── TX state ──────────────────────────────────────────────────
static uint8_t _tx_slice;
static bool    _transmitting = false;

// ── Packet state ──────────────────────────────────────────────
static ir_packet_t _packet;
static ir_packet_t _ready_packet;
static bool        _packet_ready = false;
static uint32_t    _last_fifo_time = 0;  // for stop silence detection

typedef enum {
    STATE_IDLE,
    STATE_GAP,
    STATE_MARK,
} ir_rx_state_t;

static ir_rx_state_t _state = STATE_IDLE;

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

// ── PIO RX FIFO processing ────────────────────────────────────
static void _process_fifo() {
    while (!pio_sm_is_rx_fifo_empty(_pio, _sm)) {
        uint32_t word        = pio_sm_get(_pio, _sm);
        bool     is_gap      = (word >> 31) & 1;
        uint32_t duration_us = word & 0x7FFFFFFF;

        _last_fifo_time = micros();  // update last activity timestamp

        if (!is_gap) {
            // MARK received
            if (duration_us >= 750 && duration_us <= 1250) {
                // valid start mark
                _packet  = ir_packet_t{};
                _state   = STATE_GAP;
            } else if (_state == STATE_MARK) {
                if (duration_us >= 375 && duration_us <= 625) {
                    _state = STATE_GAP;
                } else {
                    _packet.error = true;
                    _state = STATE_IDLE;
                }
            } else {
                _state = STATE_IDLE;
            }
        } else {
            // GAP received
            if (_state != STATE_GAP) {
                _state = STATE_IDLE;
                return;
            }

            if (duration_us >= 375) {
                // rounding: add half a step before dividing
                uint8_t nibble = (duration_us - GAP_BASE_US + GAP_STEP_US / 2) / GAP_STEP_US;
                if (nibble > 0xE) {
                    _packet.error = true;
                    _state = STATE_IDLE;
                    return;
                }
                if (_packet.count < MAX_NIBBLES) {
                    _packet.nibbles[_packet.count++] = nibble;
                    _state = STATE_MARK;
                } else {
                    _packet.error = true;
                    _state = STATE_IDLE;
                }
            } else {
                _packet.error = true;
                _state = STATE_IDLE;
            }
        }
    }
}

// ── public API ────────────────────────────────────────────────
void phy_init() {
    // RX — PIO
    uint offset = pio_add_program(_pio, &ir_rx_program);
    ir_rx_program_init(_pio, _sm, offset, RECEIVER_PIN, 1000000.0f);

    // TX — PWM at 38kHz
    // wrap = clk / freq - 1 = 125,000,000 / 38,000 - 1 ≈ 3289
    pinMode(SERVO_PIN, OUTPUT);
    gpio_set_function(EMITTER_PIN, GPIO_FUNC_PWM);
    _tx_slice = pwm_gpio_to_slice_num(EMITTER_PIN);
    pwm_set_wrap(_tx_slice, 3289);
    pwm_set_clkdiv(_tx_slice, 1.0f);
    pwm_set_gpio_level(EMITTER_PIN, 3289 / 2);  // 50% duty cycle
    pwm_set_enabled(_tx_slice, false);
}

void phy_send_nibble(uint8_t nibble) {
    _mark(DATA_MARK_US);
    _gap(GAP_BASE_US + nibble * GAP_STEP_US);
}

void phy_send_byte(uint8_t b) {
    phy_send_nibble((b >> 4) & 0x0F);
    phy_send_nibble( b       & 0x0F);
}

void phy_send_raw(const uint8_t *bytes, uint8_t len) {
    _transmitting = true;

    // disable PIO SM during TX to avoid reading our own signal
    pio_sm_set_enabled(_pio, _sm, false);
    // drain any stale FIFO values
    while (!pio_sm_is_rx_fifo_empty(_pio, _sm))
        pio_sm_get(_pio, _sm);

    _mark(START_MARK_US);
    for (uint8_t i = 0; i < len; i++)
        phy_send_byte(bytes[i]);

    // stop silence — just wait
    _gap(STOP_US + 1000);

    // re-enable PIO RX
    pio_sm_set_enabled(_pio, _sm, true);
    _transmitting = false;
    _state = STATE_IDLE;
}

void phy_update() {
    if (_transmitting) return;

    _process_fifo();

    // stop silence detection — if we haven't seen a FIFO word in a while
    // and we're mid-packet, finalize it
    if (_state != STATE_IDLE) {
        if (micros() - _last_fifo_time > STOP_US) {
            if (_packet.count > 0 && !_packet.error) {
                _packet.complete = true;
                _ready_packet = _packet;
                _packet_ready = true;
            }
            _state = STATE_IDLE;
        }
    }
}

bool phy_packet_ready(ir_packet_t *out) {
    if (!_packet_ready) return false;
    *out          = _ready_packet;
    _packet_ready = false;
    return true;
}