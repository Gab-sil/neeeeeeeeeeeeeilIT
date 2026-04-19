#include "sweep.h"

// ── PWM de hardware para servo (50 Hz) ───────────────────────
// 125 MHz / 64 (clkdiv) / 39063 (wrap) ≈ 50 Hz
// 0°  = 1000 µs, 90° = 1500 µs, 180° = 2000 µs
// count = us * (125_000_000 / 64) / 1_000_000 = us * 1.953f
#define PWM_CLKDIV      64.0f
#define PWM_WRAP        39062
#define US_TO_COUNT(us) ((uint16_t)((us) * 1.953f))

static uint8_t      _tx_slice;
static sweep_node_t _nodes[SWEEP_MAX_NODES];
static uint8_t      _node_count = 0;
static int          _cur_angle  = 90;

// ── helpers privados ──────────────────────────────────────────
static uint16_t _angle_to_us(int angle) {
    return (uint16_t)(1000 + (angle * 1000) / 180);
}

static void _point_to(int angle) {
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;
    pwm_set_gpio_level(SERVO_PIN, US_TO_COUNT(_angle_to_us(angle)));
    _cur_angle = angle;
    delay(SWEEP_SETTLE_MS);
}

static void _flush_rx() {
    ir_frame_t dummy;
    while (dl_receive(&dummy) == DL_OK);
}

static bool _is_known(uint8_t addr) {
    for (uint8_t i = 0; i < _node_count; i++)
        if (_nodes[i].address == addr) return true;
    return false;
}

// ── public ────────────────────────────────────────────────────
void sweep_init() {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    _tx_slice = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_set_clkdiv(_tx_slice, PWM_CLKDIV);
    pwm_set_wrap(_tx_slice, PWM_WRAP);
    pwm_set_gpio_level(SERVO_PIN, US_TO_COUNT(1500));
    pwm_set_enabled(_tx_slice, true);
    _point_to(90);
    Serial.println("[SWEEP] Servo inicializado em 90°");
}

void sweep_run() {
    _node_count = 0;
    Serial.println("\n[SWEEP] === Início do sweep 0°→180° ===");

    for (int angle = 0; angle <= 180; angle += SWEEP_STEP_DEG) {
        _point_to(angle);
        _flush_rx();

        // broadcast PING
        ir_frame_t f;
        f.address = DL_BROADCAST;
        f.control = CMD_PING;
        f.len     = 0;
        dl_send(&f);

        Serial.print("[SWEEP] ");
        if (angle < 100) Serial.print(" ");
        if (angle < 10)  Serial.print(" ");
        Serial.print(angle);
        Serial.print("°");

        // coleta PONGs durante a janela
        uint32_t deadline = millis() + SWEEP_PONG_WAIT_MS;
        while (millis() < deadline) {
            phy_update();
            ir_frame_t resp;
            if (dl_receive(&resp) != DL_OK)        continue;
            if ((resp.control & 0x7F) != CMD_PONG) continue;
            if (_is_known(resp.source))             continue;
            if (_node_count >= SWEEP_MAX_NODES)     continue;

            _nodes[_node_count].address = resp.source;
            _nodes[_node_count].angle   = (uint8_t)angle;
            _node_count++;
            Serial.print(" | NÓ 0x");
            Serial.print(resp.source, HEX);
        }
        Serial.println();
    }

    _point_to(90);
    Serial.println("[SWEEP] === Sweep concluído ===");
    sweep_print_table();
}

void sweep_point(uint8_t address) {
    for (uint8_t i = 0; i < _node_count; i++) {
        if (_nodes[i].address == address) {
            Serial.print("[SWEEP] → 0x");
            Serial.print(address, HEX);
            Serial.print(" (");
            Serial.print(_nodes[i].angle);
            Serial.println("°)");
            _point_to(_nodes[i].angle);
            return;
        }
    }
    Serial.print("[SWEEP] AVISO: 0x");
    Serial.print(address, HEX);
    Serial.println(" não está na tabela!");
}

void sweep_point_angle(int angle) {
    _point_to(angle);
    Serial.print("[SWEEP] → ");
    Serial.print(angle);
    Serial.println("°");
}

uint8_t       sweep_get_count() { return _node_count; }
sweep_node_t* sweep_get_nodes() { return _nodes; }

void sweep_print_table() {
    Serial.println("[SWEEP] Tabela de nós:");
    if (_node_count == 0) { Serial.println("  (vazia)"); return; }
    for (uint8_t i = 0; i < _node_count; i++) {
        Serial.print("  [");
        Serial.print(i);
        Serial.print("] addr=0x");
        Serial.print(_nodes[i].address, HEX);
        Serial.print("  ângulo=");
        Serial.print(_nodes[i].angle);
        Serial.println("°");
    }
}