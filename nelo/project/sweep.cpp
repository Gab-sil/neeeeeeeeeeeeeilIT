#include "sweep.h"
#include <Servo.h>

// ── estado interno ────────────────────────────────────────────
static Servo        _servo;
static sweep_node_t _nodes[SWEEP_MAX_NODES];
static uint8_t      _node_count  = 0;
static int          _cur_angle   = 90;

// ── helpers privados ──────────────────────────────────────────
static void _point_to(int angle) {
    _servo.write(angle);
    _cur_angle = angle;
    delay(SWEEP_SETTLE_MS);
}

// Drena a fila de RX sem bloquear (descarta tudo).
// Útil antes de enviar, para não processar pacotes velhos.
static void _flush_rx() {
    ir_frame_t dummy;
    while (dl_receive(&dummy) == DL_OK);
}

// ── public ────────────────────────────────────────────────────
void sweep_init() {
    _servo.attach(SERVO_PIN);
    _point_to(90);
    Serial.println("[SWEEP] Servo inicializado em 90°");
}

void sweep_run() {
    _node_count = 0;

    Serial.println("\n[SWEEP] === Início do sweep ===");
    Serial.println("[SWEEP] 0° ──────────────────────── 180°");

    for (int angle = 0; angle <= 180; angle += SWEEP_STEP_DEG) {
        _point_to(angle);
        _flush_rx();

        // ── broadcast PING ────────────────────────────────────
        ir_frame_t f;
        f.address = DL_BROADCAST;
        f.control = CMD_PING;
        f.len     = 0;
        dl_send(&f);

        Serial.print("[SWEEP] ");
        if (angle < 100) Serial.print(" ");
        if (angle < 10)  Serial.print(" ");
        Serial.print(angle);
        Serial.print("° |");

        // ── coleta PONGs ──────────────────────────────────────
        uint32_t deadline = millis() + SWEEP_PONG_WAIT_MS;
        while (millis() < deadline) {
            phy_update();
            ir_frame_t resp;
            if (dl_receive(&resp) != DL_OK) continue;
            if ((resp.control & 0x7F) != CMD_PONG) continue;

            // Ignorar duplicados (mesmo nó já encontrado)
            bool known = false;
            for (uint8_t i = 0; i < _node_count; i++) {
                if (_nodes[i].address == resp.source) { known = true; break; }
            }
            if (!known && _node_count < SWEEP_MAX_NODES) {
                _nodes[_node_count].address = resp.source;
                _nodes[_node_count].angle   = (uint8_t)angle;
                _node_count++;
                Serial.print(" NÓ 0x");
                Serial.print(resp.source, HEX);
                Serial.print(" @");
                Serial.print(angle);
                Serial.print("°");
            }
        }
        Serial.println();
    }

    // Parqueia servo no centro
    _point_to(90);

    Serial.println("[SWEEP] === Sweep concluído ===");
    sweep_print_table();
}

void sweep_point(uint8_t address) {
    for (uint8_t i = 0; i < _node_count; i++) {
        if (_nodes[i].address == address) {
            Serial.print("[SWEEP] Apontar servo → 0x");
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
    Serial.println(" não está na tabela de nós!");
}

uint8_t sweep_get_count() { return _node_count; }

sweep_node_t* sweep_get_nodes() { return _nodes; }

void sweep_print_table() {
    Serial.println("[SWEEP] Tabela de nós:");
    if (_node_count == 0) {
        Serial.println("  (nenhum nó descoberto)");
        return;
    }
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