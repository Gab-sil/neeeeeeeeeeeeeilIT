#include "app.h"

static uint8_t _regs[64] = {0};

// ── handlers RX ──────────────────────────────────────────────
static void _handle_ping(const ir_frame_t *f) {
    ir_frame_t pong;
    pong.address = f->source;
    pong.control = CMD_PONG;
    pong.len     = 0;
    dl_send(&pong);
    Serial.print("[APP] PING de 0x");
    Serial.print(f->source, HEX);
    Serial.println(" → PONG enviado");
}

static void _handle_write(const ir_frame_t *f) {
    if (f->len < 2) return;
    uint8_t start = f->payload[0];
    uint8_t count = f->payload[1];
    if (start + count > 64) return;
    for (uint8_t i = 0; i < count && (2 + i) < f->len; i++)
        _regs[start + i] = f->payload[2 + i];
    Serial.print("[APP] WRITE: ");
    Serial.print(count);
    Serial.print(" regs a partir de idx ");
    Serial.println(start);
    if (f->control & FLAG_ACK) {
        ir_frame_t ack;
        ack.address = f->source;
        ack.control = CMD_ACK;
        ack.len     = 0;
        dl_send(&ack);
        Serial.println("[APP] ACK enviado");
    }
}

static void _handle_read(const ir_frame_t *f) {
    if (f->len != 2) return;
    uint8_t start = f->payload[0];
    uint8_t count = f->payload[1];
    if (start + count > 64) return;
    ir_frame_t resp;
    resp.address    = f->source;
    resp.control    = CMD_READ;
    resp.len        = 2 + count;
    resp.payload[0] = start;
    resp.payload[1] = count;
    for (uint8_t i = 0; i < count; i++)
        resp.payload[2 + i] = _regs[start + i];
    dl_send(&resp);
    Serial.print("[APP] READ: respondido com ");
    Serial.print(count);
    Serial.print(" regs a partir de idx ");
    Serial.println(start);
}

static void _process_frame(const ir_frame_t *f) {
    uint8_t cmd = f->control & 0x7F;
    switch (cmd) {
        case CMD_PING:  _handle_ping(f);  break;
        case CMD_WRITE: _handle_write(f); break;
        case CMD_READ:  _handle_read(f);  break;
        case CMD_ACK:
        case CMD_PONG:
            Serial.print("[APP] ACK/PONG de 0x");
            Serial.println(f->source, HEX);
            break;
        default:
            Serial.print("[APP] CMD desconhecido: 0x");
            Serial.println(cmd, HEX);
            break;
    }
}

// ── serial menu ───────────────────────────────────────────────
static void _print_menu() {
    Serial.println("\n=== MENU ===");
    Serial.println("1 - PING torre");
    Serial.println("2 - WRITE registos");
    Serial.println("3 - READ registos");
    Serial.println("4 - Ver registos locais");
    Serial.println("============");
}

static uint8_t _ask_byte(const char *prompt) {
    Serial.print(prompt);
    char buf[8] = {0};
    uint8_t i = 0;
    while (true) {
        phy_update();
        if (Serial.available()) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                if (i > 0) break;
            } else if (i < sizeof(buf) - 1) {
                buf[i++] = c;
            }
        }
    }
    uint8_t val = (uint8_t)atoi(buf);
    Serial.println(val);
    return val;
}

static void _do_ping() {
    uint8_t dest = _ask_byte("Endereço destino (dec): ");
    uint8_t wait = _ask_byte("Aguardar PONG? (1/0): ");

    ir_frame_t f;
    f.address = dest;
    f.control = CMD_PING;
    f.len     = 0;
    dl_send(&f);
    Serial.print("[APP] PING enviado para 0x");
    Serial.println(dest, HEX);

    if (wait) {
        dl_error_t err = dl_wait_ack(dest, ACK_TIMEOUT_MS);
        Serial.print("[APP] PONG: ");
        Serial.println(dl_error_str(err));
    }
}

static void _do_write() {
    uint8_t dest     = _ask_byte("Endereço destino (dec): ");
    uint8_t start    = _ask_byte("Start index (dec): ");
    uint8_t count    = _ask_byte("Número de registos (dec): ");
    uint8_t with_ack = _ask_byte("Com ACK? (1/0): ");

    if (count > 25) { Serial.println("Máximo 25!"); return; }

    ir_frame_t f;
    f.address    = dest;
    f.control    = with_ack ? (CMD_WRITE | FLAG_ACK) : CMD_WRITE;
    f.len        = 2 + count;
    f.payload[0] = start;
    f.payload[1] = count;
    for (uint8_t i = 0; i < count; i++)
        f.payload[2 + i] = _ask_byte("  valor (dec): ");

    dl_send(&f);
    if (with_ack) {
        dl_error_t err = dl_wait_ack(dest, ACK_TIMEOUT_MS);
        Serial.print("[APP] ACK: ");
        Serial.println(dl_error_str(err));
    } else {
        Serial.println("[APP] WRITE enviado (sem ACK)");
    }
}

static void _do_read() {
    uint8_t dest  = _ask_byte("Endereço destino (dec): ");
    uint8_t start = _ask_byte("Start index (dec): ");
    uint8_t count = _ask_byte("Número de registos (dec): ");

    if (count > 25) { Serial.println("Máximo 25!"); return; }

    ir_frame_t f;
    f.address    = dest;
    f.control    = CMD_READ;
    f.len        = 2;
    f.payload[0] = start;
    f.payload[1] = count;
    dl_send(&f);
    Serial.println("[APP] READ enviado, à espera de resposta...");

    uint32_t deadline = millis() + ACK_TIMEOUT_MS;
    while (millis() < deadline) {
        phy_update();
        ir_frame_t resp;
        if (dl_receive(&resp) == DL_OK) {
            if ((resp.control & 0x7F) == CMD_READ && resp.len >= 2) {
                Serial.print("[APP] Dados: ");
                for (uint8_t i = 2; i < resp.len; i++) {
                    Serial.print("reg[");
                    Serial.print(resp.payload[0] + (i - 2));
                    Serial.print("]=0x");
                    Serial.print(resp.payload[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();
                return;
            }
        }
    }
    Serial.println("[APP] Timeout!");
}

static void _do_show_regs() {
    Serial.println("[APP] Registos locais:");
    for (uint8_t i = 0; i < 64; i++) {
        Serial.print("  [");
        if (i < 10) Serial.print("0");
        Serial.print(i);
        Serial.print("]=0x");
        if (_regs[i] < 0x10) Serial.print("0");
        Serial.print(_regs[i], HEX);
        if ((i + 1) % 8 == 0) Serial.println();
        else Serial.print("  ");
    }
    Serial.println();
}

// ── public ────────────────────────────────────────────────────
void app_init() {
    dl_init();
    _print_menu();
}

void app_update() {
    phy_update();

    ir_frame_t f;
    if (dl_receive(&f) == DL_OK)
        _process_frame(&f);

    if (Serial.available()) {
        char c = Serial.read();
        switch (c) {
            case '1': _do_ping();      break;
            case '2': _do_write();     break;
            case '3': _do_read();      break;
            case '4': _do_show_regs(); break;
            default: break;
        }
        _print_menu();
    }
}