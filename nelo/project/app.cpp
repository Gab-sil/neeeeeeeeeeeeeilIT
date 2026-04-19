#include "app.h"
#include "sweep.h"

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
    Serial.println("1 - PING nó");
    Serial.println("2 - WRITE registos");
    Serial.println("3 - READ registos");
    Serial.println("4 - Ver registos locais");
    Serial.println("5 - SWEEP (descobrir nós)");
    Serial.println("6 - AUTO (ler fonte → escrever destino)");
    Serial.println("7 - Ver tabela de nós");
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

// ── helper: ler N registos de um nó remoto ───────────────────
// Aponta o servo para o nó, envia CMD_READ, aguarda resposta.
// Devolve true e preenche buf[] em caso de sucesso.
static bool _remote_read(uint8_t node_addr, uint8_t start, uint8_t count, uint8_t *buf) {
    if (count == 0 || count > MAX_PAYLOAD - 2) return false;

    sweep_point(node_addr);        // aponta servo

    ir_frame_t f;
    f.address    = node_addr;
    f.control    = CMD_READ;
    f.len        = 2;
    f.payload[0] = start;
    f.payload[1] = count;
    dl_send(&f);

    uint32_t deadline = millis() + ACK_TIMEOUT_MS;
    while (millis() < deadline) {
        phy_update();
        ir_frame_t resp;
        if (dl_receive(&resp) != DL_OK) continue;
        if (resp.source           != node_addr)  continue;
        if ((resp.control & 0x7F) != CMD_READ)   continue;
        if (resp.len              < 2 + count)   continue;

        memcpy(buf, &resp.payload[2], count);
        return true;
    }
    Serial.print("[APP] Timeout lendo 0x");
    Serial.println(node_addr, HEX);
    return false;
}

// ── helper: escrever N registos num nó remoto ─────────────────
static bool _remote_write(uint8_t node_addr, uint8_t start,
                           uint8_t count, const uint8_t *vals, bool with_ack) {
    if (count == 0 || count > MAX_PAYLOAD - 2) return false;

    sweep_point(node_addr);        // aponta servo

    ir_frame_t f;
    f.address    = node_addr;
    f.control    = with_ack ? (CMD_WRITE | FLAG_ACK) : CMD_WRITE;
    f.len        = 2 + count;
    f.payload[0] = start;
    f.payload[1] = count;
    memcpy(&f.payload[2], vals, count);
    dl_send(&f);

    if (!with_ack) return true;

    dl_error_t err = dl_wait_ack(node_addr, ACK_TIMEOUT_MS);
    if (err != DL_OK) {
        Serial.print("[APP] ACK timeout de 0x");
        Serial.println(node_addr, HEX);
        return false;
    }
    return true;
}

// ── acções do menu ────────────────────────────────────────────
static void _do_ping() {
    uint8_t dest = _ask_byte("Endereço destino (dec): ");
    uint8_t wait = _ask_byte("Aguardar PONG? (1/0): ");

    if (sweep_get_count() > 0) sweep_point(dest);  // aponta se já fez sweep

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

    uint8_t vals[25];
    for (uint8_t i = 0; i < count; i++)
        vals[i] = _ask_byte("  valor (dec): ");

    bool ok = _remote_write(dest, start, count, vals, with_ack != 0);
    Serial.println(ok ? "[APP] WRITE OK" : "[APP] WRITE FALHOU");
}

static void _do_read() {
    uint8_t dest  = _ask_byte("Endereço destino (dec): ");
    uint8_t start = _ask_byte("Start index (dec): ");
    uint8_t count = _ask_byte("Número de registos (dec): ");

    if (count > 25) { Serial.println("Máximo 25!"); return; }

    uint8_t buf[25];
    if (_remote_read(dest, start, count, buf)) {
        Serial.print("[APP] Dados de 0x");
        Serial.print(dest, HEX);
        Serial.print(": ");
        for (uint8_t i = 0; i < count; i++) {
            Serial.print("reg[");
            Serial.print(start + i);
            Serial.print("]=0x");
            if (buf[i] < 0x10) Serial.print("0");
            Serial.print(buf[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
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

// ── modo autónomo ─────────────────────────────────────────────
// Protocolo de registos (especificação da competição):
//   reg[00] = endereço da torre de destino
//   reg[01] = XX = start index para escrever no destino
//   reg[02] = N  = número de registos
//   reg[03 … 03+N-1] = valores a escrever
static void _do_auto() {
    if (sweep_get_count() == 0) {
        Serial.println("[AUTO] Sem nós na tabela. Faz o sweep primeiro (opção 5)!");
        return;
    }

    sweep_print_table();
    uint8_t src_addr = _ask_byte("Endereço da torre FONTE (dec): ");

    // ── 1. Ler cabeçalho (regs 0..2) ─────────────────────────
    Serial.println("[AUTO] A ler header (regs 0..2) da torre fonte...");
    uint8_t header[3];
    if (!_remote_read(src_addr, 0, 3, header)) {
        Serial.println("[AUTO] Falhou leitura do header!");
        return;
    }
    uint8_t dest_addr  = header[0];
    uint8_t write_start = header[1];
    uint8_t count       = header[2];

    Serial.print("[AUTO] destino=0x");  Serial.print(dest_addr, HEX);
    Serial.print("  start=");           Serial.print(write_start);
    Serial.print("  count=");           Serial.println(count);

    if (count == 0 || count > 25) {
        Serial.println("[AUTO] Count inválido!");
        return;
    }

    // ── 2. Ler valores (regs 3 … 3+count-1) ──────────────────
    Serial.println("[AUTO] A ler valores da torre fonte...");
    uint8_t vals[25];
    if (!_remote_read(src_addr, 3, count, vals)) {
        Serial.println("[AUTO] Falhou leitura dos valores!");
        return;
    }

    Serial.print("[AUTO] Valores: ");
    for (uint8_t i = 0; i < count; i++) {
        Serial.print("0x");
        if (vals[i] < 0x10) Serial.print("0");
        Serial.print(vals[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    // ── 3. Escrever no destino ────────────────────────────────
    Serial.print("[AUTO] A escrever na torre 0x");
    Serial.print(dest_addr, HEX);
    Serial.println("...");

    bool ok = _remote_write(dest_addr, write_start, count, vals, true);
    Serial.println(ok ? "[AUTO] ✓ Missão concluída!" : "[AUTO] ✗ Escrita falhou!");
}

// ── public ────────────────────────────────────────────────────
void app_init() {
    dl_init();
    sweep_init();
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
            case '1': _do_ping();            break;
            case '2': _do_write();           break;
            case '3': _do_read();            break;
            case '4': _do_show_regs();       break;
            case '5': sweep_run();           break;
            case '6': _do_auto();            break;
            case '7': sweep_print_table();   break;
            default: break;
        }
        _print_menu();
    }
}