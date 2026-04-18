#include "datalink.h"

// ── CRC ───────────────────────────────────────────────────────
uint8_t dl_crc(const uint8_t *payload, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= payload[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

// ── init ──────────────────────────────────────────────────────
void dl_init() {
    phy_init();
}

// ── send ──────────────────────────────────────────────────────
dl_error_t dl_send(ir_frame_t *frame) {
    frame->source = DL_MY_ADDRESS;
    frame->crc    = dl_crc(frame->payload, frame->len);

    if (frame->len > MAX_PAYLOAD) return DL_ERR_LEN;

    // serialize to byte array
    uint8_t buf[4 + MAX_PAYLOAD + 1];
    uint8_t n = 0;
    buf[n++] = frame->source;
    buf[n++] = frame->address;
    buf[n++] = frame->control;
    buf[n++] = frame->len;
    for (uint8_t i = 0; i < frame->len; i++)
        buf[n++] = frame->payload[i];
    buf[n++] = frame->crc;

    phy_send_raw(buf, n);
    return DL_OK;
}

// ── assemble frame from phy nibbles ──────────────────────────
static dl_error_t _assemble(const ir_packet_t *p, ir_frame_t *frame) {
    // need even number of nibbles for full bytes
    if (p->count % 2 != 0)          return DL_ERR_LEN;

    uint8_t byte_count = p->count / 2;
    if (byte_count < 5)              return DL_ERR_LEN;  // min: 4 header + 1 crc

    // decode bytes
    uint8_t buf[4 + MAX_PAYLOAD + 1];
    for (uint8_t i = 0; i < byte_count && i < sizeof(buf); i++)
        buf[i] = (p->nibbles[i * 2] << 4) | p->nibbles[i * 2 + 1];

    frame->source  = buf[0];
    frame->address = buf[1];
    frame->control = buf[2];
    frame->len     = buf[3];

    // echo filter
    if (frame->source == DL_MY_ADDRESS) return DL_ERR_ECHO;

    // address filter — accept mine or broadcast
    if (frame->address != DL_MY_ADDRESS && frame->address != DL_BROADCAST)
        return DL_ERR_NOT_FOR_ME;

    // length sanity
    if (frame->len > MAX_PAYLOAD)    return DL_ERR_LEN;

    // expected total bytes: 4 header + len payload + 1 crc
    if (byte_count != (uint8_t)(4 + frame->len + 1)) return DL_ERR_LEN;

    // copy payload
    memcpy(frame->payload, &buf[4], frame->len);

    // CRC check
    uint8_t received_crc = buf[4 + frame->len];
    uint8_t computed_crc = dl_crc(frame->payload, frame->len);
    if (received_crc != computed_crc) return DL_ERR_CRC;

    frame->crc = received_crc;
    return DL_OK;
}

// ── receive (non-blocking) ────────────────────────────────────
dl_error_t dl_receive(ir_frame_t *frame) {
    ir_packet_t p;
    if (!phy_packet_ready(&p)) return DL_ERR_NO_PACKET;
    if (p.error)               return DL_ERR_CRC;
    return _assemble(&p, frame);
}

// ── wait for ACK/PONG from a specific source ─────────────────
dl_error_t dl_wait_ack(uint8_t expected_src, uint32_t timeout_ms) {
    uint32_t deadline = millis() + timeout_ms;
    while (millis() < deadline) {
        phy_update();
        ir_frame_t f;
        dl_error_t err = dl_receive(&f);
        if (err == DL_OK) {
            if (f.source == expected_src &&
               (f.control == CMD_ACK || f.control == CMD_PONG))
                return DL_OK;
        }
    }
    return DL_ERR_ACK_TIMEOUT;
}

// ── error string ──────────────────────────────────────────────
const char* dl_error_str(dl_error_t err) {
    switch (err) {
        case DL_OK:              return "OK";
        case DL_ERR_CRC:         return "ERR_CRC";
        case DL_ERR_LEN:         return "ERR_LEN";
        case DL_ERR_ECHO:        return "ERR_ECHO";
        case DL_ERR_NOT_FOR_ME:  return "ERR_NOT_FOR_ME";
        case DL_ERR_ACK_TIMEOUT: return "ERR_ACK_TIMEOUT";
        case DL_ERR_NO_PACKET:   return "ERR_NO_PACKET";
        default:                 return "ERR_UNKNOWN";
    }
}