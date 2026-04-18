#pragma once
#include <Arduino.h>
#include "phy.h"

#define DL_MY_ADDRESS  0x01
#define DL_BROADCAST   0xFF
#define MAX_PAYLOAD    27
#define ACK_TIMEOUT_MS 1000

// control commands
#define CMD_ACK   0x01
#define CMD_WRITE 0x02
#define CMD_READ  0x03
#define CMD_PING  0x04
#define CMD_PONG  0x05
#define FLAG_ACK  0x80

typedef enum {
    DL_OK              = 0,
    DL_ERR_CRC         = 1,
    DL_ERR_LEN         = 2,
    DL_ERR_ECHO        = 3,
    DL_ERR_NOT_FOR_ME  = 4,
    DL_ERR_ACK_TIMEOUT = 5,
    DL_ERR_NO_PACKET   = 6,
} dl_error_t;

typedef struct {
    uint8_t source;
    uint8_t address;
    uint8_t control;
    uint8_t len;
    uint8_t payload[MAX_PAYLOAD];
    uint8_t crc;
} ir_frame_t;

void        dl_init();
uint8_t     dl_crc(const uint8_t *payload, uint8_t len);
dl_error_t  dl_send(ir_frame_t *frame);
dl_error_t  dl_receive(ir_frame_t *frame);  // non-blocking, call in loop
dl_error_t  dl_wait_ack(uint8_t expected_src, uint32_t timeout_ms);
const char* dl_error_str(dl_error_t err);