#pragma once
#include "dataLink.h"
#include "hardware/pwm.h"

// ── pinos e parâmetros ────────────────────────────────────────
#define SERVO_PIN           9
#define SWEEP_MAX_NODES     16
#define SWEEP_STEP_DEG       5
#define SWEEP_SETTLE_MS    120   // tempo de espera após mover servo
#define SWEEP_PONG_WAIT_MS 300   // janela de espera por PONG por endereço

// ── defaults do range de endereços ───────────────────────────
#define SWEEP_ADDR_MIN_DEFAULT  1
#define SWEEP_ADDR_MAX_DEFAULT  5

// ── tabela de nós descobertos ─────────────────────────────────
typedef struct {
    uint8_t address;
    uint8_t angle;
} sweep_node_t;

// ── API ───────────────────────────────────────────────────────
void          sweep_init();
void          sweep_set_range(uint8_t addr_min, uint8_t addr_max); // configura range
void          sweep_run();                   // sweep 0→180° com PING por endereço
void          sweep_point(uint8_t address);
void          sweep_point_angle(int angle);
uint8_t       sweep_get_count();
sweep_node_t* sweep_get_nodes();
void          sweep_print_table();