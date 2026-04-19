#pragma once
#include "dataLink.h"
 
// ── pinos e parâmetros ────────────────────────────────────────
#define SERVO_PIN           0
#define SWEEP_MAX_NODES     16
#define SWEEP_STEP_DEG      5
#define SWEEP_SETTLE_MS     120   // tempo de espera após mover servo
#define SWEEP_PONG_WAIT_MS  300   // janela de coleta de PONGs por step
 
// ── tabela de nós descobertos ─────────────────────────────────
typedef struct {
    uint8_t address;
    uint8_t angle;
} sweep_node_t;
 
// ── API ───────────────────────────────────────────────────────
void          sweep_init();                     // attach servo, posição central
void          sweep_run();                      // sweep 0→180° em steps de 5°
void          sweep_point(uint8_t address);     // aponta servo para o nó
uint8_t       sweep_get_count();
sweep_node_t* sweep_get_nodes();
void          sweep_print_table();
 