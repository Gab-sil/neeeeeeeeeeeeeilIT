#define ServoIn   18
#define Emitter   19
#define Receiver  20
#define MAX_NIBBLES 64

struct ir_packet_t {
    uint8_t  nibbles[MAX_NIBBLES];
    uint8_t  count    = 0;
    bool     complete = false;
    bool     error    = false;
};

typedef enum {
    STATE_IDLE,
    STATE_START_MARK,
    STATE_GAP,
    STATE_MARK,
} ir_rx_state_t;

volatile ir_rx_state_t state          = STATE_IDLE;
volatile uint32_t      last_edge_time = 0;
volatile bool          packet_ready   = false;

ir_packet_t packet;
ir_packet_t ready_packet;

void on_edge() {
    bool pin = digitalRead(Receiver);
    uint32_t now = micros();
    uint32_t elapsed = now - last_edge_time;

    bool falling = (pin == LOW);
    bool rising  = (pin == HIGH);

    switch (state) {
        case STATE_IDLE:
            if (falling) {
                last_edge_time = now;
                state = STATE_START_MARK;
            }
            break;

        case STATE_START_MARK:
            if (rising) {
                if (elapsed >= 750 && elapsed <= 1250) {
                    packet = ir_packet_t{};
                    last_edge_time = now;
                    state = STATE_GAP;
                } else {
                    state = STATE_IDLE;
                }
            }
            break;

        case STATE_GAP:
            if (falling) {
                if (elapsed > 5000) {
                    packet.complete = true;
                    ready_packet = packet;
                    packet_ready = true;
                    state = STATE_IDLE;
                } else if (elapsed >= 375) {
                    if (packet.count < MAX_NIBBLES) {
                      uint8_t nibble = (elapsed - 500) / 300;
                      if (nibble > 0xE) {
                          packet.error = true;
                          state = STATE_IDLE;
                          return;
                      }
                      packet.nibbles[packet.count++] = nibble;
                    } else {
                        packet.error = true;
                    }
                    last_edge_time = now;
                    state = STATE_MARK;
                } else {
                    packet.error = true;
                    state = STATE_IDLE;
                }
            }
            break;

        case STATE_MARK:
            if (rising) {
                if (elapsed >= 375 && elapsed <= 625) {
                    last_edge_time = now;
                    state = STATE_GAP;
                } else {
                    packet.error = true;
                    state = STATE_IDLE;
                }
            }
            break;
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(ServoIn,  OUTPUT);
    pinMode(Emitter,  OUTPUT);
    pinMode(Receiver, INPUT);
    analogWriteFreq(50);
    analogWriteRange(20000);
    digitalWrite(LED_BUILTIN, HIGH);

    attachInterrupt(digitalPinToInterrupt(Receiver), on_edge, CHANGE);
}

void loop() {
    static ir_rx_state_t last_state = STATE_IDLE;
    ir_rx_state_t current_state = state;

    if (last_state == STATE_START_MARK && current_state == STATE_GAP) {
        Serial.println("[RX] Start mark validated, receiving nibbles...");
    }

    last_state = current_state;

    if (packet_ready) {
        noInterrupts();
        ir_packet_t p = ready_packet;
        packet_ready = false;
        interrupts();

        Serial.println("[RX] Packet complete!");
        Serial.print("[RX] Nibble count: ");
        Serial.println(p.count);

        if (p.error) {
            Serial.println("[RX] Warning: packet flagged error");
        }

        Serial.print("[RX] Decoded bytes (hex): ");
        for (int i = 0; i + 1 < p.count; i += 2) {
            uint8_t byte_val = (p.nibbles[i] << 4) | p.nibbles[i + 1];
            if (byte_val < 0x10) Serial.print("0");
            Serial.print(byte_val, HEX);
            Serial.print(" ");
        }
        Serial.println();

        Serial.print("[RX] Decoded string: ");
        for (int i = 0; i + 1 < p.count; i += 2) {
            uint8_t byte_val = (p.nibbles[i] << 4) | p.nibbles[i + 1];
            Serial.print((char)byte_val);
        }
        Serial.println();
    }
}