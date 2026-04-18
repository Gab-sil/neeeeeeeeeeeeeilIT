#include "./phy.h"

void setup() {
    Serial.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);

    phy_init();

}

void loop() {

    digitalWrite(LED_BUILTIN, HIGH);

    phy_update();  // stop silence detection

    ir_packet_t p;
    if (phy_packet_ready(&p)) {
        Serial.println("[RX] Packet complete!");
        Serial.print("[RX] Nibble count: ");
        Serial.println(p.count);

        if (p.error)
            Serial.println("[RX] Warning: packet flagged error");

        Serial.print("[RX] Bytes (hex): ");
        for (int i = 0; i + 1 < p.count; i += 2) {
            uint8_t b = (p.nibbles[i] << 4) | p.nibbles[i + 1];
            if (b < 0x10) Serial.print("0");
            Serial.print(b, HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
}