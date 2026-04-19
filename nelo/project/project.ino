#include "app.h"

void setup() {
    Serial.begin(115200);
    while (!Serial);
    app_init();
}

void loop() {
    app_update();
}