#include "app.h"

void setup() {
    Serial.begin(115200);
    app_init();
}

void loop() {
    app_update();
}