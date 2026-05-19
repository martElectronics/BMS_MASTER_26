#include <Arduino.h>

void setup() {
    Serial.begin(115200);
}

void loop() {
    static uint32_t last = 0;
    static uint32_t n = 0;

    if (millis() - last >= 500) {
        last = millis();
        Serial.print("Numero: ");
        Serial.println(n++);
    }
}

