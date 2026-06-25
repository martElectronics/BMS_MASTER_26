// main.cpp
#include <Arduino.h>

#define PIN_BMS_OK      PA_4

void setup() {
     Serial.begin(115200);
    delay(500);

    pinMode(PIN_BMS_OK, OUTPUT);
    digitalWrite(PIN_BMS_OK, HIGH);  // BMS_OK HIGH = pack sano
    delay(2000);
    digitalWrite(PIN_BMS_OK, LOW);  // BMS_OK HIGH = pack sano
}

void loop() {
   

    Serial.println("BMS_OK HIGH = pack sano");  

}