/**
 * @file    sdc_tson_high.cpp
 * @brief   MINIMO — pone SDC_TSON (PA6) en HIGH y nada mas.
 *
 * ⚠ OJO: aqui el pin es PA6 (SIN guion bajo) = nº de pin Arduino. NO usar PA_6
 *   (PinName), que digitalWrite interpreta como indice y apunta a OTRA pata.
 *
 * Lanzar con:  pio run -e sdc_tson_high -t upload
 */

#include <Arduino.h>

#define PIN_SDC_TSON  PA6   // sin guion bajo: nº de pin Arduino

void setup()
{
    Serial.begin(115200);
    delay(500);
    pinMode(PIN_SDC_TSON, OUTPUT);
    digitalWrite(PIN_SDC_TSON, HIGH);
    Serial.println("SDC_TSON (PA6) -> HIGH");
}

void loop()
{
    digitalWrite(PIN_SDC_TSON, HIGH);
}
