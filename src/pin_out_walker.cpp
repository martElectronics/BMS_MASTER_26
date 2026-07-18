/**
 * @file    pin_out_walker.cpp
 * @brief   BARRIDO DE SALIDAS — pone cada GPIO en HIGH por turnos (2 s)
 *
 * Utilidad de bring-up para localizar A QUE PATA del micro va una senal de
 * SALIDA (SDC_TSON, PRECHARGE_FAIL, BMS_OK...). Recorre los pines uno a uno,
 * deja SOLO ese en HIGH y el resto en LOW, e imprime cual esta activo:
 *
 *     Pin en HIGH: PA6  (indice 4/41)
 *
 * Mides con el multimetro la red que buscas (p.ej. SDC_TSON): cuando se ponga
 * a HIGH, el nombre que aparece por serie es su pata. Igual que buscaste el
 * boton con pin_walker, pero al reves (salidas en vez de entradas).
 *
 * ── COMANDOS ────────────────────────────────────────────────────────────────
 *   espacio/p = pausa/continua    n = siguiente pin    b = pin anterior
 *   s = repite el nombre del pin actual    r = reset del MCU
 *
 * ⚠ EXCLUIDOS a proposito (drivarlos rompe la placa):
 *     · PA2 / PA3  = USART2, el VCP del ST-Link -> es el 'Serial' del monitor.
 *     · PA13/ PA14 = SWD (SWDIO/SWCLK) -> debugger y flasheo.
 *
 * ⚠ BANCO EN SECO: forzar salidas a HIGH en una PCB poblada puede crear
 *   contencion (dos drivers peleando) si el pin choca con una salida externa
 *   o una entrada drivada por un aislador. Usar con la electronica sin enchufar.
 *
 * Lanzar con:  pio run -e pin_out_walker -t upload
 * Monitor:     pio device monitor -e pin_out_walker
 */

#include <Arduino.h>

// PA2/PA3 (USART2/VCP) y PA13/PA14 (SWD) NO estan en la lista: ver cabecera.
const int pines[] = {
  PA0, PA1, PA4, PA5, PA6, PA7,
  PA8, PA9, PA10, PA11, PA12, PA15,
  PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7,
  PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,
  PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7,
  PC8, PC9, PC10, PC11, PC12
};

const char* nombresPines[] = {
  "PA0", "PA1", "PA4", "PA5", "PA6", "PA7",
  "PA8", "PA9", "PA10", "PA11", "PA12", "PA15",
  "PB0", "PB1", "PB2", "PB3", "PB4", "PB5", "PB6", "PB7",
  "PB8", "PB9", "PB10", "PB11", "PB12", "PB13", "PB14", "PB15",
  "PC0", "PC1", "PC2", "PC3", "PC4", "PC5", "PC6", "PC7",
  "PC8", "PC9", "PC10", "PC11", "PC12"
};

const int numPines = sizeof(pines) / sizeof(pines[0]);

int  indiceActual = 0;
bool pausado      = false;

#define PASO_MS   2500UL   // tiempo en HIGH por pin

void apagarTodos() {
  for (int i = 0; i < numPines; i++) digitalWrite(pines[i], LOW);
}

void aplicarPin() {
  apagarTodos();
  digitalWrite(pines[indiceActual], HIGH);
  Serial.printf("Pin en HIGH: %-5s  (indice %d/%d)%s\n",
                nombresPines[indiceActual], indiceActual + 1, numPines,
                pausado ? "  [PAUSA]" : "");
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  while (Serial.available()) Serial.read();
  switch (c) {
    case ' ':
    case 'p':
      pausado = !pausado;
      Serial.printf("[%s]\n", pausado ? "PAUSA" : "CONTINUA");
      aplicarPin();
      break;
    case 'n':   // siguiente pin (util en pausa)
      indiceActual = (indiceActual + 1) % numPines;
      aplicarPin();
      break;
    case 'b':   // pin anterior
      indiceActual = (indiceActual + numPines - 1) % numPines;
      aplicarPin();
      break;
    case 's':
      aplicarPin();
      break;
    case 'r':
      Serial.println(F("Restart..."));
      delay(100);
      NVIC_SystemReset();
      break;
    default: break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  for (int i = 0; i < numPines; i++) pinMode(pines[i], OUTPUT);
  apagarTodos();

  Serial.println(F("=== BARRIDO DE SALIDAS ==="));
  Serial.println(F("Cada pin a HIGH 2 s. Mide la red buscada y anota el nombre que sale."));
  Serial.println(F("Cmd: espacio/p=pausa  n=siguiente  b=anterior  s=repite  r=reset"));
  Serial.println(F("PA2/3 (serie) y PA13/14 (SWD) excluidos."));
  aplicarPin();
}

void loop() {
  handleSerial();

  if (pausado) return;

  static unsigned long tPaso = 0;
  if (millis() - tPaso < PASO_MS) return;
  tPaso = millis();

  indiceActual = (indiceActual + 1) % numPines;
  aplicarPin();
}