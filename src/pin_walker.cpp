/**
 * @file    pin_walker.cpp
 * @brief   RASTREADOR DE ENTRADAS — lee todos los GPIO y avisa cual se mueve
 *
 * Utilidad de bring-up para localizar A QUE PATA del micro entra una senal:
 * configura todos los pines como ENTRADA y vigila sus niveles. Cuando inyectas
 * una senal (pulsas un boton, cierras el SDC, drivas un aislador...) imprime
 * al instante que pin cambio y a que nivel:
 *
 *     PB4: 0 -> 1  @12345 ms
 *
 * Asi no hay que leer 41 pines a la vez: solo salta el que se mueve.
 *
 * ── PULL INTERNO (comandos u/p/n) ───────────────────────────────────────────
 *   Por defecto PULL-DOWN: lo no conectado queda en 0 y una senal activa-alta
 *   (aislador push-pull, 3V3) salta a 1 limpio. Si buscas senales activa-baja,
 *   usa 'u' (pull-up): lo no conectado queda en 1 y la senal la baja a 0.
 *     u = pull-up    p = pull-down (defecto)    n = sin pull (flotante, ruidoso)
 *     s = volcado completo de todos los pines    r = reset del MCU
 *
 * ⚠ EXCLUIDOS a proposito (leerlos como GPIO rompe la placa):
 *     · PA2 / PA3  = USART2, el VCP del ST-Link -> es el 'Serial' del monitor.
 *     · PA13/ PA14 = SWD (SWDIO/SWCLK) -> debugger y flasheo.
 *
 * Lanzar con:  pio run -e pin_walker -t upload
 * Monitor:     pio device monitor -e pin_walker
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

// Ultimo nivel leido de cada pin (para detectar flancos).
int nivelPrev[numPines];

// Modo de pull actual (para el texto del volcado).
const char* modoPull = "PULL-DOWN";

// Cadencia de muestreo: 20 ms filtra rebotes finos sin perder pulsaciones.
#define SCAN_MS   20UL

void configurarPull(int mode, const char* nombre) {
  for (int i = 0; i < numPines; i++) pinMode(pines[i], mode);
  modoPull = nombre;
  // Re-sincroniza el estado previo para no soltar 41 "flancos" falsos al
  // cambiar de pull.
  delay(2);
  for (int i = 0; i < numPines; i++) nivelPrev[i] = digitalRead(pines[i]);
  Serial.printf("[PULL] entradas -> %s\n", nombre);
}

void volcarTodos() {
  Serial.printf("\n=== SNAPSHOT (pull=%s) ===\n", modoPull);
  for (int i = 0; i < numPines; i++) {
    Serial.printf("%-5s=%d%s", nombresPines[i], digitalRead(pines[i]),
                  ((i % 6) == 5) ? "\n" : "  ");
  }
  Serial.println(F("\n========================="));
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  while (Serial.available()) Serial.read();
  switch (c) {
    case 'u': configurarPull(INPUT_PULLUP,   "PULL-UP");   break;
    case 'p': configurarPull(INPUT_PULLDOWN, "PULL-DOWN"); break;
    case 'n': configurarPull(INPUT,          "SIN PULL (flotante)"); break;
    case 's': volcarTodos(); break;
    case 'r': Serial.println(F("Restart...")); delay(100); NVIC_SystemReset(); break;
    default: break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Arranca en pull-down: lo no conectado queda en 0, una senal activa-alta
  // salta a 1. configurarPull deja pinMode + nivelPrev sincronizados.
  configurarPull(INPUT_PULLDOWN, "PULL-DOWN");

  Serial.println(F("=== RASTREADOR DE ENTRADAS ==="));
  Serial.println(F("Inyecta una senal y mira que pin salta. PA2/3 y PA13/14 excluidos."));
  Serial.println(F("Cmd: u=pull-up  p=pull-down  n=sin pull  s=snapshot  r=reset"));
  volcarTodos();
}

void loop() {
  handleSerial();

  static unsigned long tScan = 0;
  if (millis() - tScan < SCAN_MS) return;
  tScan = millis();

  for (int i = 0; i < numPines; i++) {
    int nivel = digitalRead(pines[i]);
    if (nivel != nivelPrev[i]) {
      Serial.printf("%-5s: %d -> %d  @%lu ms\n",
                    nombresPines[i], nivelPrev[i], nivel, millis());
      nivelPrev[i] = nivel;
    }
  }
}