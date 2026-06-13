//==============================================================================
// Tests nativos del FanController (curva sobre Tmax + feed-forward).
//   Lanzar:  pio test -e native
// update() es determinista (no usa millis ni HW: _apply va bajo #ifdef ARDUINO),
// así que la curva se testea directamente por el duty que devuelve.
// Umbrales por defecto: OFF 32 · ON 35 · FULL 50 · MIN 30 · FF_I 100 · FF_DUTY 50.
//==============================================================================
#include <unity.h>
#include "FanController.h"

void setUp(void)    {}
void tearDown(void) {}

// Fail-safe → 100 % aunque el pack esté frío.
void test_failsafe_100(void) {
    FanController f(0);
    TEST_ASSERT_EQUAL_UINT8(100, f.update(20.0f, 0.0f, true));
}

// Histéresis: arranca a ON≥35, se mantiene hasta caer por debajo de OFF<32.
void test_histeresis_off_on(void) {
    FanController f(0);
    TEST_ASSERT_EQUAL_UINT8(0,  f.update(33.0f, 0.0f, false));  // <35 y estaba OFF → OFF
    TEST_ASSERT_EQUAL_UINT8(30, f.update(35.0f, 0.0f, false));  // ≥35 → ON (duty mín)
    TEST_ASSERT_EQUAL_UINT8(30, f.update(33.0f, 0.0f, false));  // 32≤33<35 → sigue ON (histéresis)
    TEST_ASSERT_EQUAL_UINT8(0,  f.update(31.0f, 0.0f, false));  // <32 → OFF
}

// Rampa lineal: en el punto medio (42.5 °C) → 30 + 0.5·(100−30) = 65 %.
void test_rampa_punto_medio(void) {
    FanController f(0);
    f.update(40.0f, 0.0f, false);   // enciende
    TEST_ASSERT_EQUAL_UINT8(65, f.update(42.5f, 0.0f, false));
}

// Saturación: Tmax ≥ FAN_T_FULL (50 °C) → 100 %.
void test_saturacion_100(void) {
    FanController f(0);
    f.update(40.0f, 0.0f, false);   // enciende
    TEST_ASSERT_EQUAL_UINT8(100, f.update(50.0f, 0.0f, false));
    TEST_ASSERT_EQUAL_UINT8(100, f.update(70.0f, 0.0f, false));
}

// Feed-forward: |I| alta fuerza ON + piso de duty aunque Tmax esté baja.
void test_feedforward_corriente(void) {
    FanController f(0);
    // Frío (20 °C → OFF) pero corriente alta (150 > 100) → fuerza duty ≥ 50.
    TEST_ASSERT_EQUAL_UINT8(50, f.update(20.0f, 150.0f, false));
    // También en carga (corriente negativa).
    FanController g(0);
    TEST_ASSERT_EQUAL_UINT8(50, g.update(20.0f, -150.0f, false));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_failsafe_100);
    RUN_TEST(test_histeresis_off_on);
    RUN_TEST(test_rampa_punto_medio);
    RUN_TEST(test_saturacion_100);
    RUN_TEST(test_feedforward_corriente);
    return UNITY_END();
}
