//==============================================================================
// Tests nativos del FaultTimer (debounce K-de-N + ventana).
//   Lanzar:  pio test -e native
// Valida las esquinas que no se ven a ojo: K=2, reset por OK, saturación de
// badRun y overflow de millis() (resta unsigned).
//==============================================================================
#include <unity.h>
#include "FaultTimer.h"

void setUp(void)    {}
void tearDown(void) {}

// Una sola muestra mala NO confirma (badRun=1 < k=2), aunque la ventana sobre.
void test_una_mala_no_confirma(void) {
    FaultTimer f;
    f.sample(true, 0);
    TEST_ASSERT_FALSE(f.confirmed(1000, 500));   // window OK, k NO
}

// Dos consecutivas: confirma solo cuando ADEMÁS se cumple la ventana.
void test_dos_malas_y_ventana(void) {
    FaultTimer f;
    f.sample(true, 0);
    f.sample(true, 250);
    TEST_ASSERT_FALSE(f.confirmed(400, 500));    // k OK, window NO
    TEST_ASSERT_TRUE (f.confirmed(500, 500));    // k y window OK
}

// Un OK rompe la serie: badRun y tStart se reinician.
void test_ok_resetea_la_serie(void) {
    FaultTimer f;
    f.sample(true, 0);
    f.sample(false, 100);                        // OK → reset
    f.sample(true, 200);                         // empieza de nuevo
    TEST_ASSERT_EQUAL_UINT8(1, f.badRun);
    TEST_ASSERT_FALSE(f.confirmed(700, 500));    // solo 1 mala desde el reset
}

// badRun satura a 255 (no da la vuelta a 0) con muchas malas seguidas.
void test_badrun_satura_255(void) {
    FaultTimer f;
    for (int i = 0; i < 300; i++) f.sample(true, (unsigned long)i);
    TEST_ASSERT_EQUAL_UINT8(255, f.badRun);
    TEST_ASSERT_TRUE(f.confirmed(1000, 500));
}

// Overflow de millis(): tStart cerca del máximo y now ya envolvió.
// La resta unsigned (now - tStart) sigue dando el tiempo correcto (600 ms).
// (En el host de 32-bit unsigned long esto ejercita el wrap real de millis()).
void test_overflow_millis(void) {
    FaultTimer f;
    unsigned long t0 = 0xFFFFFF00UL;             // cerca del wrap
    f.sample(true, t0);
    f.sample(true, t0 + 250);                    // badRun=2, tStart=t0
    TEST_ASSERT_TRUE(f.confirmed(t0 + 600, 500)); // now envolvió; elapsed=600 ✓
}

// confirmed() exige AMBAS condiciones: ni solo-k ni solo-ventana bastan.
void test_confirmed_exige_k_y_ventana(void) {
    FaultTimer f;
    f.sample(true, 0);
    TEST_ASSERT_FALSE(f.confirmed(1000, 500));   // window sí, k no
    f.sample(true, 1);
    TEST_ASSERT_FALSE(f.confirmed(1, 500));      // k sí, window no
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_una_mala_no_confirma);
    RUN_TEST(test_dos_malas_y_ventana);
    RUN_TEST(test_ok_resetea_la_serie);
    RUN_TEST(test_badrun_satura_255);
    RUN_TEST(test_overflow_millis);
    RUN_TEST(test_confirmed_exige_k_y_ventana);
    return UNITY_END();
}
