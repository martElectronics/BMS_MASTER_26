//==============================================================================
// Tests nativos del SocEstimator (coulomb counting + re-snap OCV).
//   Lanzar:  pio test -e native
// Blinda los arreglos P2 (voltsReliable), P3 (re-snap por tiempo) y el gateo de
// coulomb por iReliable. El tiempo se inyecta (begin/update reciben `now`).
// Firma: update(I, minCellV, voltsReliable, iReliable, now).
//==============================================================================
#include <unity.h>
#include "SocEstimator.h"

void setUp(void)    {}
void tearDown(void) {}

// _ocvToSoc (vía begin + socF): clamps en los extremos e interpolación.
void test_ocv_clamps_e_interpolacion(void) {
    SocEstimator s(1.0f);
    s.begin(2.50f, 0); TEST_ASSERT_FLOAT_WITHIN(0.01f,   0.0f, s.socF()); // V≤3.00 → 0%
    s.begin(4.30f, 0); TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, s.socF()); // V≥4.18 → 100%
    s.begin(3.74f, 0); TEST_ASSERT_FLOAT_WITHIN(0.01f,  50.0f, s.socF()); // punto de tabla
    s.begin(3.78f, 0); TEST_ASSERT_FLOAT_WITHIN(0.30f,  55.0f, s.socF()); // mitad de 3.74↔3.82
}

// Coulomb counting: descarga baja, carga sube, cantidad correcta, y clamp [0,100].
// capAh=1.0 → (I·dt)/(1·3600)·100. I=36 A, dt=1 s → 1.0 %.
void test_coulomb_descarga_carga_clamp(void) {
    SocEstimator s(1.0f);
    s.begin(3.74f, 0);                              // SOC = 50
    s.update(36.0f, 3.74f, false, true, 1000);      // descarga 1 s → −1 %
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 49.0f, s.socF());
    s.update(-36.0f, 3.74f, false, true, 2000);     // carga 1 s → +1 %
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 50.0f, s.socF());
    s.update(1.0e5f, 3.74f, false, true, 3000);     // descarga enorme → clamp 0
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, s.socF());
    s.update(-1.0e5f, 3.74f, false, true, 4000);    // carga enorme → clamp 100
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, s.socF());
}

// iReliable=false → el coulomb counting NO integra (Hall railado no arrastra SOC).
void test_i_no_fiable_no_integra(void) {
    SocEstimator s(1.0f);
    s.begin(3.74f, 0);                              // SOC = 50
    s.update(36.0f, 3.74f, false, false, 1000);     // I NO fiable → no integra
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, s.socF());
    s.update(36.0f, 3.74f, false, true, 2000);      // I fiable → ahora sí (−1 %)
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 49.0f, s.socF());
}

// dt fuera de rango (debe ser >0 y <5 s) → no integra (filtra saltos del reloj).
void test_dt_fuera_de_rango_no_integra(void) {
    SocEstimator s(1.0f);
    s.begin(3.74f, 0);                              // SOC = 50, _lastMs = 0
    s.update(36.0f, 3.74f, false, true, 6000);      // dt = 6 s ≥ 5 → NO integra
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, s.socF());
    s.update(36.0f, 3.74f, false, true, 6000);      // dt = 0 → NO integra
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, s.socF());
}

// Regresión P3: el re-snap NO converge de golpe — un paso de dt pequeño mueve
// el SOC muy poco (tau ~50 s), no salta al OCV. El bug por-iteración movería ~1 %.
void test_p3_resnap_no_converge_de_golpe(void) {
    SocEstimator s(1.0f);
    s.begin(4.18f, 0);                              // SOC = 100
    s.update(0.0f, 3.74f, true, true, 1);           // arranca el contador de reposo
    s.update(0.0f, 3.74f, true, true, 30001);       // pasa los 30 s → 1er nudge (dt grande)
    float before = s.socF();
    s.update(0.0f, 3.74f, true, true, 30101);       // dt pequeño (0.1 s)
    float delta = before - s.socF();
    TEST_ASSERT_TRUE(delta > 0.0f);                 // se mueve hacia 50 (baja)
    TEST_ASSERT_TRUE(delta < 0.2f);                 // pero MUY poco (no salta; el bug daría ~1)
}

// Regresión P2: con voltsReliable=false NO hay re-snap, aunque esté en reposo.
void test_p2_sin_resnap_si_no_fiable(void) {
    SocEstimator s(1.0f);
    s.begin(4.18f, 0);                              // SOC = 100
    s.update(0.0f, 3.74f, false, true, 1);
    s.update(0.0f, 3.74f, false, true, 40000);      // >30 s en reposo, pero V NO fiable
    s.update(0.0f, 3.74f, false, true, 40100);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, s.socF());  // sigue en 100 (no tira a 50)
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_ocv_clamps_e_interpolacion);
    RUN_TEST(test_coulomb_descarga_carga_clamp);
    RUN_TEST(test_i_no_fiable_no_integra);
    RUN_TEST(test_dt_fuera_de_rango_no_integra);
    RUN_TEST(test_p3_resnap_no_converge_de_golpe);
    RUN_TEST(test_p2_sin_resnap_si_no_fiable);
    return UNITY_END();
}
