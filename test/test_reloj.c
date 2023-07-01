/*Copyright 2023, Edgardo Rodrigo Díaz <rodrigo.09tuc@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* PRUEBAS
// 1.  Al inicializar el reloj está en 00:00 y con hora invalida.
// 2.  Al ajustar la hora el reloj queda en hora y es válida.
// 3. Después de n ciclos de reloj la hora avanza un segundo,
// diez segundos, un minutos, diez minutos, una hora, diez
// horas y un día completo.
// 4. Fijar la hora de la alarma y consultarla.
// 5. Fijar la alarma y avanzar el reloj para que suene.
// 6. Fijar la alarma, deshabilitarla y avanzar el reloj para no suene.
// 7. Hacer sonar la alarma y posponerla.
// 8. Hacer sonar la alarma y cancelarla hasta el otro dia..
*/

/** \brief Brief description of the file
 **
 ** Full file description
 **
 ** \addtogroup name Module denomination
 ** \brief Brief description of the module
 ** @{ */

/* === Headers files inclusions =============================================================== */

#include "reloj.h"
#include "unity.h"

/* === Macros definitions ====================================================================== */

#define TICKS_POR_SEGUNDO 5

/* === Private data type declarations ========================================================== */

/* === Private variable declarations =========================================================== */

/* === Private function declarations =========================================================== */

void SetUp(void);

void ActivarAlarma(void);

/* === Public variable definitions ============================================================= */
static clock_t reloj;
static uint8_t hora[6];
static const uint8_t INICIAL[] = {1, 2, 3, 4, 0, 0};
static bool estado_alarma;
/* === Private variable definitions ============================================================ */

/* === Private function implementation ========================================================= */

/**
 * @brief Funcion que simula el tiempo en segundos
 *
 * @param seconds
 */
void SimulateTime(int seconds) {
    for (int count = 0; count < seconds; count++) {
        for (int index = 0; index < TICKS_POR_SEGUNDO; index++) {
            ClockTick(reloj);
        }
    }
}

void ActivarAlarma(void) {
    estado_alarma = true;
}

void SetUp(void) {
    estado_alarma = false;
    reloj = ClockCreate(TICKS_POR_SEGUNDO, ActivarAlarma);
    ClockSetTime(reloj, INICIAL, sizeof(INICIAL));
}

/* === Public function implementation ========================================================= */

// 1.  Al inicializar el reloj está en 00:00 y con hora invalida.

void test_iniciar_reloj_invalido(void) {
    static uint8_t ESPERADO[] = {0, 0, 0, 0, 0, 0};
    uint8_t hora[] = {0xFF};

    clock_t reloj = ClockCreate(5, ActivarAlarma);
    TEST_ASSERT_FALSE(ClockGetTime(reloj, hora, 6));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(ESPERADO, hora, 6);
}

// 2.  Al ajustar la hora el reloj queda en hora y es válida.

void test_ajustar_hora_valida(void) {
    static uint8_t ESPERADO[] = {1, 2, 3, 4, 0, 0};
    uint8_t hora[6];

    clock_t reloj = ClockCreate(5, ActivarAlarma);
    TEST_ASSERT_TRUE(ClockSetTime(reloj, ESPERADO, 6));
    TEST_ASSERT_TRUE(ClockGetTime(reloj, hora, 6));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(ESPERADO, hora, 6);
}

// 3. Después de n ciclos de reloj la hora avanza un segundo,
// diez segundos, un minutos, diez minutos, una hora, diez
// horas y un día completo.

void test_incrementar_seconds_units(void) {

    SetUp();
    static const uint8_t ESPERADO[] = {1, 2, 3, 4, 0, 1};

    SimulateTime(1);
    TEST_ASSERT_TRUE(ClockGetTime(reloj, hora, 6));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(ESPERADO, hora, 6);
}

void test_incrementar_segundos_decenas(void) {

    SetUp();
    static const uint8_t ESPERADO[] = {1, 2, 3, 4, 1, 0};

    SimulateTime(10);
    TEST_ASSERT_TRUE(ClockGetTime(reloj, hora, 6));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(ESPERADO, hora, 6);
}

void test_incrementar_minutos_unidades(void) {
    SetUp();
    static const uint8_t ESPERADO[] = {1, 2, 3, 5, 0, 0};

    SimulateTime(60);
    TEST_ASSERT_TRUE(ClockGetTime(reloj, hora, 6));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(ESPERADO, hora, 6);
}

void test_incrementar_minutos_decenas(void) {
    SetUp();
    static const uint8_t ESPERADO[] = {1, 2, 4, 4, 0, 0};

    SimulateTime(60 * 10);
    TEST_ASSERT_TRUE(ClockGetTime(reloj, hora, 6));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(ESPERADO, hora, 6);
}

void test_incrementar_horas_unidades(void) {
    SetUp();
    static const uint8_t ESPERADO[] = {1, 3, 3, 4, 0, 0};

    SimulateTime(60 * 60);
    TEST_ASSERT_TRUE(ClockGetTime(reloj, hora, 6));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(ESPERADO, hora, 6);
}

void test_incrementar_horas_decenas(void) {
    SetUp();
    static const uint8_t ESPERADO[] = {2, 2, 3, 4, 0, 0};

    SimulateTime(60 * 60 * 10);
    TEST_ASSERT_TRUE(ClockGetTime(reloj, hora, 6));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(ESPERADO, hora, 6);
}

void test_incrementar_dia(void) {
    SetUp();
    static const uint8_t ESPERADO[] = {1, 2, 3, 4, 0, 0};

    SimulateTime(60 * 60 * 24);
    TEST_ASSERT_TRUE(ClockGetTime(reloj, hora, 6));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(ESPERADO, hora, 6);
}

// 4. Fijar la hora de la alarma y consultarla.

void test_fijar_alarma(void) {
    SetUp();
    static const uint8_t ESPERADO[] = {1, 2, 4, 4, 0, 0};

    TEST_ASSERT_TRUE(ClockSetAlarma(reloj, ESPERADO, 6));
    TEST_ASSERT_TRUE(ClockGetAlarma(reloj, hora, 6));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(ESPERADO, hora, 6);
}

// 5. Fijar la alarma y avanzar el reloj para que suene.

void test_fijar_avanzar_activar_alarma(void) {
    SetUp();
    static const uint8_t ESPERADO[] = {1, 2, 4, 4, 0, 0};

    ClockSetAlarma(reloj, ESPERADO, 6);
    TEST_ASSERT_FALSE(estado_alarma);
    SimulateTime(60 * 10);

    TEST_ASSERT_TRUE(estado_alarma); // La alarma esta sonando
    TEST_ASSERT_EQUAL_UINT8_ARRAY(ESPERADO, hora, 6);
}

// 6. Fijar la alarma, deshabilitarla y avanzar el reloj para no suene.

void test_desactivar_alarma(void) {
    SetUp();
    static const uint8_t ESPERADO[] = {1, 2, 4, 4, 0, 0};
    ClockSetAlarma(reloj, ESPERADO, 6);
    ClockDesactivarAlarma(reloj); // Desactivo Alarma
    SimulateTime(60 * 10);        // avanzo el tiempo en el reloj, pero como deshabilite la alarma, no deberia sonar
    ClockGetTime(reloj, hora, sizeof(hora));

    TEST_ASSERT_FALSE(ClockGetAlarma(reloj, hora, 6));
    TEST_ASSERT_FALSE(estado_alarma); // con esto verifico que no ocurrio el evento de ActivarAlarma
}

// 7. Hacer sonar la alarma y posponerla.
void test_posponer_alarma(void) {
    SetUp();
    static const uint8_t ALARMA[] = {1, 2, 4, 4, 0, 0};
    static const uint8_t ESPERADO[] = {1, 2, 4, 9, 0, 0};
    uint8_t minutos = 5;
    ClockSetAlarma(reloj, ALARMA, 6);

    SimulateTime(60 * 10);
    TEST_ASSERT_TRUE(estado_alarma);                       // La alarma esta sonando
    estado_alarma = false;                                 // la alarma no esta sonando
    TEST_ASSERT_TRUE(ClockPosponerAlarma(reloj, minutos)); // Con esto pospongo alarma

    TEST_ASSERT_FALSE(estado_alarma); // compruebo que la alarma no este sonando
    SimulateTime(60 * 5);
    ClockGetTime(reloj, hora, sizeof(hora));
    TEST_ASSERT_TRUE(estado_alarma);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(ESPERADO, hora, 6);
}

// 8. Hacer sonar la alarma y cancelarla hasta el otro dia.
void test_cancelar_hasta_otro_dia(void) {
    SetUp();
    static const uint8_t ALARMA[] = {1, 2, 4, 4, 0, 0};

    ClockSetAlarma(reloj, ALARMA, 6);
    SimulateTime(60 * 10);
    TEST_ASSERT_TRUE(estado_alarma);
    estado_alarma = false;
    SimulateTime(60 * 60 * 24);
    TEST_ASSERT_TRUE(estado_alarma);
}

// 9 Probar que suena alarma, luego posponer, pero al otro dia debe sonar a la hora que estaba
// configurada originalmente

/* === End of documentation ==================================================================== */

/** @} End of module definition for doxygen */
