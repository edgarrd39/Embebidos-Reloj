/* Copyright 2022, Laboratorio de Microprocesadores
 * Facultad de Ciencias Exactas y Tecnología
 * Universidad Nacional de Tucuman
 * http://www.microprocesadores.unt.edu.ar/
 * Copyright 2022, Esteban Volentini <evolentini@herrera.unt.edu.ar>
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

/** \brief Simple sample of use LPC HAL gpio functions
 **
 ** \addtogroup samples Sample projects
 ** \brief Sample projects to use as a starting point
 ** @{ */

/* === Headers files inclusions =============================================================== */

#include "bsp.h"
#include "chip.h"
#include "digital.h"
#include "poncho.h"
#include "reloj.h"
#include <stdbool.h>

/* === Macros definitions ====================================================================== */

#define TICS_POR_SEGUNDO 10

/* === Private data type declarations ========================================================== */

typedef enum {
    SIN_CONFIGURAR,
    MOSTRANDO_HORA,
    AJUSTANDO_MINUTOS_ACTUAL,
    AJJUSTANDO_HORAS_ACTUAL,
    AJUSTANDO_MINUTOS_ALARMA,
    AJUSTANDO_HORAS_ALAMA,
} modo_t;

/* === Private variable declarations =========================================================== */

/* === Private function declarations =========================================================== */
void ActivarAlarma(void);
/* === Public variable definitions ============================================================= */

/* === Private variable definitions ============================================================ */
static board_t board;

static clock_t reloj;

static modo_t modo;
/* === Private function implementation ========================================================= */

void ActivarAlarma(void) {
}
/* === Public function implementation ========================================================= */

int main(void) {

    reloj = ClockCreate(TICS_POR_SEGUNDO, ActivarAlarma);
    board = BoardCreate();
    modo = SIN_CONFIGURAR;

    SisTick_Init(1000);
    DisplayFlashDigits(board->display, 0, 3, 200);

    while (true) {

        if (DigitalInputHasActivated(board->accept)) {
        }

        if (DigitalInputHasActivated(board->cancel)) {
        }

        if (DigitalInputHasActivated(board->set_time)) {
        }

        if (DigitalInputHasActivated(board->set_alarm)) {
        }

        if (DigitalInputHasActivated(board->increment)) {
        }

        if (DigitalInputHasActivated(board->decrement)) {
        }

        // para ver el parpadeo se tiene que implementar un delay
        for (int index = 0; index < 100; index++) {
            for (int delay = 0; delay < 2500; delay++) {
                __asm("NOP");
            }
        }
    }
}

void SysTick_Handler(void) {

    static const int medio_seg = TICS_POR_SEGUNDO / 2;
    int valor_actual;
    uint8_t hora[6];
    DisplayRefresh(board->display);
    valor_actual = ClockTick(reloj);

    if (valor_actual == medio_seg || valor_actual == 0) {

        if (modo <= MOSTRANDO_HORA) {
            ClockGetTime(reloj, hora, sizeof(hora));
            DisplayWriteBCD(board->display, hora, sizeof(hora));
            DisplayToggleDot(board->display, 1);
        }
    }
}
/* === End of documentation ==================================================================== */

/** @} End of module definition for doxygen */
