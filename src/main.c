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

#define TICS_POR_SEGUNDO 1000
#define TIEMPO_POSPONER 5
#define TIEMPO_MAXIMO_PRESIONAR 3000
#define TIEMPO_MAXIMO_CONFIGURACION TIEMPO_MAXIMO_PRESIONAR * 10

/* === Private data type declarations ========================================================== */

typedef enum {
    SIN_CONFIGURAR,
    MOSTRANDO_HORA,
    AJUSTANDO_MINUTOS_ACTUAL,
    AJUSTANDO_HORAS_ACTUAL,
    AJUSTANDO_MINUTOS_ALARMA,
    AJUSTANDO_HORAS_ALARMA,
} modo_t;

/* === Private variable declarations =========================================================== */

/* === Private function declarations =========================================================== */
void ActivarAlarma(void);
/* === Public variable definitions ============================================================= */

static board_t board;

static clock_t reloj;

static modo_t modo;

static bool alarma_sonando = false;

static uint16_t contador_setear_tiempo = 0;

static uint16_t contador_setear_alarma = 0;

static uint16_t contador_configuracion = 0;

/* === Private variable definitions ============================================================ */

static const uint8_t LIMITES_MINUTOS[] = {5, 9};
static const uint8_t LIMITES_HORAS[] = {2, 3};

/* === Private function implementation ========================================================= */

void ActivarAlarma(void) {
    DigitalOutputActivate(board->buzzer);
    alarma_sonando = true;
}

void CambiarModo(modo_t valor) {
    modo = valor;
    switch (modo) {
    case SIN_CONFIGURAR:
        DisplayFlashDigits(board->display, 0, 3, 200);
        break;
    case MOSTRANDO_HORA:
        DisplayFlashDigits(board->display, 0, 0, 0);
        break;
    case AJUSTANDO_MINUTOS_ACTUAL:
        DisplayFlashDigits(board->display, 2, 3, 200);
        break;
    case AJUSTANDO_HORAS_ACTUAL:
        DisplayFlashDigits(board->display, 0, 1, 200);
        break;
    case AJUSTANDO_MINUTOS_ALARMA:
        DisplayFlashDigits(board->display, 2, 3, 200);
        break;
    case AJUSTANDO_HORAS_ALARMA:
        DisplayFlashDigits(board->display, 0, 1, 200);

        break;
    default:
        break;
    }
}

void IncrementarBCD(uint8_t numero[2], const uint8_t limite[2]) {

    numero[1]++;
    if ((numero[0] >= limite[0]) && (numero[1]) > limite[1]) {
        numero[1] = 0;
        numero[0] = 0;
    }
    if (numero[1] > 9) {
        numero[1] = 0;
        numero[0]++;
    }
}

void DecrementarBCD(uint8_t numero[2], const uint8_t limite[2]) {

    numero[1]--;

    if (numero[1] > 9) {
        numero[1] = 9;
        if (numero[0] > 0)
            numero[0]--;
        else {
            numero[0] = limite[0];
            numero[1] = limite[1];
        }
    }
}

/* === Public function implementation ========================================================= */

int main(void) {
    uint8_t entrada[4];

    reloj = ClockCreate(TICS_POR_SEGUNDO / 50, ActivarAlarma);
    board = BoardCreate();

    SisTick_Init(TICS_POR_SEGUNDO);
    CambiarModo(SIN_CONFIGURAR);

    while (true) {

        if (DigitalInputHasActivated(board->accept)) {

            if (modo == MOSTRANDO_HORA) {
                if (alarma_sonando) {
                    ClockPosponerAlarma(reloj, TIEMPO_POSPONER);
                    DigitalOutputDesactivate(board->buzzer);
                    alarma_sonando = false;
                } else {
                    DigitalOutputActivate(board->buzzer);
                }
            } else if (modo == AJUSTANDO_MINUTOS_ACTUAL) {
                CambiarModo(AJUSTANDO_HORAS_ACTUAL);
            } else if (modo == AJUSTANDO_HORAS_ACTUAL) {
                ClockSetTime(reloj, entrada, sizeof(entrada));
                CambiarModo(MOSTRANDO_HORA);
            } else if (modo == AJUSTANDO_MINUTOS_ALARMA) {
                CambiarModo(AJUSTANDO_HORAS_ALARMA);
            } else if (modo == AJUSTANDO_HORAS_ALARMA) {
                ClockSetAlarma(reloj, entrada, sizeof(entrada));
                CambiarModo(MOSTRANDO_HORA);
                ClockActivarAlarma(reloj);
            }
        }

        if (DigitalInputHasActivated(board->cancel)) {
            if (modo == MOSTRANDO_HORA) {
                if (alarma_sonando) {
                    DisplayToggleDot(board->display, 3);
                    DigitalOutputDesactivate(board->buzzer);
                    ClockCancelarAlarma(reloj);
                    alarma_sonando = false;
                } else {
                    ClockDesactivarAlarma(reloj);
                    DigitalOutputDesactivate(board->buzzer);
                }
            } else if (ClockGetTime(reloj, entrada, sizeof(entrada)) && (modo != MOSTRANDO_HORA)) {
                CambiarModo(MOSTRANDO_HORA);
            } else {
                CambiarModo(SIN_CONFIGURAR);
            }
        }

        if (DigitalInputHasActivated(board->set_alarm)) {
            contador_setear_alarma = 1;
        }
        if (contador_setear_alarma > TIEMPO_MAXIMO_PRESIONAR) {
            contador_setear_alarma = 0;
            contador_configuracion = 1;
            CambiarModo(AJUSTANDO_MINUTOS_ALARMA);
            ClockGetAlarma(reloj, entrada, sizeof(entrada));
            DisplayWriteBCD(board->display, entrada, sizeof(entrada));
        }
        if (DigitalInputHasActivated(board->set_time)) {
            contador_setear_tiempo = 1;
        }

        if (contador_setear_tiempo > TIEMPO_MAXIMO_PRESIONAR) {
            contador_setear_tiempo = 0;
            contador_configuracion = 1;
            CambiarModo(AJUSTANDO_MINUTOS_ACTUAL);
            ClockGetTime(reloj, entrada, sizeof(entrada));
            DisplayWriteBCD(board->display, entrada, sizeof(entrada));
        }
        if (DigitalInputHasActivated(board->increment)) {
            if (modo == AJUSTANDO_MINUTOS_ACTUAL || modo == AJUSTANDO_MINUTOS_ALARMA) {
                contador_configuracion = 1;
                IncrementarBCD(&entrada[2], LIMITES_MINUTOS);
            } else if (modo == AJUSTANDO_HORAS_ACTUAL || modo == AJUSTANDO_HORAS_ALARMA) {
                contador_configuracion = 1;
                IncrementarBCD(entrada, LIMITES_HORAS);
            }
            if ((modo == AJUSTANDO_MINUTOS_ACTUAL) || (modo == AJUSTANDO_HORAS_ACTUAL)) {
                DisplayWriteBCD(board->display, entrada, sizeof(entrada));
            } else if (((modo == AJUSTANDO_MINUTOS_ALARMA) || (modo == AJUSTANDO_HORAS_ALARMA))) {
                DisplayWriteBCD(board->display, entrada, sizeof(entrada));
            }
        }

        if (DigitalInputHasActivated(board->decrement)) {
            if (modo == AJUSTANDO_MINUTOS_ACTUAL || modo == AJUSTANDO_MINUTOS_ALARMA) {
                contador_configuracion = 1;
                DecrementarBCD(&entrada[2], LIMITES_MINUTOS);
            } else if (modo == AJUSTANDO_HORAS_ACTUAL || modo == AJUSTANDO_HORAS_ALARMA) {
                contador_configuracion = 1;
                DecrementarBCD(entrada, LIMITES_HORAS);
            }
            if ((modo == AJUSTANDO_MINUTOS_ACTUAL) || (modo == AJUSTANDO_HORAS_ACTUAL)) {
                DisplayWriteBCD(board->display, entrada, sizeof(entrada));

            } else if (((modo == AJUSTANDO_MINUTOS_ALARMA) || (modo == AJUSTANDO_HORAS_ALARMA))) {
                DisplayWriteBCD(board->display, entrada, sizeof(entrada));
            }
        }

        if (contador_configuracion >= TIEMPO_MAXIMO_CONFIGURACION) {
            contador_configuracion = 0;
            if (ClockGetTime(reloj, entrada, sizeof(entrada))) {
                CambiarModo(MOSTRANDO_HORA);
            } else {
                CambiarModo(SIN_CONFIGURAR);
            }
        }

        for (int index = 0; index < 100; index++) {
            for (int delay = 0; delay < 2500; delay++) {
                __asm("NOP");
            }
        }
    }
}

void SysTick_Handler(void) {

    static bool last_value = false;
    bool current_value;
    uint8_t hora[6];
    DisplayRefresh(board->display);
    current_value = ClockTick(reloj);

    if (current_value != last_value) {

        last_value = current_value;
        if (modo <= MOSTRANDO_HORA) {
            ClockGetTime(reloj, hora, sizeof(hora));
            DisplayWriteBCD(board->display, hora, sizeof(hora));
            if (ClockGetAlarma(reloj, hora, sizeof(hora))) {
                DisplayToggleDot(board->display, 3);
            }
            DisplayToggleDot(board->display, 1);
        }

        if (modo == AJUSTANDO_MINUTOS_ALARMA || modo == AJUSTANDO_HORAS_ALARMA) {
            DisplayToggleDot(board->display, 0);
            DisplayToggleDot(board->display, 1);
            DisplayToggleDot(board->display, 2);
            DisplayToggleDot(board->display, 3);
        }
    }

    if ((DigitalInputGetState(board->set_time)) && (contador_setear_tiempo > 0)) {
        contador_setear_tiempo++;
    }

    if ((DigitalInputGetState(board->set_alarm)) && (contador_setear_alarma > 0)) {
        contador_setear_alarma++;
    }

    if (!(DigitalInputGetState(board->set_time)) && !(DigitalInputGetState(board->set_alarm)) &&
        !(DigitalInputGetState(board->increment)) && !(DigitalInputGetState(board->decrement)) &&
        contador_configuracion > 0) {
        contador_configuracion++;
    }
}
/* === End of documentation ==================================================================== */

/** @} End of module definition for doxygen */
