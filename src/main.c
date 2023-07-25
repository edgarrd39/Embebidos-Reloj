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

#include "FreeRTOS.h"
#include "bsp.h"
#include "chip.h"
#include "digital.h"
#include "event_groups.h"
#include "poncho.h"
#include "reloj.h"
#include "task.h"
#include <stdbool.h>

/* === Macros definitions ====================================================================== */

//! Eventos

#define EVENT_KEY_ACCEPT_ON (1 << 0)
#define EVENT_KEY_CANCEL_ON (1 << 1)
#define EVENT_KEY_SET_TIME_ON (1 << 2)
#define EVENT_KEY_SET_ALARM_ON (1 << 3)
#define EVENT_KEY_INCREMENT_ON (1 << 4)
#define EVENT_KEY_DECREMENT_ON (1 << 5)

#define EVENT_KEY_ACCEPT_OFF (1 << 6)
#define EVENT_KEY_CANCEL_OFF (1 << 7)
#define EVENT_KEY_SET_TIME_OFF (1 << 8)
#define EVENT_KEY_SET_ALARM_OFF (1 << 9)
#define EVENT_KEY_INCREMENT_OFF (1 << 10)
#define EVENT_KEY_DECREMENT_OFF (1 << 11)
//

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

EventGroupHandle_t key_events;

static bool alarma_sonando = false;

static uint8_t entrada[4];

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

static void CambiarModo(modo_t valor) {
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

static void RefreshTask(void * parameters) {

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1));
        DisplayRefresh(board->display);
    }
}

static void SystickTask(void * parameters) {

    bool current_value;
    uint8_t hora[6];
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1));
        static bool last_value = false;
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
    }
}
static void KeyTask(void * parameters) {
    board_t board = parameters;
    uint8_t estado_final, estado_actual, changes, events;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));

        estado_actual = 0;
        if (DigitalInputGetState(board->accept)) {
            estado_actual |= EVENT_KEY_ACCEPT_ON;
        }

        if (DigitalInputGetState(board->cancel)) {
            estado_actual |= EVENT_KEY_CANCEL_ON;
        }

        if (DigitalInputGetState(board->set_time)) {
            estado_actual |= EVENT_KEY_SET_TIME_ON;
        }

        if (DigitalInputGetState(board->set_alarm)) {
            estado_actual |= EVENT_KEY_SET_ALARM_ON;
        }

        if (DigitalInputGetState(board->increment)) {
            estado_actual |= EVENT_KEY_INCREMENT_ON;
        }

        if (DigitalInputGetState(board->decrement)) {
            estado_actual |= EVENT_KEY_DECREMENT_ON;
        }

        changes = estado_actual ^ estado_final;
        estado_final = estado_actual;
        events = ((changes & !estado_actual) << 6) | (changes & estado_actual);
        xEventGroupSetBits(key_events, events);
    }
}

static void AcceptKeyTask(void * parameters) {

    while (true) {
        xEventGroupWaitBits(key_events, EVENT_KEY_ACCEPT_ON, TRUE, FALSE, portMAX_DELAY);

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
}

static void CancelKeyTask(void * parameters) {

    while (true) {
        xEventGroupWaitBits(key_events, EVENT_KEY_CANCEL_ON, TRUE, FALSE, portMAX_DELAY);

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
}

static void SetTimeTask(void * parameters) {

    while (true) {

        xEventGroupWaitBits(key_events, EVENT_KEY_SET_TIME_ON, TRUE, FALSE, portMAX_DELAY);
        CambiarModo(AJUSTANDO_MINUTOS_ACTUAL);
        ClockGetTime(reloj, entrada, sizeof(entrada));
        DisplayWriteBCD(board->display, entrada, sizeof(entrada));
    }
}

static void SetAlarmTask(void * parameters) {
    while (true) {
        xEventGroupWaitBits(key_events, EVENT_KEY_SET_ALARM_ON, TRUE, FALSE, portMAX_DELAY);
        CambiarModo(AJUSTANDO_MINUTOS_ALARMA);
        ClockGetAlarma(reloj, entrada, sizeof(entrada));
        DisplayWriteBCD(board->display, entrada, sizeof(entrada));
    }
}

static void IncrementTimeTask(void * parameters) {
    while (true) {

        xEventGroupWaitBits(key_events, EVENT_KEY_INCREMENT_ON, TRUE, FALSE, portMAX_DELAY);

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
}

static void DecrementTimeTask(void * parameters) {
    while (true) {
        xEventGroupWaitBits(key_events, EVENT_KEY_DECREMENT_ON, TRUE, FALSE, portMAX_DELAY);

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
}
/* === Public function implementation ========================================================= */

int main(void) {

    reloj = ClockCreate(TICS_POR_SEGUNDO / 50, ActivarAlarma);
    board = BoardCreate();

    SisTick_Init(TICS_POR_SEGUNDO);
    CambiarModo(SIN_CONFIGURAR);

    key_events = xEventGroupCreate();

    // Creación de las tareas

    xTaskCreate(RefreshTask, "Refresh", 256, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(SystickTask, "Systick", 256, NULL, tskIDLE_PRIORITY + 1, NULL);

    xTaskCreate(KeyTask, "Teclas", 256, (void *)board, tskIDLE_PRIORITY + 2, NULL);

    xTaskCreate(AcceptKeyTask, "Aceptar", 256, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(CancelKeyTask, "Cancelar", 256, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(SetTimeTask, "SetearTiempo", 256, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(SetAlarmTask, "SetearAlarma", 256, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(IncrementTimeTask, "IncrementarTiempo", 256, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(DecrementTimeTask, "DecrementarTiempo", 256, NULL, tskIDLE_PRIORITY + 3, NULL);

    vTaskStartScheduler();

    while (true) {
    }
    return 0;
}

/* === End of documentation ==================================================================== */

/** @} End of module definition for doxygen */
