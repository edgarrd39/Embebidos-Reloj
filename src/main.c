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

#define EVENT_TEC1_ON (1 << 0)
#define EVENT_TEC2_ON (1 << 1)
#define EVENT_TEC3_ON (1 << 2)
#define EVENT_TEC4_ON (1 << 3)

#define EVENT_TEC1_OFF (1 << 4)
#define EVENT_TEC2_OFF (1 << 5)
#define EVENT_TEC3_OFF (1 << 6)
#define EVENT_TEC4_OFF (1 << 7)
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

typedef struct parametros_s {
    uint16_t tecla;
    digital_output_t pin;
    uint32_t tiempo;
} * parametros_t;
/* === Private variable declarations =========================================================== */

/* === Private function declarations =========================================================== */
void ActivarAlarma(void);
/* === Public variable definitions ============================================================= */

static board_t board;

static clock_t reloj;

static modo_t modo;

EventGroupHandle_t key_events;

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

static void KeyTask(void * parameters) {
    board_t board = parameters;
    uint8_t estado_final, estado_actual, changes, events;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(150));

        estado_actual = 0;
        if (DigitalInputGetState(board->accept)) {
            estado_actual |= EVENT_TEC1_ON;
        }

        if (DigitalInputGetState(board->cancel)) {
            estado_actual |= EVENT_TEC2_ON;
        }

        if (DigitalInputGetState(board->set_time)) {
            estado_actual |= EVENT_TEC3_ON;
        }

        if (DigitalInputGetState(board->set_alarm)) {
            estado_actual |= EVENT_TEC4_ON;
        }

        changes = estado_actual ^ estado_final;
        estado_final = estado_actual;
        events = ((changes & !estado_actual) << 4) | (changes & estado_actual);
        xEventGroupSetBits(key_events, events);
    }
}

static void FlashTask(void * parameters) {
    parametros_t parametros = parameters;
    // TickType_t last_value = xTaskGetTickCount();

    while (true) {
        xEventGroupWaitBits(key_events, parametros->tecla, TRUE, FALSE, portMAX_DELAY);

        DigitalOutputActivate(parametros->pin);
        vTaskDelay(pdMS_TO_TICKS(parametros->tiempo));
        DigitalOutputDesactivate(parametros->pin);
        // vTaskDelayUntil(&last_value, pdMS_TO_TICKS(1000));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* === Public function implementation ========================================================= */

int main(void) {
    // uint8_t entrada[4];

    // reloj = ClockCreate(TICS_POR_SEGUNDO / 50, ActivarAlarma);
    board = BoardCreate();

    SisTick_Init(TICS_POR_SEGUNDO);
    // CambiarModo(SIN_CONFIGURAR);

    static struct parametros_s parametros[3];
    parametros[0].tecla = EVENT_TEC1_ON;
    parametros[0].pin = board->led_verde;
    parametros[0].tiempo = 500;

    parametros[1].tecla = EVENT_TEC2_ON;
    parametros[1].pin = board->led_amarillo;
    parametros[1].tiempo = 250;

    parametros[2].tecla = EVENT_TEC3_ON;
    parametros[2].pin = board->led_rojo;
    parametros[2].tiempo = 750;

    key_events = xEventGroupCreate();

    xTaskCreate(KeyTask, "Teclas", 256, (void *)board, tskIDLE_PRIORITY + 1, NULL);

    xTaskCreate(FlashTask, "Verde", 256, &parametros[0], tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(FlashTask, "Amarillo", 256, &parametros[1], tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(FlashTask, "Rojo", 256, &parametros[2], tskIDLE_PRIORITY + 2, NULL);

    vTaskStartScheduler();
    while (true) {
    }
    return 0;
}

/* === End of documentation ==================================================================== */

/** @} End of module definition for doxygen */
