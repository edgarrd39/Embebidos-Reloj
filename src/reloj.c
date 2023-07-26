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

/** \brief Brief description of the file
 **
 ** Full file description
 **
 ** \addtogroup name Module denomination
 ** \brief Brief description of the module
 ** @{ */

/* === Headers files inclusions =============================================================== */

#include "reloj.h"
#include <string.h>

/* === Macros definitions ====================================================================== */

#define TIME_SIZE 6

#define SEGUNDOS_UNIDADES 5
#define SEGUNDOS_DECENAS 4
#define MINUTOS_UNIDADES 3
#define MINUTOS_DECENAS 2
#define HORA_UNIDADES 1
#define HORA_DECENAS 0
#define MAX_HR_UNI 4 // Unidad maxima de hora
#define MAX_HR_DEC 2 // Decena maxima de hora

/* === Private data type declarations ========================================================== */

struct clock_s {
    uint8_t hora_actual[TIME_SIZE];
    uint8_t hora_alarma[TIME_SIZE];
    uint8_t hora_pospuesta[TIME_SIZE];
    uint8_t hora_alarma_inicial[TIME_SIZE];
    uint8_t ticks_por_segundo;
    uint8_t ticks;
    alarma_evento_t evento;
    bool valida;
    bool tiene_alarma;
    bool posponer;
};

/* === Private variable declarations =========================================================== */

/* === Private function declarations =========================================================== */
void IncrementarTiempo(uint8_t * hora_actual);

void CompararHoraConAlarma(clock_t reloj);

/* === Public variable definitions ============================================================= */

/* === Private variable definitions ============================================================ */

/* === Private function implementation ========================================================= */

void IncrementarTiempo(uint8_t * hora) {

    hora[SEGUNDOS_UNIDADES]++;

    if (hora[SEGUNDOS_UNIDADES] == 10) {
        hora[SEGUNDOS_UNIDADES] = 0;
        hora[SEGUNDOS_DECENAS]++;
    }
    if (hora[SEGUNDOS_DECENAS] == 6) {
        hora[SEGUNDOS_DECENAS] = 0;
        hora[MINUTOS_UNIDADES]++;
    }
    if (hora[MINUTOS_UNIDADES] == 10) {
        hora[MINUTOS_UNIDADES] = 0;
        hora[MINUTOS_DECENAS]++;
    }
    if (hora[MINUTOS_DECENAS] == 6) {
        hora[MINUTOS_DECENAS] = 0;
        hora[HORA_UNIDADES]++;
    }

    if (hora[HORA_UNIDADES] == 10) {
        hora[HORA_UNIDADES] = 0;
        hora[HORA_DECENAS]++;
    }
    if (hora[HORA_DECENAS] == MAX_HR_DEC && hora[HORA_UNIDADES] == MAX_HR_UNI) {
        memset(hora, 0, TIME_SIZE);
    }
}

void CompararHoraConAlarma(clock_t reloj) {

    if (reloj->tiene_alarma) {
        if (memcmp(reloj->hora_alarma, reloj->hora_actual, TIME_SIZE) == 0)
            reloj->evento();
    }
}
/* === Public function implementation ========================================================= */

clock_t ClockCreate(int ticks_por_segundo, alarma_evento_t evento) {
    static struct clock_s self[1];
    memset(self, 0, sizeof(self));
    self->ticks_por_segundo = ticks_por_segundo;
    self->evento = evento;
    return self;
}

bool ClockGetTime(clock_t reloj, uint8_t * hora, int size) {
    memcpy(hora, reloj->hora_actual, size);
    return reloj->valida;
}

bool ClockSetTime(clock_t reloj, const uint8_t * hora, int size) {
    memcpy(reloj->hora_actual, hora, size);
    reloj->valida = true;
    return true;
}

bool ClockTick(clock_t reloj) {
    reloj->ticks++;
    if (reloj->ticks == reloj->ticks_por_segundo) {
        reloj->ticks = 0;
        IncrementarTiempo(reloj->hora_actual);
        CompararHoraConAlarma(reloj);
    }
    return (reloj->ticks > (reloj->ticks_por_segundo / 2));
}

bool ClockSetAlarma(clock_t reloj, const uint8_t * hora, int size) {
    memcpy(reloj->hora_alarma, hora, size);
    reloj->tiene_alarma = true;
    return true;
}

bool ClockGetAlarma(clock_t reloj, uint8_t * hora, int size) {
    memcpy(hora, reloj->hora_alarma, size);
    return reloj->tiene_alarma;
}

bool ClockGetStateAlarm(clock_t reloj) {
    return reloj->tiene_alarma;
}

void ClockActivarAlarma(clock_t reloj) {
    reloj->tiene_alarma = true;
}

void ClockDesactivarAlarma(clock_t reloj) {
    reloj->posponer = false;
    reloj->tiene_alarma = false;
}

bool ClockToggleAlarm(clock_t reloj) {
    reloj->tiene_alarma = !reloj->tiene_alarma;
    return reloj->tiene_alarma;
}

void ClockPosponerAlarma(clock_t reloj, uint8_t minutos) {
    if (reloj->posponer == false) {
        memcpy(reloj->hora_alarma_inicial, reloj->hora_alarma, 6);
    }
    memcpy(reloj->hora_pospuesta, reloj->hora_alarma, 6);
    if ((reloj->hora_alarma[3] + minutos) > 9) {
        reloj->hora_alarma[2] += 1;
        reloj->hora_alarma[3] -= minutos; // por ahora solo del 1 al 9
        if (reloj->hora_alarma[2] > 5) {
            reloj->hora_alarma[1] += 1;
            reloj->hora_alarma[2] = 0;

            if (reloj->hora_alarma[1] == 10) {
                reloj->hora_alarma[0]++;
                reloj->hora_alarma[1] = 0;
            }
            if (reloj->hora_alarma[0] == 2 && reloj->hora_alarma[1] == 4) {
                reloj->hora_alarma[0] = 0;
                reloj->hora_alarma[1] = 0;
            }
        }
    } else {
        reloj->hora_alarma[MINUTOS_UNIDADES] += minutos;
    }
    reloj->posponer = true;
}

void ClockCancelarAlarma(clock_t reloj) {
    if (reloj->posponer) {
        memcpy(reloj->hora_alarma, reloj->hora_alarma_inicial, 6);
        reloj->posponer = false;
    }
}
/* === End of documentation ==================================================================== */

/** @} End of module definition for doxygen */
