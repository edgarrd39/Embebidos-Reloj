/* Copyright 2023, Edgardo Rodrigo Díaz <rodrigo.09tuc@gmail.com>
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

#ifndef reloj_H
#define reloj_H

/** \brief Brief description of the file
 **
 ** Full file description
 **
 ** \addtogroup name Module denomination
 ** \brief Brief description of the module
 ** @{ */

/* === Headers files inclusions ================================================================ */

#include <stdbool.h>
#include <stdint.h>

/* === Cabecera C++ ============================================================================ */

#ifdef __cplusplus
extern "C" {
#endif

/* === Public macros definitions =============================================================== */

/* === Public data type declarations =========================================================== */

typedef struct clock_s * clock_t;

typedef void (*alarma_evento_t)(void);

/* === Public variable declarations ============================================================ */

/* === Public function declarations ============================================================ */

/**
 * @brief
 *
 * @param ticks_por_segundo
 * @return clock_t
 */
clock_t ClockCreate(int ticks_por_segundo, alarma_evento_t evento);

/**
 * @brief
 *
 * @param reloj
 * @param hora
 * @param size
 * @return true
 * @return false
 */
bool ClockGetTime(clock_t reloj, uint8_t hora[], int size);

/**
 * @brief Setea la hora del reloj
 *
 * @param reloj
 * @param hora
 * @param size
 * @return true
 * @return false
 */
bool ClockSetTime(clock_t reloj, const uint8_t * hora, int size);

/**
 * @brief
 *
 * @param reloj
 */
bool ClockTick(clock_t reloj);

/**
 * @brief
 *
 * @param reloj
 * @param hora
 * @param size
 * @return true
 * @return false
 */
bool ClockSetAlarma(clock_t reloj, const uint8_t * hora, int size);

/**
 * @brief
 *
 * @param reloj
 * @param hora
 * @param size
 * @return true
 * @return false
 */
bool ClockGetAlarma(clock_t reloj, uint8_t * hora, int size);

/**
 * @brief
 *
 * @param reloj
 */
void ClockActivarAlarma(clock_t reloj);

/**
 * @brief
 *
 * @param reloj
 */
void ClockDesactivarAlarma(clock_t reloj);

/**
 * @brief
 *
 * @param reloj
 * @param minutos
 * @return true
 * @return false
 */
bool ClockPosponerAlarma(clock_t reloj, uint8_t minutos);

/**
 * @brief
 *
 * @param reloj
 * @return true
 * @return false
 */
bool ClockGetStateAlarm(clock_t reloj);
/* === End of documentation ==================================================================== */

#ifdef __cplusplus
}
#endif

/** @} End of module definition for doxygen */

#endif /* TEMPLATE_H */