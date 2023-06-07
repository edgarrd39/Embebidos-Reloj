/************************************************************************************************
Copyright (c) 2023, <'Edgardo Rodrigo Díaz' mail: rodrigo.09tuc@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

SPDX-License-Identifier: MIT
*************************************************************************************************/

#ifndef DIGITAL_H
#define DIGITAL_H

/** \brief Digital inputs/outputs
 **
 **
 ** \addtogroup hal HAL
 ** \brief Hardware Abstraction layer
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

//! Referencia a un descriptor para gestionar una salida digital
typedef struct digital_output_s * digital_output_t;

//! Referencia a un descriptor para gestionar una entrada digital
typedef struct digital_input_s * digital_input_t;

/* === Public variable declarations ============================================================ */

/* === Public function declarations ============================================================ */

/**
 * @brief Método para crear una salida digital
 *
 * @param port  Puerto GPIO que contiene a la salida
 * @param pin   Número del terminal del puerto GPIO asignado a la salida
 * @param inverted  Indica si se esta usando logica inversa
 * @return digital_output_t     Puntero al descriptor de la salida creada
 */
digital_output_t DigitalOutputCreate(uint8_t port, uint8_t pin, bool inverted);

/**
 * @brief Método para prender una salida digital
 *
 * @param output    Puntero al descriptor de salida
 */
void DigitalOutputActivate(digital_output_t output);

/**
 * @brief Método para apagar una salida digital
 *
 * @param output    Puntero al descriptor de salida
 */
void DigitalOutputDesactivate(digital_output_t output);

/**
 * @brief Método para invertir el estado de una salida digital
 *
 * @param output    Puntero al descriptor de salida
 */
void DigitalOutputToggle(digital_output_t output);

// Entradas ↓

/**
 * @brief Método para crear una entrada digital
 *
 * @param port      Puerto GPIO que contiene a la entrada
 * @param pin       Número del terminal del puerto GPIO asignado a la entrada
 * @param inverted  Indica si se esta usando logica inversa
 * @return digital_input_t  Puntero al descriptor a la salida creada
 */
digital_input_t DigitalInputCreate(uint8_t port, uint8_t pin, bool inverted);

/**
 * @brief Método para consultar el estado de una entrada digital
 *
 * @param input     Puntero al descriptor de la entrada
 * @return true     La entrada está activa
 * @return false    La entrada está inactiva
 */
bool DigitalInputGetState(digital_input_t input);

/**
 * @brief Método para consultar cambios en el estado de una entrada digital
 *
 * @param input     Puntero al descriptor de la entrada
 * @return true     La entrada tuvo cambios desde la ultima llamada
 * @return false    La entrada no tuvo cambios desde la ultima llamada
 */
bool DigitalInputHasChanged(digital_input_t input);

/**
 * @brief Método para consultar activaciones en una entrada digital
 *
 * @param input     Puntero al descriptor de la entrada
 * @return true     La entrada tuvo activaciones desde la última llamada
 * @return false    La entrada no tuvo activaciones desde la ultima llamada
 */
bool DigitalInputHasActivated(digital_input_t input);

/**
 * @brief Método para consultar desactivaciones en una entrada digital
 *
 * @param input     Puntero al descriptor de la entrada
 * @return true     La entrada tuvo desactivaciones desde la última llamada
 * @return false    La entrada no tuvo desactivaciones desde la ultima llamada
 */
bool DigitalInputHasDeactivated(digital_input_t input);

/* === End of documentation ==================================================================== */

#ifdef __cplusplus
}
#endif

/** @} End of module definition for doxygen */

#endif /* DIGITAL_H */
