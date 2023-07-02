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

#ifndef PANTALLA_H
#define PANTALLA_H

/** \brief Pantalla
 **
 **
 ** \addtogroup pantalla PANTALLA
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

// Definiciones de bits asociados a cada segmento para construir los digitos
#define SEGMENT_A (1 << 0)
#define SEGMENT_B (1 << 1)
#define SEGMENT_C (1 << 2)
#define SEGMENT_D (1 << 3)
#define SEGMENT_E (1 << 4)
#define SEGMENT_F (1 << 5)
#define SEGMENT_G (1 << 6)
#define SEGMENT_P (1 << 7)

/* === Public data type declarations =========================================================== */

//! Referencia a un descriptor para gestionar una pantalla de siete segmentos mulplezada
typedef struct display_s * display_t;

//! Funcion de callback para apagar los segmentos y digitos de la pantalla
typedef void (*display_screen_off_t)(void);

//! Funcion de callback para prender los segmentos de la pantalla
typedef void (*display_segment_on_t)(uint8_t segment);

//! Funcion de callback para prender un digito de la pantalla
typedef void (*display_digit_on_t)(uint8_t digit);

//! Estructura con las funciones de bajo nivel para manejo de la pantalla
typedef struct display_driver_s {
    display_screen_off_t ScreenTurnOff;  //!< Función para apagar los segmentos y los digitos
    display_segment_on_t SegmentsTurnOn; //!< Funcion para prender determinados segmentos
    display_digit_on_t DigitTurnOn;      //!< Funcion para prender un digito
} const * const display_driver_t;        //!< Puntero al controlador de la pantalla

/* === Public variable declarations ============================================================ */

/* === Public function declarations ============================================================ */

/**
 * @brief Metodo para crear una pantalla multiplexada de siete segmentos
 *
 * @param digits        Cantidad de digitos que forman la pantalla multiplexada
 * @param driver        Puntero a la estructura con las funcionesde bajo nivel
 * @return display_t    Puntero al descriptor de la pantalla creada
 */
display_t DisplayCreate(uint8_t digits, display_driver_t driver);

/**
 * @brief Funcion para escribir un numero BCD en la pantalla de siete segmentos
 *
 * @param display   Puntero al descriptor de la pantalla en la que se escribe
 * @param number    Puntero a el primero elemento de numero BCD a escribir
 * @param size      Cantidad de elementos en el vector que contienen al numero BCD
 */
void DisplayWriteBCD(display_t display, uint8_t * number, uint8_t size);

/**
 * @brief Funcion para refrescar la pantalla
 *
 * @param display  Puntero al descriptor de la pantalla que se debe refrescar
 */
void DisplayRefresh(display_t display);

/**
 * @brief Funcion para hacer parpadear algunos digitos de la pantalla
 *
 * @param display Puntero al descriptor de la pantalla con la que se quiere operar
 * @param from Posicion del primer digito que se quiere hacer parpadear
 * @param to Posicion del ultimo digito que se quiere hacer parpadear
 * @param frecuency Factor de division de la frecuencia de refresco para el parpadeo
 */
void DisplayFlashDigits(display_t display, uint8_t from, uint8_t to, uint16_t frecuency);

/* === End of documentation ==================================================================== */

#ifdef __cplusplus
}
#endif

/** @} End of module definition for doxygen */

#endif /* PANTALLA_H */
