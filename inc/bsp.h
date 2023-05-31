/************************************************************************************************
Copyright (c) <year>, <copyright holders>

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

#ifndef BSP_H
#define BSP_H

/** \brief Board Hardware Support (BSP)
 **
 ** Este módulo se encarga de proporcionar las configuraciones de entrada y salida para la placa EDU-CIIA-NXP
 ** \addtogroup bsp BSP
 ** \brief Board Hardware Support
 ** @{ */

/* === Headers files inclusions ================================================================ */

#include <digital.h>
/* === Cabecera C++ ============================================================================ */

#ifdef __cplusplus
extern "C" {
#endif

/* === Public macros definitions =============================================================== */

/* === Public data type declarations =========================================================== */
typedef struct board_s {
    digital_output_t led_azul;
    digital_output_t led_rojo;
    digital_output_t led_amarillo;
    digital_output_t led_verde;

    digital_input_t tecla_1;
    digital_input_t tecla_2;
    digital_input_t tecla_3;
    digital_input_t tecla_4;
} const * const board_t;
/* === Public variable declarations ============================================================ */

/* === Public function declarations ============================================================ */

board_t BoardCreate(void);
/* === End of documentation ==================================================================== */

#ifdef __cplusplus
}
#endif

/** @} End of module definition for doxygen */

#endif /* BSP_H */
