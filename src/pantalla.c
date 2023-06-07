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

/** \brief Pantalla
 **
 **
 ** \addtogroup pantalla PANTALLA
 ** \brief Hardware Abstraction layer
 ** @{ */

/* === Headers files inclusions =============================================================== */

#include "pantalla.h"
#include "chip.h"
#include <string.h>

/* === Macros definitions ====================================================================== */

#ifndef DISPLAY_MAX_DIGITS
#define DISPLAY_MAX_DIGITS 8
#endif
/* === Private data type declarations ========================================================== */

struct display_s {
    uint8_t digits;
    uint8_t active_digit;
    uint8_t memory[DISPLAY_MAX_DIGITS];
    struct display_driver_s driver[1];
};
/* === Private variable declarations =========================================================== */

static const uint8_t IMAGES[] = {
    SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F,             //!< 0
    SEGMENT_B | SEGMENT_C,                                                             //!< 1
    SEGMENT_A | SEGMENT_B | SEGMENT_D | SEGMENT_E | SEGMENT_G,                         //!< 2
    SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_G,                         //!< 3
    SEGMENT_B | SEGMENT_C | SEGMENT_F | SEGMENT_G,                                     //!< 4
    SEGMENT_A | SEGMENT_C | SEGMENT_D | SEGMENT_F | SEGMENT_G,                         //!< 5
    SEGMENT_A | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G,             //!< 6
    SEGMENT_A | SEGMENT_B | SEGMENT_C,                                                 //!< 7
    SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G, //!< 8
    SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_F | SEGMENT_G              //!< 9
};
/* === Private function declarations =========================================================== */

//! Funcion para asignar un descriptor para crear una nueva pantalla de siete segmentos
static display_t DisplayAllocate(void);

/* === Public variable definitions ============================================================= */

/* === Private variable definitions ============================================================ */

/* === Private function implementation ========================================================= */

display_t DisplayAllocate(void) {
    static struct display_s instances[1] = {0};

    return &instances[0];
}

/* === Public function implementation ========================================================== */

display_t DisplayCreate(uint8_t digits, display_driver_t driver) {
    display_t display = DisplayAllocate();

    if (display) {
        display->digits = digits;
        display->active_digit = digits - 1;
        memcpy(display->driver, driver, sizeof(display->driver));
        memset(display->memory, 0, sizeof(display->memory));
        display->driver->ScreenTurnOff();
    }
    return display;
}

void DisplayWriteBCD(display_t display, uint8_t * number, uint8_t size) {
    memset(display->memory, 0, sizeof(display->memory));
    for (int index = 0; index < size; index++) {
        if (index == display->digits)
            break;
        display->memory[index] = IMAGES[number[index]];
    }
}

void DisplayRefresh(display_t display) {
    display->driver->ScreenTurnOff();
    display->active_digit = (display->active_digit + 1) % display->digits;
    display->driver->SegmentsTurnOn(display->memory[display->active_digit]);
    display->driver->DigitTurnOn(display->active_digit);
}
/* === End of documentation ==================================================================== */

/** @} End of module definition for doxygen */
