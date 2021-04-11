/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MBED_PINNAMES_H
#define MBED_PINNAMES_H

#include "cmsis.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PIN_INPUT,
    PIN_OUTPUT
} PinDirection;

typedef enum {
    // Not connected
    NC = (int)0xFFFFFFFF,

    P0_0 = 0,
    P0_1 = 1,
    P0_2 = 2,
    P0_3 = 3,
    P0_4 = 4,
    P0_5 = 5,
    P0_6 = 6,
    P0_7 = 7,
    P0_8 = 8,
    P0_9 = 9,
    P0_10 = 10,
    P0_11 = 11,
    P0_12 = 12,
    P0_13 = 13,
    P0_14 = 14,
    P0_15 = 15,
    P0_16 = 16,
    P0_17 = 17,
    
    D0 = P0_0,
    D1 = P0_4,
    D2 = P0_6,
    D3 = P0_8,
    D4 = P0_9,
    D5 = NC,
    D6 = NC,
    
    D7 = P0_7,
    D8 = P0_17,
    D9 = P0_16,
    D10 = P0_13,
    D11 = P0_14,
    D12 = P0_15,
    D13 = P0_12,
    D14 = P0_10,
    D15 = P0_11,
    
    A0 = NC,
    A1 = NC,
    A2 = NC,
    A3 = NC,
    A4 = P0_10,
    A5 = P0_11,
    
    // LPC800-MAX board
    LED_RED = P0_11,
    
    // mbed original LED naming
    LED1 = LED_RED,
    
    // Serial to USB pins
    CONSOLE_TX = P0_6,
    CONSOLE_RX = P0_1,

} PinName;

typedef enum {
    PullUp = 2,
    PullDown = 1,
    PullNone = 0,
    Repeater = 3,
    OpenDrain = 4,
    PullDefault = PullDown
} PinMode;

#define STDIO_UART_TX     USBTX
#define STDIO_UART_RX     USBRX

typedef struct {
    unsigned char n;
    unsigned char offset;
} SWM_Map;

#ifdef __cplusplus
}
#endif

#endif
