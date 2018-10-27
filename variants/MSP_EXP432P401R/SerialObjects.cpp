/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ti/runtime/wiring/HardwareSerial.h>

#include <ti/drivers/UART.h>

#include "Board.h"

void uartReadCallback(UART_Handle uart, void *buf, size_t count)
{
    Serial.readCallback(uart, buf, count);
}

void uartReadCallback1(UART_Handle uart, void *buf, size_t count)
{
    Serial1.readCallback(uart, buf, count);
}

#ifdef ENERGIA_BOARD_SERIAL2_ENABLE
void uartReadCallback2(UART_Handle uart, void *buf, size_t count)
{
    Serial2.readCallback(uart, buf, count);
}
#endif

#ifdef ENERGIA_BOARD_SERIAL3_ENABLE
void uartReadCallback3(UART_Handle uart, void *buf, size_t count)
{
    Serial3.readCallback(uart, buf, count);
}
#endif




void uartWriteCallback(UART_Handle uart, void *buf, size_t count)
{
    Serial.writeCallback(uart, buf, count);
}

void uartWriteCallback1(UART_Handle uart, void *buf, size_t count)
{
    Serial1.writeCallback(uart, buf, count);
}

#ifdef ENERGIA_BOARD_SERIAL2_ENABLE
void uartWriteCallback2(UART_Handle uart, void *buf, size_t count)
{
    Serial2.writeCallback(uart, buf, count);
}
#endif

#ifdef ENERGIA_BOARD_SERIAL3_ENABLE
void uartWriteCallback3(UART_Handle uart, void *buf, size_t count)
{
    Serial3.writeCallback(uart, buf, count);
}
#endif


/*
 * Pre-Initialize Serial instances
 */
HardwareSerial Serial(Board_UARTA0, uartReadCallback, uartWriteCallback);
HardwareSerial Serial1(Board_UARTA2, uartReadCallback1, uartWriteCallback1);

#ifdef ENERGIA_BOARD_SERIAL2_ENABLE
HardwareSerial Serial2(Board_UARTA1, uartReadCallback2, uartWriteCallback2);
#endif


#ifdef ENERGIA_BOARD_SERIAL3_ENABLE
HardwareSerial Serial3(Board_UARTA3, uartReadCallback3, uartWriteCallback3);
#endif
