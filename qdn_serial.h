/**************************************************************************
 *
 * Copyright (c) 2013, Qromodyn Corporation
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the FreeBSD Project.
 **************************************************************************/

#ifndef _QDN_SERIAL_COMMS_H
#define _QDN_SERIAL_COMMS_H

#include "qdn_xos.h"
#include "qdn_cpu.h"
#ifdef STM32F10X_XL
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "qdn_stm32f10x.h"
#else
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "qdn_stm32f4xx.h"
#endif
#include <stdint.h>
#include "qdn_gpio.h"
#include "qdn_project.h"

#ifndef USE_RTOS
#define BARE_TX_QUEUE
#define BARE_RX_QUEUE
#endif

#if defined(BARE_TX_QUEUE) || defined(BARE_RX_QUEUE)
#ifndef BARE_QUEUE_SIZE
#define BARE_QUEUE_SIZE 128
#endif

typedef struct {
    volatile uint8_t buffer[BARE_QUEUE_SIZE];
    volatile uint32_t wptr;
    volatile uint32_t rptr;
    int overflows;
} BareQueue_t;
#endif

#define  SOFTWARE_RTS  1

typedef struct {
    IRQn_Type      irqn;
    uint8_t        altFunc;
} USART_Helper_t;

typedef struct {
    USART_TypeDef* uart;

    QDN_OutputPin*     txPin;
    QDN_InputPin*      rxPin;
    QDN_OutputPin*     rtsPin;
    QDN_InputPin*      ctsPin;

    int32_t       interruptPriority;
    USART_Helper_t helper;
    /* The queue used to hold received characters. */
#ifdef BARE_RX_QUEUE
    BareQueue_t rxQueue;
#else
    XOS_FixedQueue_t rxQueue;
#endif
#if SOFTWARE_RTS == 1
    uint16_t lowWater;
    uint16_t highWater;
#endif
#ifdef BARE_TX_QUEUE
    BareQueue_t txQueue;
#else
    XOS_FixedQueue_t txQueue;
#endif
} ComPortHandle_t;


QDN_EXTERN_C
USART_Helper_t       QDN_USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct);


ComPortHandle_t*     QDN_SerialPortInit        ( USART_TypeDef* uart,  uint32_t ulWantedBaud, uint32_t uxQueueLength ); // configure and enable
ComPortHandle_t*     QDN_SerialPortInitEx      ( USART_TypeDef* uart,  uint32_t ulWantedBaud, uint32_t uxQueueLength ); // configure but don't enable
void                 QDN_SerialPortEnable      ( ComPortHandle_t* pxPort );
void                 QDN_SerialPutString       ( ComPortHandle_t* pxPort, const uint8_t * const pcString );
uint32_t             QDN_SerialWriteBuffer     ( ComPortHandle_t* pxPort, const uint8_t * const buffer, uint16_t usStringLength );
int32_t              QDN_SerialGetChar         ( ComPortHandle_t* pxPort, uint8_t* pcRxedChar, int32_t xBlockTime );
int32_t              QDN_SerialPutChar         ( ComPortHandle_t* pxPort, uint8_t  cOutChar );
int32_t              QDN_SerialWaitForSemaphore( ComPortHandle_t* xPort );
void                 QDN_SerialClose           ( ComPortHandle_t* xPort );
#ifdef DEBUG_UART
void                 QDN_SerialDebugWrite      (const char* data);
#else
#define              QDN_SerialDebugWrite(x)
#endif

QDN_END_EXTERN_C

#endif

