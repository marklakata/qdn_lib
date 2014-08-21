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

#ifndef _QDN_STM32F10x_H_
#define _QDN_STM32F10x_H_

#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"

#include "qdn_cpu.h"

QDN_EXTERN_C
// this STM32 wrapper routines also enable the correct AHB or APB clocks as needed
// It makes the initialization more "black box".

void QDN_GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
void QDN_VectorInit(void); // assigns the vector table to the section .intvec

///
/// Enables DMA such that n ADC channels are written directly to a uint32[] struct
///
void QDN_ADC_ConfigureDMAAndMuxMany(DMA_Channel_TypeDef *dma, volatile uint16_t* dstArray, ADC_TypeDef * adc,...);


/// RTC helper functions
uint64_t QDN_RTC_GetIntegerTimeStamp(void);
void     QDN_RTC_SetTime(uint64_t value64);

struct QDN_RTC_DateTime_s;
uint8_t QDN_RTC_GetBinaryTimeStamp(struct QDN_RTC_DateTime_s* pTimestamp);
void    QDN_GetShortASCIITimeStamp(char* string, int len);

QDN_END_EXTERN_C

#ifdef __cplusplus
struct SpiThing : public SPI_TypeDef
{
};
#endif

#if 1
// fixme
// this should be part of the project config
#define NVIC_PRIORITY_DEFAULT 1
#endif


#endif

