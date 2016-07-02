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

#include <stdarg.h>

#include "qdn_adc.h"
#include "qdn_spi.h"
#include "qdn_gpio.h"
#include "qdn_util.h"
#include "qdn_dma.h"

#include "misc.h"
#include "qdn_stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"

#include "qdn_stm32f10x.h"


// RX SPI
QDN_DMA::QDN_DMA(QDN_SPI& spi, bool tx)
   :counter{0}
{
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_DEFAULT;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;

    if (spi.spi == SPI1)
    {
        dma = DMA1;
        dmaChannel = tx?DMA1_Channel3:DMA1_Channel2;
    }
    else if (spi.spi == SPI2)
    {
        dma = DMA1;
        dmaChannel = tx?DMA1_Channel5:DMA1_Channel4;
    }
    else if (spi.spi == SPI3)
    {
        dma = DMA2;
        dmaChannel = tx?DMA2_Channel2:DMA2_Channel1;
    }
    else
    {
        QDN_Exception("not implemented");
    }

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & spi.spi->DR;
    DMA_InitStructure.DMA_DIR                = tx ? DMA_DIR_PeripheralDST : DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Priority           = tx ? DMA_Priority_Low      : DMA_Priority_High;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
//    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // or DMA_PeripheralDataSize_Byte
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
//    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
}


QDN_DMA::QDN_DMA(QDN_ADC& adc)
    :counter{0}
{
    if (adc.adc == ADC1)
    {
        dma = DMA1;
        dmaChannel = DMA1_Channel1;
    }
    else if (adc.adc == ADC3)
    {
        dma = DMA2;
        dmaChannel = DMA1_Channel5;
    }
    else
    {
        QDN_Exception("not implemented");
    }

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &adc.adc->DR;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Low;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord; //DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular; // DMA_Mode_Normal;
}


void QDN_DMA::Init()
{
    if (dma == DMA1)
    {
        RCC->AHBENR |= RCC_AHBPeriph_DMA1;
    }
    else if (dma == DMA2)
    {
        RCC->AHBENR |= RCC_AHBPeriph_DMA2;
    }
    else
    {
        QDN_Exception();
    }
    DMA_DeInit(dmaChannel);
    DMA_Init(dmaChannel, &DMA_InitStructure);

    if ( NVIC_InitStructure.NVIC_IRQChannelCmd == ENABLE)
    {
        NVIC_Init(&NVIC_InitStructure);
        dmaChannel->CCR |=  DMA_CCR1_TCIE ; // note: the flag is the same for CCR1 to CCR7
    }
    else
    {
        dmaChannel->CCR &= ~DMA_CCR1_TCIE ;
    }
}

QDN_DMA& QDN_DMA::SetMemory(volatile uint16_t* dst, uint32_t numElements, uint32_t unitSize)
{
    switch(unitSize )
    {
    case 1:
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
        break;
    case 2:
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
        DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
        break;
    case 4:
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
        DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Word;
        break;
    }
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t) dst;


    DMA_InitStructure.DMA_BufferSize         = numElements;
    counter = numElements;

    return *this;
}

void QDN_DMA::Enable()
{
    dmaChannel->CCR |= DMA_CCR1_EN;
}

void QDN_DMA::Disable()
{
    dmaChannel->CCR &= ~DMA_CCR1_EN;
}

void QDN_DMA::DisableAndRearm()
{
    dmaChannel->CCR   &= ~DMA_CCR1_EN;
    dmaChannel->CNDTR  = counter;
}

void QDN_DMA::DisableAndRearm(volatile void* dest)
{
    dmaChannel->CCR   &= ~DMA_CCR1_EN;
    dmaChannel->CMAR   = reinterpret_cast<uint32_t>(dest);
    dmaChannel->CNDTR  = counter;
}
static ISR_t dma11_IRQHandler;
static ISR_t dma12_IRQHandler;
static ISR_t dma13_IRQHandler;
static ISR_t dma14_IRQHandler;
static ISR_t dma15_IRQHandler;
static ISR_t dma16_IRQHandler;
static ISR_t dma17_IRQHandler;
static ISR_t dma21_IRQHandler;
static ISR_t dma22_IRQHandler;
static ISR_t dma23_IRQHandler;
static ISR_t dma24_IRQHandler;



extern "C" void DMA1_Channel1_IRQHandler(void)
{
    dma11_IRQHandler();
    DMA1->IFCR = DMA_ISR_TCIF1;
}

extern "C" void DMA1_Channel2_IRQHandler(void)
{
    dma12_IRQHandler();
    DMA1->IFCR = DMA_ISR_TCIF2;
}

extern "C" void DMA1_Channel3_IRQHandler(void)
{
    dma13_IRQHandler();
    DMA1->IFCR = DMA_ISR_TCIF3;
}

extern "C" void DMA1_Channel4_IRQHandler(void)
{
    dma14_IRQHandler();
    DMA1->IFCR = (DMA_IFCR_CGIF4 | DMA_IFCR_CTCIF4 | DMA_IFCR_CHTIF4);
}

extern "C" void DMA1_Channel5_IRQHandler(void)
{
    dma15_IRQHandler();
    DMA1->IFCR = DMA_ISR_TCIF5;
}

extern "C" void DMA1_Channel6_IRQHandler(void)
{
    dma16_IRQHandler();
    DMA1->IFCR = DMA_ISR_TCIF6;
}

extern "C" void DMA1_Channel7_IRQHandler(void)
{
    dma17_IRQHandler();
    DMA1->IFCR = DMA_ISR_TCIF7;
}

extern "C" void DMA2_Channel1_IRQHandler(void)
{
    dma21_IRQHandler();
    DMA2->IFCR = DMA_ISR_TCIF1;
}

extern "C" void DMA2_Channel2_IRQHandler(void)
{
    dma22_IRQHandler();
    DMA2->IFCR = DMA_ISR_TCIF2;
}

extern "C" void DMA2_Channel3_IRQHandler(void)
{
    dma23_IRQHandler();
    DMA2->IFCR = DMA_ISR_TCIF3;
}

extern "C" void DMA2_Channel4_5_IRQHandler(void)
{
    dma24_IRQHandler();
    DMA2->IFCR = DMA_ISR_TCIF4;
}


QDN_DMA& QDN_DMA::SetCallback(ISR_t callback, uint8_t preemptionPriority, uint8_t subPriority)
{
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = preemptionPriority; // NVIC_PRIORITY_DEFAULT;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = subPriority;


    if (dmaChannel == DMA1_Channel1)
    {
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
        dma11_IRQHandler = callback;
    }
    else if (dmaChannel == DMA1_Channel2)
    {
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
        dma12_IRQHandler = callback;
    }
    else if (dmaChannel == DMA1_Channel3)
    {
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
        dma13_IRQHandler = callback;
    }
    else if (dmaChannel == DMA1_Channel4)
    {
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
        dma14_IRQHandler = callback;
    }
    else if (dmaChannel == DMA1_Channel5)
    {
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
        dma15_IRQHandler = callback;
    }
    else if (dmaChannel == DMA1_Channel6)
    {
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
        dma16_IRQHandler = callback;
    }
    else if (dmaChannel == DMA1_Channel7)
    {
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
        dma17_IRQHandler = callback;
    }
    else if (dmaChannel == DMA2_Channel1)
    {
        NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel1_IRQn;
        dma21_IRQHandler = callback;
    }
    else if (dmaChannel == DMA2_Channel2)
    {
        NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel2_IRQn;
        dma22_IRQHandler = callback;
    }
    else if (dmaChannel == DMA2_Channel3)
    {
        NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel3_IRQn;
        dma23_IRQHandler = callback;
    }
    else if (dmaChannel == DMA2_Channel4)
    {
        NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel4_5_IRQn;
        dma24_IRQHandler = callback;
    }
    else if (dmaChannel == DMA2_Channel5)
    {
        NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel4_5_IRQn;
        dma24_IRQHandler = callback;
    }
    else
    {
        QDN_Exception("not implemented");
    }

    NVIC_InitStructure.NVIC_IRQChannelCmd = callback? ENABLE:DISABLE;

    return *this;
}






