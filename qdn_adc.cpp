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




#if 1
QDN_ADC_Pin::QDN_ADC_Pin(GPIO_TypeDef* gpio0, int pin0, QDN_ADC& adc0)
#ifdef STM32F10X_XL
: QDN_Pin(gpio0, pin0, GPIO_Mode_AIN)
#else
: QDN_Pin(gpio0, pin0, GPIO_Mode_AN broken)
GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
#endif
, Channel(0)
, adc(adc0)
{
    if (gpio == GPIOA)
    {
    	if (pin0 >=0 && pin0 < 16)
    		Channel = pin0;
    	else
    		QDN_Exception();
    	if (adc0.adc == ADC3 && pin0 >= 4) QDN_Exception();
	}
    else if (gpio == GPIOB)
    {
		switch(pin0) {
		case 0: Channel = ADC_Channel_8; break;
		case 1: Channel = ADC_Channel_9; break;
        default: QDN_Exception();
        }
    }
    else if (gpio == GPIOC)
    {
        switch(pin0) {
        case 0: Channel = ADC_Channel_10; break;
        case 1: Channel = ADC_Channel_11; break;
        case 2: Channel = ADC_Channel_12; break;
        case 3: Channel = ADC_Channel_13; break;
        case 4: Channel = ADC_Channel_14; break;
        case 5: Channel = ADC_Channel_15; break;
        default: QDN_Exception();
        }
    }
    else if (gpio == GPIOF)
    {
    	if (adc0.adc != ADC3) QDN_Exception();
    	switch(pin0) {
    	case 6: Channel = ADC_Channel_4; break;
    	case 7: Channel = ADC_Channel_5; break;
    	case 8: Channel = ADC_Channel_6; break;
    	case 9: Channel = ADC_Channel_7; break;
    	case 10:Channel = ADC_Channel_8; break;
        default: QDN_Exception();
        }
    }
	else
	{
    	QDN_Exception();
    }

}

void  QDN_ADC_Pin::Init() {
	((QDN_Pin*)this)->Init();
#ifdef STM32F10X_XL
	if (adc.adc == ADC1) {
		RCC->APB2ENR |= (RCC_APB2Periph_ADC1);
	} else if (adc.adc == ADC2) {
		RCC->APB2ENR |= (RCC_APB2Periph_ADC2);
	} else if (adc.adc == ADC3) {
		RCC->APB2ENR |= (RCC_APB2Periph_ADC3);
	} else {
		QDN_Exception();
	}
#else
#error
#endif
}
#endif

QDN_ADC::QDN_ADC(int unit)
: sampleTime(ADC_SampleTime_239Cycles5)
{
	if (unit == 1) adc = ADC1;
	else {
		QDN_Exception();
	}
}


#ifdef DMA_DONE
uint32_t dma1_1Done = 0;
static void dmaDone(void)
{
	dma1_1Done++;
}
#endif

void QDN_ADC::DMA_Configure(QDN_DMA& dma, volatile uint16_t* destinationPtr, const std::vector<const QDN_ADC_Pin*>& adcList)
{
    int numChannels = adcList.size();

    dma
        .SetMemory(destinationPtr,numChannels, sizeof(uint16_t))
#ifdef DMA_DONE
        .SetCallback(dmaDone)
#endif
        .Init();

    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode       = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel       = numChannels;
    ADC_Init(adc, &ADC_InitStructure);

    uint32_t rank = 1;
    for(auto x : adcList )
    {
        ADC_RegularChannelConfig(adc, x->Channel, rank, sampleTime);
        rank++;
    }

    ADC_DMACmd(adc, ENABLE);
    EnableAndCalibrate();
    ADC_SoftwareStartConvCmd(adc, ENABLE);
}

void QDN_ADC::EnableTempSensor()
{
    ADC_TempSensorVrefintCmd(ENABLE);
}


void QDN_ADC::EnableAndCalibrate()
{
    ADC_Cmd(adc, ENABLE);
    ADC_ResetCalibration(adc);
    while(ADC_GetResetCalibrationStatus(adc));
    ADC_StartCalibration(adc);
    while(ADC_GetCalibrationStatus(adc));
}


uint16_t QDN_ADC_Pin::ReadOnce()
{
    return 0;
}
