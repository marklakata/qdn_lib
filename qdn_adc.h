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

#ifndef _QDN_ADC_H_
#define _QDN_ADC_H_

#include "qdn_gpio.h"

class QDN_ADC;
class QDN_ADC_Pin;

class MyFoo
{
public:
	MyFoo(int unit) {}
};

class QDN_DMA
{
public:
	QDN_DMA(int unit, int channel);
#if 0
	void Init();
	void SetADCtoMem(QDN_ADC& adc, volatile uint16_t* dst, uint32_t size);

private:
	friend QDN_ADC;
	DMA_TypeDef* dma;
	DMA_Channel_TypeDef* dmaChannel;
#endif
};

class QDN_ADC
{
public:
	QDN_ADC(int unit);
    void DMA_Configure(QDN_DMA& dma, volatile uint16_t* dstArray, /*QDN_ADC_Pin*/...);
    void EnableAndCalibrate();
private:
    friend QDN_ADC_Pin;
    friend QDN_DMA;
    int sampleTime;
    ADC_TypeDef* adc;
};

class QDN_ADC_Pin : QDN_Pin {
public:
    QDN_ADC_Pin(GPIO_TypeDef* gpio0, int pin0, QDN_ADC& adc0);
    void Init();
    uint16_t Channel;

    uint16_t ReadOnce();
    QDN_ADC& adc;
};

#endif
