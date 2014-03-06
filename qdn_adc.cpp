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


#include "qdn_adc.h"

#include "qdn_gpio.h"

QDN_ADC_Pin::QDN_ADC_Pin(GPIO_TypeDef* gpio0, int pin0)
#ifdef STM32F10X_XL
: QDN_Pin(gpio0, pin0, GPIO_Mode_AIN)
#else
: QDN_Pin(gpio0, pin0, GPIO_Mode_AN broken)
#endif
, Channel(0)
{
    if (gpio == GPIOA) {
        switch(pin0) {
        case 0: Channel = ADC_Channel_0; break;
        case 3: Channel = ADC_Channel_3; break;
        case 5: Channel = ADC_Channel_5; break;
        default: while(1);
        }
    } else if (gpio == GPIOC) {
        switch(pin0) {
        case 0: Channel = ADC_Channel_10; break;
        default: while(1);
        }
    } else {
        while(1);
    }

}

void  QDN_ADC_Pin::Init() {
	((QDN_Pin*)this)->Init();

#if 0
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
#ifdef STM32F10X_XL
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN_FLOATING;
#else
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
#endif
    GPIO_InitStructure.GPIO_Pin   = (pinMask);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    QDN_GPIO_Init(gpio, &GPIO_InitStructure);
#endif
}

