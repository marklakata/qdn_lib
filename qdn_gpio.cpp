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

#include "qdn_gpio.h"

void QDN_Pin::HiZ()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
#ifdef STM32F10X_XL
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
#else
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
#endif
    GPIO_InitStructure.GPIO_Pin   = (pinMask);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    QDN_GPIO_Init(gpio, &GPIO_InitStructure);
}

void     QDN_Pin::Init() {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
#ifdef STM32F10X_XL
    GPIO_InitStructure.GPIO_Mode  = mode;
#else
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
#endif
    GPIO_InitStructure.GPIO_Pin   = (pinMask);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    QDN_GPIO_Init(gpio, &GPIO_InitStructure);
}

//////////////////

__IO uint32_t QDN_OutputPin::dummyRegister = 0;

QDN_GPIO_Output:: QDN_GPIO_Output(GPIO_TypeDef* gpio0, int pin0)
#ifdef STM32F10X_XL
	: QDN_OutputPin(gpio0,pin0,GPIO_Mode_Out_PP,gpio0->BSRR,gpio0->BRR)
#else
	: QDN_OutputPin(gpio0,pin0,GPIO_OType_PP,gpio0->BSRRL,gpio0->BSRRH)
#endif
{
}


#if 0
void QDN_GPIO_Output::Init( ) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
#ifdef STM32F10X_XL
    GPIO_InitStructure.GPIO_Mode  = mode;
#else
    GPIO_InitStructure.GPIO_OType = (GPIOOType_TypeDef) otype;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
#endif
    GPIO_InitStructure.GPIO_Pin   = (pinMask);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    QDN_GPIO_Init(gpio, &GPIO_InitStructure);
}
#endif


bool QDN_OutputPin::IsAsserted() {
    return GPIO_ReadOutputDataBit(gpio,pinMask) != 0;
}


QDN_GPIO_OutputN::QDN_GPIO_OutputN(GPIO_TypeDef* gpio0, int pin0)
#ifdef STM32F10X_XL
	: QDN_OutputPin(gpio0,pin0,GPIO_Mode_Out_PP,gpio0->BRR,gpio0->BSRR)
#else
	: QDN_OutputPin(gpio0,pin0,GPIO_OType_PP,gpio0->BSRRH,gpio0->BSRRL)
#endif
{
}



QDN_GPIO_OpenDrainN::QDN_GPIO_OpenDrainN(GPIO_TypeDef* gpio0, int pin0)
#ifdef STM32F10X_XL
	: QDN_OutputPin(gpio0,pin0,GPIO_Mode_Out_OD,gpio0->BRR,gpio0->BSRR)
#else
	: QDN_OutputPin(gpio0,pin0,GPIO_OType_OD,gpio0->BSRRH,gpio0->BSRRL)
#endif
{
}

///-------------------------------------------------------------------------

bool     QDN_InputPin::IsAsserted() {
    return (GPIO_ReadInputDataBit(gpio,pinMask) == polarity);
}

QDN_GPIO_Input::QDN_GPIO_Input(GPIO_TypeDef* gpio0, int pin0)
: QDN_InputPin(gpio0,pin0, GPIO_Mode_IPD,1)
{

}

#if 0
void     QDN_GPIO_Input::Init() {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
#ifdef STM32F10X_XL
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;
#else
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
#endif
    GPIO_InitStructure.GPIO_Pin   = (pinMask);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    QDN_GPIO_Init(gpio, &GPIO_InitStructure);
}

#endif



#if 0
void     QDN_GPIO_Input::HiZ() {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
#ifdef STM32F10X_XL
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
#else
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
#endif
    GPIO_InitStructure.GPIO_Pin   = (pinMask);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    QDN_GPIO_Init(gpio, &GPIO_InitStructure);
}
#endif


QDN_GPIO_InputN::QDN_GPIO_InputN(GPIO_TypeDef* gpio0, int pin0)
: QDN_InputPin(gpio0, pin0,GPIO_Mode_IPD,0)
{

}

/*
{ GPIO_Mode_AIN = 0x0,
  GPIO_Mode_IN_FLOATING = 0x04,
  GPIO_Mode_IPD = 0x28,
  GPIO_Mode_IPU = 0x48,
  GPIO_Mode_Out_OD = 0x14,
  GPIO_Mode_Out_PP = 0x10,
  GPIO_Mode_AF_OD = 0x1C,
  GPIO_Mode_AF_PP = 0x18
*/
