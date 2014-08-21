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
	if (gpio)
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
}

void     QDN_Pin::Init() {
	if (gpio)
	{
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
}

void  QDN_Pin::HighSpeedInit() {
	if (gpio)
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_StructInit(&GPIO_InitStructure);
	#ifdef STM32F10X_XL
		GPIO_InitStructure.GPIO_Mode  = mode;
	#else
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	#endif
		GPIO_InitStructure.GPIO_Pin   = (pinMask);
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		QDN_GPIO_Init(gpio, &GPIO_InitStructure);
	}
}

const QDN_Pin::Port QDN_Pin::GetPort()
{
#ifdef STM32F10X_XL
	uint32_t offset = reinterpret_cast<uint32_t>(gpio) - reinterpret_cast<uint32_t>(GPIOA);
	offset >>= 10;
	offset ++;
	return static_cast<Port>(offset);
#else
#error
#endif
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
	if (gpio) {
		return (gpio->ODR & pinMask) != 0;
	} else {
		return false;
	}
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

void QDN_OutputPin::Toggle()
{
	if(gpio)
	{
#ifdef STM32F10X_XL
		gpio->ODR ^= pinMask;
#else
#error not implemented yet
#endif
	}
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

///////////////////////////////////////////////////////////////////

QDN_ExternalInterrupt::QDN_ExternalInterrupt(GPIO_TypeDef* gpio0, int pin0)
: QDN_InputPin(gpio0,pin0, GPIO_Mode_IN_FLOATING,1)
{

}

static ISR_t ext0_IRQHandler;
static ISR_t ext1_IRQHandler;
static ISR_t ext2_IRQHandler;
static ISR_t ext3_IRQHandler;
static ISR_t ext4_IRQHandler;

QDN_ExternalInterrupt& QDN_ExternalInterrupt::SetCallback(ISR_t callback)
{
    ext0_IRQHandler = callback;
    return *this;
}

void QDN_ExternalInterrupt::Init()
{
    QDN_Pin::Init();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    QDN_Pin::Port port = GetPort();
    uint8_t portSource = static_cast<int>(port) - 1;
    GPIO_EXTILineConfig(portSource, pinNum);

    EXTI_InitTypeDef   EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line    = pinMask;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitTypeDef   NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = EXTI0_IRQn + pinNum;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_DEFAULT;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void QDN_ExternalInterrupt::Enable()
{
    EXTI->RTSR |= pinMask;
}

void QDN_ExternalInterrupt::Disable()
{
    EXTI->RTSR &= ~pinMask;
}

extern "C" void EXTI0_IRQHandler()
{
    if ((EXTI->PR & EXTI_Line0)  && (EXTI->IMR & EXTI_Line0))
    {
        ext0_IRQHandler();
        EXTI->PR = EXTI_Line0;
    }
}

extern "C" void EXTI1_IRQHandler()
{
    if(EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        ext1_IRQHandler();

        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}
extern "C" void EXTI2_IRQHandler()
{
    if(EXTI_GetITStatus(EXTI_Line2) != RESET)
    {
        ext2_IRQHandler();

        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}
extern "C" void EXTI3_IRQHandler()
{
    if(EXTI_GetITStatus(EXTI_Line3) != RESET)
    {
        ext3_IRQHandler();

        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}
extern "C" void EXTI4_IRQHandler()
{
    if(EXTI_GetITStatus(EXTI_Line4) != RESET)
    {
        ext4_IRQHandler();

        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}


