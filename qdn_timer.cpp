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

#include "qdn_timer.h"
#include "qdn_gpio.h"
#include "qdn_util.h"

#include "misc.h"
#include "qdn_stm32f10x.h"
#include "stm32f10x_tim.h"


QDN_Timer::QDN_Timer()
: timer(0)
, channel(0)
{
    TIM_TimeBaseStructInit(&baseInit);
}

QDN_Timer::QDN_Timer(int timerId)
{
	switch (timerId)
	{
	case 1: timer = TIM1; break;
    case 2: timer = TIM2; break;
    case 3: timer = TIM3; break;
    case 4: timer = TIM4; break;
    case 5: timer = TIM5; break;
    case 6: timer = TIM6; break;
    case 7: timer = TIM7; break;
    case 8: timer = TIM8; break;
    case 9: timer = TIM9; break;
	}
	channel = 0;

    TIM_TimeBaseStructInit(&baseInit);
}



void QDN_Timer::Init()
{
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE);


#define CASE(x) case reinterpret_cast<uint32_t>(x)
    switch((uint32_t)timer)
	{
	CASE (TIM1): RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); break;
	CASE (TIM2): RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); break;
	CASE (TIM3): RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); break;
	CASE (TIM4): RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); break;
	CASE (TIM5): RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); break;
	CASE (TIM6): RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); break;
	CASE (TIM7): RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); break;
	CASE (TIM8): RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE); break;
	CASE (TIM9): RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE); break;
	default:
		QDN_Exception("not supported");
	}
#undef CASE

    TIM_DeInit(timer); // aka reset periph after the clock is enabled.
}

void QDN_Timer::SetFrequencyHerz(float freq)
{
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);

    uint32_t prescaler = 1;
    double period;
    do {
        period = RCC_Clocks.SYSCLK_Frequency/prescaler/freq;
        baseInit.TIM_Period = static_cast<int>(period + 0.5) - 1;
        baseInit.TIM_Prescaler = (prescaler-1);
        prescaler *= 2;
    } while (period >= 65536) ;
}

void QDN_Timer::Start()
{
    timer->CR1 |= TIM_CR1_CEN;
}

void QDN_Timer::Stop()
{
    timer->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
}

void QDN_Timer::EnableIRQ()
{
    timer->DIER |= TIM_DIER_UIE;
}

void QDN_Timer::RestoreIRQ(bool state)
{
    if (state)
        timer->DIER |=  TIM_DIER_UIE;
    else
        timer->DIER &= ~TIM_DIER_UIE;
}

bool QDN_Timer::DisableIRQ()
{
   bool flag = !! (timer->DIER & TIM_DIER_UIE);
   timer->DIER &= (uint16_t)~TIM_DIER_UIE;
   return flag;
}


typedef void (*ISR_t)(void);

extern "C"
void QDN_NotDefinedException(void)
{
    QDN_Exception("timer callback not defined");
}


ISR_t tim2Callback = QDN_NotDefinedException;
ISR_t tim3Callback = QDN_NotDefinedException;
ISR_t tim4Callback = QDN_NotDefinedException;
ISR_t tim5Callback = QDN_NotDefinedException;
ISR_t tim6Callback = QDN_NotDefinedException;
ISR_t tim7Callback = QDN_NotDefinedException;

void QDN_Timer::AssignCallback(void (*function)(void))
{
#define CASE(x) case reinterpret_cast<uint32_t>(x)
    switch((uint32_t)timer)
    {
    CASE (TIM2): tim2Callback = function; break;
    CASE (TIM3): tim3Callback = function; break;
    CASE (TIM4): tim4Callback = function; break;
    CASE (TIM5): tim5Callback = function; break;
    CASE (TIM6): tim6Callback = function; break;
    CASE (TIM7): tim7Callback = function; break;
    default:
            QDN_Exception("not supported");
    }
#undef CASE
}


////////////////////////////////////////////////////////////////////////

QDN_PulseGenerator::QDN_PulseGenerator(int timerId, QDN_OutputPin& output)
: QDN_PulseGenerator(output)
{
	if (timerId == 1 && timer != TIM1) QDN_Exception("bad config");
	if (timerId == 2 && timer != TIM2) QDN_Exception("bad config");
	if (timerId == 3 && timer != TIM3) QDN_Exception("bad config");
	if (timerId == 4 && timer != TIM4) QDN_Exception("bad config");
	if (timerId == 5 && timer != TIM5) QDN_Exception("bad config");
	if (timerId == 6 && timer != TIM6) QDN_Exception("bad config");
	if (timerId == 7 && timer != TIM7) QDN_Exception("bad config");
	if (timerId == 8 && timer != TIM8) QDN_Exception("bad config");
	if (timerId == 9 && timer != TIM9) QDN_Exception("bad config");
}

QDN_PulseGenerator::QDN_PulseGenerator(QDN_OutputPin& output)
	: QDN_Timer()
    , outputPin(output)
{
	baseInit.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_OCStructInit(&outputControl);
//	outputControl.TIM_OCIdleState = ;
	outputControl.TIM_OCMode      = TIM_OCMode_PWM1;
//	outputControl.TIM_OCNIdleState = ;
//	outputControl.TIM_OCNPolarity = ;
	outputControl.TIM_OCPolarity  = TIM_OCPolarity_High;
	outputControl.TIM_OutputState = TIM_OutputState_Enable;

#define CASE(a,b) case QDN_Pin::PinHash(QDN_Pin::Port::a,b)
	switch(outputPin.GetPinHash())
	{
	CASE(A,0) : timer = TIM2; channel = 1; break;
	CASE(A,1) : timer = TIM2; channel = 2; break;
	CASE(A,2) : timer = TIM2; channel = 3; break;
	CASE(A,3) : timer = TIM2; channel = 4; break;
	CASE(A,6) : timer = TIM3; channel = 1; break;
	CASE(A,7) : timer = TIM3; channel = 2; break;
	CASE(A,8) : timer = TIM1; channel = 1; break; // alt map
	CASE(A,9) : timer = TIM1; channel = 2; break; // alt map
	CASE(A,10): timer = TIM1; channel = 3; break; // alt map
	CASE(A,11): timer = TIM1; channel = 4; break; // alt map

	CASE(B,0) : timer = TIM3; channel = 3; break;
	CASE(B,1) : timer = TIM3; channel = 4; break;
	CASE(B,10): timer = TIM2; channel = 3; break; // alt map
	CASE(B,11): timer = TIM2; channel = 4; break; // alt map

	CASE(C,6) : timer = TIM3; channel = 1; break; // alt map
	CASE(C,7) : timer = TIM3; channel = 2; break; // alt map
	CASE(C,8) : timer = TIM3; channel = 3; break; // alt map
	CASE(C,9) : timer = TIM3; channel = 4; break; // alt map

	CASE(D,12): timer = TIM4; channel = 1; break; // alt map
	CASE(D,13): timer = TIM4; channel = 2; break; // alt map
	CASE(D,14): timer = TIM4; channel = 3; break; // alt map
	CASE(D,15): timer = TIM4; channel = 4; break; // alt map

	CASE(E,9) : timer = TIM1; channel = 1; break; // alt map
	CASE(E,11): timer = TIM1; channel = 2; break; // alt map
	CASE(E,13): timer = TIM1; channel = 3; break; // alt map
	CASE(E,14): timer = TIM1; channel = 4; break; // alt map

	default: QDN_Exception("not supported");
	}
#undef CASE
}

void QDN_PulseGenerator::Init()
{
	QDN_Timer::Init();

	outputPin.SetMode(GPIO_Mode_AF_PP);
//	outputPin.SetAltFunction();
//	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	outputPin.Init();


//    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//	baseInit.TIM_RepetitionCounter = ?;
	TIM_TimeBaseInit(timer, &baseInit);

	if (channel == 1) {
		TIM_OC1Init(timer,&outputControl);
        TIM_OC4PreloadConfig(timer, TIM_OCPreload_Enable);
	}
	if (channel == 2) {
		TIM_OC2Init(timer,&outputControl);
        TIM_OC4PreloadConfig(timer, TIM_OCPreload_Enable);
	}
	if (channel == 3) {
		TIM_OC3Init(timer,&outputControl);
        TIM_OC4PreloadConfig(timer, TIM_OCPreload_Enable);
	}
	if (channel == 4) {
		TIM_OC4Init(timer,&outputControl);
        TIM_OC4PreloadConfig(timer, TIM_OCPreload_Enable);
	}

    TIM_ARRPreloadConfig(timer, ENABLE);

//	TIM_CtrlPWMOutputs()?;
    TIM_Cmd(timer, ENABLE);
}

QDN_PulseGenerator& QDN_PulseGenerator::SetDutyCycle(uint32_t onCounts, uint32_t offCounts)
{
    QDN_Exception("not implemented");
    return *this;
}


QDN_PulseGenerator& QDN_PulseGenerator::SetDutyCycle(float fraction)
{
	outputControl.TIM_Pulse = static_cast<uint32_t>((baseInit.TIM_Period+1)*fraction + 0.5) - 1;
	return *this;
}


////////////////////////////////////////////////////////////////////////////////

QDN_EventGenerator::QDN_EventGenerator(int timerId)
    : QDN_EventGenerator(timerId, nullptr)
{
}

QDN_EventGenerator::QDN_EventGenerator(int timerId, void (*function0)(void))
    : QDN_Timer(timerId)
    , function(function0)
{
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_DEFAULT;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
}

QDN_EventGenerator& QDN_EventGenerator::SetCallback(void (*function)(void),uint8_t preemptionPriority, uint8_t subPriority)
{
    NVIC_InitStructure.NVIC_IRQChannelCmd = (function!=nullptr)?ENABLE : DISABLE;
    QDN_Timer::AssignCallback(function);
    return * this;
}

void QDN_EventGenerator::Init()
{
    QDN_Timer::Init();
    TIM_TimeBaseInit(timer, &baseInit);

#define CASE(x) case reinterpret_cast<uint32_t>(x)
    switch((uint32_t)timer)
    {
    CASE (TIM2): NVIC_InitStructure.NVIC_IRQChannel =  TIM2_IRQn; break;
    CASE (TIM3): NVIC_InitStructure.NVIC_IRQChannel =  TIM3_IRQn; break;
    CASE (TIM4): NVIC_InitStructure.NVIC_IRQChannel =  TIM4_IRQn; break;
    CASE (TIM5): NVIC_InitStructure.NVIC_IRQChannel =  TIM5_IRQn; break;
    CASE (TIM6): NVIC_InitStructure.NVIC_IRQChannel =  TIM6_IRQn; break;
    CASE (TIM7): NVIC_InitStructure.NVIC_IRQChannel =  TIM7_IRQn; break;
    default:
        QDN_Exception("not supported");
    }
#undef CASE

    NVIC_Init(&NVIC_InitStructure);
}

void QDN_EventGenerator::Start()
{
    QDN_Timer::Start();
    EnableIRQ();

#if 0
    TIM_GenerateEvent(timer,TIM_EventSource_Update);
#endif
}

/////////////////////////////////////////////////////////////////////////////////

extern "C" void TIM2_IRQHandler(void);
extern "C" void TIM3_IRQHandler(void);
extern "C" void TIM4_IRQHandler(void);
extern "C" void TIM5_IRQHandler(void);
extern "C" void TIM6_IRQHandler(void);
extern "C" void TIM7_IRQHandler(void);

extern "C" void TIM2_IRQHandler(void)
{
    TIM2->SR &= ~TIM_FLAG_Update;
    tim2Callback();
}

extern "C" void TIM3_IRQHandler(void)
{
    TIM3->SR &= ~TIM_FLAG_Update;
    tim3Callback();
}
extern "C" void TIM4_IRQHandler(void)
{
	tim4Callback();
    TIM4->SR &= ~TIM_FLAG_Update;
}
extern "C" void TIM5_IRQHandler(void)
{
	tim5Callback();
    TIM5->SR &= ~TIM_FLAG_Update;
}
extern "C" void TIM6_IRQHandler(void)
{
	tim6Callback();
    TIM6->SR &= ~TIM_FLAG_Update;
}
extern "C" void TIM7_IRQHandler(void)
{
	tim7Callback();
    TIM7->SR &= ~TIM_FLAG_Update;
}

//TIM1_BRK_TIM9_IRQHandler
//TIM1_UP_TIM10_IRQHandler
//TIM1_TRG_COM_TIM11_IRQHandler
//TIM1_CC_IRQHandler


//TIM8_BRK_TIM12_IRQHandler
//TIM8_UP_TIM13_IRQHandler
//TIM8_TRG_COM_TIM14_IRQHandler
//TIM8_CC_IRQHandler



