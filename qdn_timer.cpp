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

}
QDN_Timer::QDN_Timer(int timerId)
{
	if (timerId == 1)
	{
		timer = TIM1;
	}
}


#define CASE(x) case reinterpret_cast<uint32_t>(x)

void QDN_Timer::Init()
{
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
}

#undef CASE

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
	TIM_TimeBaseStructInit(&baseInit);
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

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE);

	TIM_DeInit(timer);

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
	return *this;
}

QDN_PulseGenerator& QDN_PulseGenerator::SetFrequencyHerz(float freq)
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
	return *this;
}

QDN_PulseGenerator& QDN_PulseGenerator::SetDutyCycle(float fraction)
{
	outputControl.TIM_Pulse = static_cast<uint32_t>((baseInit.TIM_Period+1)*fraction + 0.5) - 1;
	return *this;
}

void QDN_PulseGenerator::Start()
{
	timer->CR1 |= TIM_CR1_CEN;
}

void QDN_PulseGenerator::Stop()
{
    timer->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
}

////////////////////////////////////////////////////////////////////////////////

QDN_EventGenerator::QDN_EventGenerator(int timerId)
{
}

QDN_EventGenerator::QDN_EventGenerator(int timerId, void (*function0)(void))
: function(function0)
{
}

void QDN_EventGenerator::Init()
{
	QDN_Exception("not supported");
}

QDN_EventGenerator& QDN_EventGenerator::SetFrequencyHerz(float freq)
{
	return *this;
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
}

extern "C" void TIM3_IRQHandler(void)
{
}
extern "C" void TIM4_IRQHandler(void)
{
}
extern "C" void TIM5_IRQHandler(void)
{
}
extern "C" void TIM6_IRQHandler(void)
{
}
extern "C" void TIM7_IRQHandler(void)
{
}


//TIM1_BRK_TIM9_IRQHandler
//TIM1_UP_TIM10_IRQHandler
//TIM1_TRG_COM_TIM11_IRQHandler
//TIM1_CC_IRQHandler


//TIM8_BRK_TIM12_IRQHandler
//TIM8_UP_TIM13_IRQHandler
//TIM8_TRG_COM_TIM14_IRQHandler
//TIM8_CC_IRQHandler



