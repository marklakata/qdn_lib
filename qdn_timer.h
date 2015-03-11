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

#ifndef _QDN_Timer_H_
#define _QDN_Timer_H_

#include "qdn_gpio.h"

class QDN_Timer;

#ifdef STM32F10X_XL
#include "stm32f10x_tim.h"
#endif


class QDN_Timer
{
protected:
	QDN_Timer();
public:
	QDN_Timer(int id);
protected:
#ifdef STM32F10X_XL
	TIM_TypeDef* timer;
	uint8_t channel;
#else
#error
#endif
	void Init();
	void SetFrequencyHerz(float freq);
    void SetCallback(void (*function)(void));
public:
    void Start();
    void Stop();
    void EnableIRQ();
    bool DisableIRQ();            ///> Returns the previous state of the interrupts before disabling
    void RestoreIRQ(bool state); ///> Returns the IRQ enable to the state before disabling.

protected:
#ifdef STM32F10X_XL
	TIM_TimeBaseInitTypeDef baseInit;
	TIM_OCInitTypeDef outputControl;
#else
#error
#endif
};

class QDN_EventGenerator : public QDN_Timer
{
public:
	QDN_EventGenerator(int timerId);
	QDN_EventGenerator(int timerId, void (*function0)(void));
	void Init();
	QDN_EventGenerator& SetFrequencyHerz(float freq)        { QDN_Timer::SetFrequencyHerz(freq); return *this; }
	QDN_EventGenerator& SetCallback(void (*function)(void)) { QDN_Timer::SetCallback(function); return * this; }
	void Start(void);
private:
	void (*function)(void);
};

class QDN_PulseGenerator : public QDN_Timer
{
public:
	QDN_PulseGenerator(QDN_OutputPin& output);
	QDN_PulseGenerator(int timerId, QDN_OutputPin& output);
	void Init();
	QDN_PulseGenerator& SetDutyCycle(uint32_t onCounts, uint32_t offCounts);
	QDN_PulseGenerator& SetFrequencyHerz(float freq) { QDN_Timer::SetFrequencyHerz(freq); return *this; }
	QDN_PulseGenerator& SetDutyCycle(float fraction);  // fraction of time that output is asserted
	QDN_PulseGenerator& SetRisingEdgeCallback(void (*func)(void));  // fraction of time that output is asserted
	QDN_PulseGenerator& SetFallingEdgeCallback(void (*func)(void));  // fraction of time that output is asserted
private:
	QDN_OutputPin& outputPin;

};

class QDN_Delay
{
public:
    /**
     * @brief Inserts a delay time in uS.
     * @param delay_us: specifies the delay time in micro second.
     * @retval None
     */
    #define STM32_SYSCLK 72000000
    static void DelayUs(uint32_t delay_us)
    {
        uint32_t nb_loop;
        nb_loop = (((STM32_SYSCLK / 1000000)/4)*delay_us)+1; /* uS (divide by 4 because each loop take about 4 cycles including nop +1 is here to avoid delay of 0 */
        asm volatile(
                "1: " "\n\t"
                " nop " "\n\t"
                " subs.w %0, %0, #1 " "\n\t"
                " bne 1b " "\n\t"
                : "=r" (nb_loop)
                : "0"(nb_loop)
                : "r3"
        );
    }
};

#endif
