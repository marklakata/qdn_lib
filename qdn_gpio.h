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

#ifndef _QDN_GPIO_H_
#define _QDN_GPIO_H_

#include "qdn_util.h"

#ifdef STM32F10X_XL
#include "qdn_stm32f10x.h"
#include "stm32f10x_gpio.h"
#else
#include "qdn_stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#endif

#ifdef STM32F10X_XL
#define MODE_t GPIOMode_TypeDef
#else
#define MODE_t uint8_t
#endif

class QDN_Pin
{
protected:
	QDN_Pin(GPIO_TypeDef* gpio0, int pin0, MODE_t mode0) :
			gpio(gpio0), pinMask(1 << pin0), pinNum(pin0), mode(mode0)
	{
	}
public:
	void Init();
	void HighSpeedInit();
	void HiZ();
#ifndef STM32F10X_XL
	void SetAF(uint8_t altFun)
	{
		GPIO_PinAFConfig(gpio, pinNum, altFunc);
	}
#endif

	enum class Port { A=1, B=2, C=3, D=4, E=5, F=6, G=7, H=8};
	constexpr inline static uint16_t PinHash(const Port port, const int pin)
	{
	    return (static_cast<uint16_t>(port) << 8) | pin;
	}
	uint16_t GetPinHash() const
	{
		return (static_cast<int>(GetPort())<< 8) | pinNum;
	}
	const Port GetPort() const;
	const GPIO_TypeDef* GetGPIO() const { return gpio;}
	const int GetPinNum() const { return pinNum;}
	void SetMode(MODE_t mode0) { mode = mode0; }
protected:
	GPIO_TypeDef* gpio;
	uint16_t pinMask;
	uint8_t pinNum;

protected:
	MODE_t mode;
};

class QDN_OutputPin: public QDN_Pin
{
public:
	QDN_OutputPin(GPIO_TypeDef* gpio0, int pin0, MODE_t mode0,
	__IO uint32_t& assertReg0, __IO uint32_t& deassertReg0, uint8_t polarity0)
        : QDN_Pin(gpio0, pin0, mode0), assertReg(assertReg0), deassertReg(deassertReg0), polarity(polarity0)
	{
	}
	QDN_OutputPin(bool /* dummy*/)
	    : QDN_Pin((GPIO_TypeDef*) 0, 0, (MODE_t) 0), assertReg(dummyRegister), deassertReg(dummyRegister)
	{

	}

	inline void Assert()
	{
		assertReg = pinMask;
	}
	inline void Assert(bool v)
	{
		if (v)
			Assert();
		else
			Deassert();
	}
	inline void Deassert()
	{
		deassertReg = pinMask;
	}
	void Toggle();

	bool IsAsserted() const;
private:
	__IO uint32_t& assertReg;__IO uint32_t& deassertReg;
	uint8_t polarity;
    static __IO uint32_t dummyRegister;
};

class QDN_GPIO_Output: public QDN_OutputPin
{
public:
	QDN_GPIO_Output(GPIO_TypeDef* gpio0, int pin0);
};

class QDN_GPIO_OutputN: public QDN_OutputPin
{
public:
	QDN_GPIO_OutputN(GPIO_TypeDef* gpio0, int pin0);
};

class QDN_GPIO_OpenDrainN: public QDN_OutputPin
{
public:
	QDN_GPIO_OpenDrainN(GPIO_TypeDef* gpio0, int pin0);
};

////////////////////////////////////////////////////////////////////

class QDN_InputPin: public QDN_Pin
{
public:
	QDN_InputPin(GPIO_TypeDef* gpio0, int pin0, MODE_t mode0, uint8_t polarity0) :
			QDN_Pin(gpio0, pin0, mode0), polarity(polarity0)
	{

	}
	bool IsAsserted() const;
private:
	uint8_t polarity;
};

class QDN_GPIO_Input: public QDN_InputPin
{
public:
	QDN_GPIO_Input(GPIO_TypeDef* gpio0, int pin0);
};

class QDN_GPIO_InputN: public QDN_InputPin
{
public:
	QDN_GPIO_InputN(GPIO_TypeDef* gpio0, int pin0);
};


class QDN_ExternalInterrupt : public QDN_InputPin
{
public:
	QDN_ExternalInterrupt(GPIO_TypeDef* gpio0, int pin0);
	QDN_ExternalInterrupt& SetCallback(ISR_t callback);
	void Enable();
	void Disable();
	void Init();
};


#endif
