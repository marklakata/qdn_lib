#ifndef _QDN_GPIO_H_
#define _QDN_GPIO_H_

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


class QDN_Pin {
protected:
	QDN_Pin(GPIO_TypeDef* gpio0, int pin0,MODE_t mode0
			)
		: gpio(gpio0)
		, pinMask(1<<pin0)
		, pinNum(pin0)
		, mode(mode0)
	{
	}
public:
	void Init();
	void HiZ();
#ifndef STM32F10X_XL
	void SetAF(uint8_t altFun)
	{
		GPIO_PinAFConfig(gpio, pinNum, altFunc);
	}

#endif

protected:
    GPIO_TypeDef* gpio;
    uint16_t pinMask;
    uint8_t pinNum;

public:
    MODE_t mode;
};

class QDN_OutputPin  : public QDN_Pin
{
public:
	QDN_OutputPin(GPIO_TypeDef* gpio0, int pin0,MODE_t mode0,
			__IO uint32_t& assertReg0,  __IO uint32_t& deassertReg0)
		: QDN_Pin(gpio0, pin0, mode0)
		, assertReg(assertReg0)
		, deassertReg(deassertReg0)
	{

	}
	void Assert()
	{
		assertReg = pinMask;
	}
    void Assert(bool v)
    {
    	if (v) Assert(); else Deassert();
    }
    void Deassert()
    {
    	deassertReg = pinMask;
    }
    void Toggle()
    {
    	if (IsAsserted()) Deassert(); else Assert();
    }
    bool IsAsserted();
private:
    __IO uint32_t& assertReg;
    __IO uint32_t& deassertReg;
};

class QDN_GPIO_Output : public QDN_OutputPin
{
public:
    QDN_GPIO_Output(GPIO_TypeDef* gpio0, int pin0);
};

class QDN_GPIO_OutputN : public QDN_OutputPin
{
public:
    QDN_GPIO_OutputN(GPIO_TypeDef* gpio0, int pin0);
};

class QDN_GPIO_OpenDrainN : public QDN_OutputPin
{
public:
    QDN_GPIO_OpenDrainN(GPIO_TypeDef* gpio0, int pin0);
};

////////////////////////////////////////////////////////////////////


class QDN_InputPin : public QDN_Pin
{
public:
	QDN_InputPin(GPIO_TypeDef* gpio0, int pin0,MODE_t mode0, uint8_t polarity0)
		: QDN_Pin(gpio0,pin0,mode0)
		, polarity(polarity0)
	{

	}
    bool IsAsserted();
private:
    uint8_t polarity;
};

class QDN_GPIO_Input : public QDN_InputPin
{
public:
    QDN_GPIO_Input(GPIO_TypeDef* gpio0, int pin0);
};

class QDN_GPIO_InputN : public QDN_InputPin {
public:
    QDN_GPIO_InputN(GPIO_TypeDef* gpio0, int pin0);
};

#endif
