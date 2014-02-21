#ifndef _G8_GPIO_H_
#define _G8_GPIO_H_

#ifdef STM32F10X_XL
#include "qdn_stm32f10x.h"
#include "stm32f10x_gpio.h"
#else
#include "qdn_stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#endif

class QDN_GPIO_Output {
public:
    QDN_GPIO_Output(GPIO_TypeDef* gpio0, int pin0);
public:
    void Init();
    void Assert();
    void Assert(bool v);
    void Deassert();
    void HiZ();
    void Toggle();
    bool IsAsserted();
protected:
    GPIO_TypeDef* gpio;
    int pinMask; 
#ifdef STM32F10X_XL
    GPIOMode_TypeDef mode;
#else
    uint8_t otype;
#endif
};

class QDN_GPIO_OutputN : public QDN_GPIO_Output {
public:
    QDN_GPIO_OutputN(GPIO_TypeDef* gpio0, int pin0);
    void Deassert();
    void Assert();
};

class QDN_GPIO_OpenDrainN : public QDN_GPIO_OutputN {
public:
    QDN_GPIO_OpenDrainN(GPIO_TypeDef* gpio0, int pin0);
};

class QDN_GPIO_Input {
public:
    QDN_GPIO_Input(GPIO_TypeDef* gpio0, int pin0);
    void Init();
    bool IsAsserted();
    void HiZ();
protected:
    GPIO_TypeDef* gpio;
    int pinMask;
};

class QDN_GPIO_InputN : public QDN_GPIO_Input {
public:
    QDN_GPIO_InputN(GPIO_TypeDef* gpio0, int pin0);
    bool IsAsserted();
};

#endif
