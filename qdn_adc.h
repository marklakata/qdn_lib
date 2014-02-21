#ifndef _QDN_ADC_H_
#define _QDN_ADC_H_

// #include "g8_stm32f4xx.h"
//#include "stm32f4xx_gpio.h"
#include "qdn_gpio.h"

class QDN_ADC_Pin {
public:
    QDN_ADC_Pin(GPIO_TypeDef* gpio0, int pin0);
    void Init();
    uint16_t Channel;
protected:
    GPIO_TypeDef* gpio;
    int pinMask;
};

#endif
