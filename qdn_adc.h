#ifndef _QDN_ADC_H_
#define _QDN_ADC_H_

#include "qdn_gpio.h"

class QDN_ADC_Pin : QDN_Pin {
public:
    QDN_ADC_Pin(GPIO_TypeDef* gpio0, int pin0);
    void Init();
    uint16_t Channel;
};

#endif
