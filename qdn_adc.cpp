#include "qdn_adc.h"

#include "qdn_gpio.h"

QDN_ADC_Pin::QDN_ADC_Pin(GPIO_TypeDef* gpio0, int pin0)
#ifdef STM32F10X_XL
: QDN_Pin(gpio0, pin0, GPIO_Mode_AIN)
#else
: QDN_Pin(gpio0, pin0, GPIO_Mode_AN broken)
#endif
, Channel(0)
{
    if (gpio == GPIOA) {
        switch(pin0) {
        case 0: Channel = ADC_Channel_0; break;
        case 3: Channel = ADC_Channel_3; break;
        case 5: Channel = ADC_Channel_5; break;
        default: while(1);
        }
    } else if (gpio == GPIOC) {
        switch(pin0) {
        case 0: Channel = ADC_Channel_10; break;
        default: while(1);
        }
    } else {
        while(1);
    }

}

void  QDN_ADC_Pin::Init() {
	((QDN_Pin*)this)->Init();

#if 0
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
#ifdef STM32F10X_XL
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN_FLOATING;
#else
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
#endif
    GPIO_InitStructure.GPIO_Pin   = (pinMask);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    QDN_GPIO_Init(gpio, &GPIO_InitStructure);
#endif
}

