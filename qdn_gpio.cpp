#include "qdn_gpio.h"

void QDN_Pin::HiZ()
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

void     QDN_Pin::Init() {
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

//////////////////


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
    return GPIO_ReadOutputDataBit(gpio,pinMask) != 0;
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
