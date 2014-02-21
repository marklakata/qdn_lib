#include "qdn_gpio.h"


QDN_GPIO_Output:: QDN_GPIO_Output(GPIO_TypeDef* gpio0, int pin0)
	: gpio(gpio0)
	, pinMask(1<<pin0)
#ifdef STM32F10X_XL
	, mode(GPIO_Mode_Out_PP)
#else
	, otype(GPIO_OType_PP)
#endif
{
}


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
void QDN_GPIO_Output::Assert() {
#ifdef STM32F10X_XL
	GPIO_SetBits(gpio,pinMask);
#else
    gpio->BSRRL = pinMask;
#endif
    //GPIO_SetBits(gpio,pinMask);
}
void QDN_GPIO_Output::Assert(bool value) {
    GPIO_WriteBit(gpio,pinMask,value?Bit_SET:Bit_RESET);
}
void QDN_GPIO_Output::Deassert() {
#ifdef STM32F10X_XL
	GPIO_ResetBits(gpio,pinMask);
#else
    gpio->BSRRH = pinMask;
#endif
    //GPIO_ResetBits(gpio,pinMask);
}
void QDN_GPIO_Output::Toggle() {
#ifdef STM32F10X_XL
	BitAction ba = GPIO_ReadInputDataBit(gpio,pinMask);
	Assert(!ba);
#else
    GPIO_ToggleBits(gpio,pinMask);
#endif
}
bool QDN_GPIO_Output::IsAsserted() {
    return GPIO_ReadOutputDataBit(gpio,pinMask) != 0;
}
void QDN_GPIO_Output::HiZ() {
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

///---

QDN_GPIO_OutputN::QDN_GPIO_OutputN(GPIO_TypeDef* gpio0, int pin0)
: QDN_GPIO_Output(gpio0,pin0) {
}

void QDN_GPIO_OutputN::Deassert() {
    gpio->BSRRL = pinMask;
//    GPIO_SetBits(gpio,pinMask);
}
void QDN_GPIO_OutputN::Assert() {
    gpio->BSRRH = pinMask;
//    GPIO_ResetBits(gpio,pinMask);
}

///---

QDN_GPIO_OpenDrainN::QDN_GPIO_OpenDrainN(GPIO_TypeDef* gpio0, int pin0)
: QDN_GPIO_OutputN(gpio0,pin0) {
    otype = GPIO_OType_OD;
}

///-------------------------------------------------------------------------

QDN_GPIO_Input::QDN_GPIO_Input(GPIO_TypeDef* gpio0, int pin0)
	: gpio(gpio0)
	, pinMask(1<<pin0)
{

}

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
bool     QDN_GPIO_Input::IsAsserted() {
    return (GPIO_ReadInputDataBit(gpio,pinMask) != 0);
}
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

QDN_GPIO_InputN::QDN_GPIO_InputN(GPIO_TypeDef* gpio0, int pin0) : QDN_GPIO_Input(gpio0, pin0) { }
bool QDN_GPIO_InputN::IsAsserted() {
    return (GPIO_ReadInputDataBit(gpio,pinMask) == 0);
}

{ GPIO_Mode_AIN = 0x0,
  GPIO_Mode_IN_FLOATING = 0x04,
  GPIO_Mode_IPD = 0x28,
  GPIO_Mode_IPU = 0x48,
  GPIO_Mode_Out_OD = 0x14,
  GPIO_Mode_Out_PP = 0x10,
  GPIO_Mode_AF_OD = 0x1C,
  GPIO_Mode_AF_PP = 0x18
