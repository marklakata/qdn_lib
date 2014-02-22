#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"

#include "qdn_cpu.h"

QDN_EXTERN_C
// this STM32 wrapper routines also enable the correct AHB or APB clocks as needed
// It makes the initialization more "black box".

void QDN_GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
void QDN_VectorInit(void); // assigns the vector table to the section .intvec

///
/// Enables DMA such that n ADC channels are written directly to a uint32[] struct
///
void QDN_ADC_ConfigureDMAAndMuxMany(DMA_Channel_TypeDef *dma, volatile uint16_t* dstArray, ADC_TypeDef * adc,...);


/// RTC helper functions
uint64_t QDN_RTC_GetIntegerTimeStamp(void);
void     QDN_RTC_SetTime(uint64_t value64);

struct QDN_RTC_DateTime_s;
uint8_t QDN_RTC_GetBinaryTimeStamp(struct QDN_RTC_DateTime_s* pTimestamp);
void    QDN_GetShortASCIITimeStamp(char* string, int len);

QDN_END_EXTERN_C
