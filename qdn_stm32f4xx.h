#ifndef _G8_STM32FXX_H_
#define _G8_STM32FXX_H_

#include <stdint.h>

#include "g8_stm32fxxx.h"
#include <stm32f4xx_usart.h>

struct  G8_RTC_DateTime_s;

typedef struct {
    IRQn_Type      irqn;
    uint8_t        altFunc;
} USART_Helper_t;

_EXTERN_C

void    G8_ADC_ConfigureDMAAndMuxMany(DMA_Stream_TypeDef *dma, volatile uint16_t* dst, ADC_TypeDef * adc,...);

void    G8_RTC_Config(void);
uint8_t G8_RTC_GetBinaryTimeStamp(struct G8_RTC_DateTime_s* pTimestamp);
void    G8_GetShortASCIITimeStamp(char* string, int len);
int     G8_RTC_SetBinaryTimeStamp(struct G8_RTC_DateTime_s* pTimestamp);

USART_Helper_t G8_USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct);
void           G8_GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);

void           G8_VectorInit(void);

_END_EXTERN_C


#endif // _G8_STM32FXX_H_

