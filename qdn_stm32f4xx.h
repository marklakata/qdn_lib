#ifndef _QDN_STM32FXX_H_
#define _QDN_STM32FXX_H_

#include <stdint.h>

#include "qdn_stm32fxxx.h"
#include <stm32f4xx_usart.h>

struct  QDN_RTC_DateTime_s;

typedef struct {
    IRQn_Type      irqn;
    uint8_t        altFunc;
} USART_Helper_t;

_EXTERN_C

void    QDN_ADC_ConfigureDMAAndMuxMany(DMA_Stream_TypeDef *dma, volatile uint16_t* dst, ADC_TypeDef * adc,...);

void    QDN_RTC_Config(void);
uint8_t QDN_RTC_GetBinaryTimeStamp(struct QDN_RTC_DateTime_s* pTimestamp);
void    QDN_GetShortASCIITimeStamp(char* string, int len);
int     QDN_RTC_SetBinaryTimeStamp(struct QDN_RTC_DateTime_s* pTimestamp);

USART_Helper_t QDN_USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct);
void           QDN_GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);

void           QDN_VectorInit(void);

_END_EXTERN_C


#endif // _G8_STM32FXX_H_

