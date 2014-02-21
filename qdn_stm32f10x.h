#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"

// this STM32 wrapper routines also enable the correct AHB or APB clocks as needed
// It makes the initialization more "black box".

void QDN_GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
void QDN_USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct);


void QDN_VectorInit(void); // assigns the vector table to the section .intvec

///
/// Enables DMA such that n ADC channels are written directly to a uint32[] struct
///
void QDN_ADC_ConfigureDMAAndMuxMany(DMA_Channel_TypeDef *dma, volatile uint16_t* dstArray, ADC_TypeDef * adc,...);


/// RTC helper functions
uint64_t QDN_RTC_GetIntegerTimeStamp(void);
void     QDN_RTC_SetTime(uint64_t value64);

struct QDN_RTC_DateTime_t;
uint8_t QDN_RTC_GetBinaryTimeStamp(struct QDN_RTC_DateTime_t* pTimestamp);
void    QDN_GetShortASCIITimeStamp(char* string, int len);
