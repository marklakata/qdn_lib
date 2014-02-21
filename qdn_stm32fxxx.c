// placeholder for general stm32 functions that work on all 100, 200 and 400 platforms.
#include "qdn_xos.h"
#include "qdn_stm32fxxx.h"


void G8_NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct) {
#if defined(FREERTOS)
    assert(NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    assert(NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority <= configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
#endif
    
    NVIC_Init(NVIC_InitStruct);

}
