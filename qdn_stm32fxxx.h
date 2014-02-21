#ifndef _G8_STM32FXXX_H_
#define _G8_STM32FXXX_H_

#include <misc.h>

#ifdef __cplusplus
extern "C" {
#endif

void G8_NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
void G8_SystemReset(void);

#ifdef __cplusplus
}
#endif

#endif //  _G8_STM32FXXX_H_
