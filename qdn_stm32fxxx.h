#ifndef _QDN_STM32FXXX_H_
#define _QDN_STM32FXXX_H_

#include <misc.h>

#ifdef __cplusplus
extern "C" {
#endif

void QDN_NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
void QDN_SystemReset(void);

#ifdef __cplusplus
}
#endif

#endif //  _QDN_STM32FXXX_H_
