#ifndef _G8_INTERPOLATION_H_
#define _G8_INTERPOLATION_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
    
uint16_t G8_InterpolateADC(uint16_t adc, const int16_t* table, int logTableSize);

#ifdef __cplusplus
}
#endif

#endif
