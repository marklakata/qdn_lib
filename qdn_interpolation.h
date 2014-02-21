#ifndef _QDN_INTERPOLATION_H_
#define _QDN_INTERPOLATION_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
    
uint16_t QDN_InterpolateADC(uint16_t adc, const int16_t* table, int logTableSize);

#ifdef __cplusplus
}
#endif

#endif
