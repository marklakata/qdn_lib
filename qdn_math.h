#ifndef _G8_MATH_H_
#define _G8_MATH_H_

#include <stdint.h>

inline int16_t iabs(int16_t a) {
    if (a < 0) {
        return -a;
    } else {
        return a;
    }
}

#define ROUND_INT16(x) ((int16_t)((x>=0)?(x+0.5):(x-0.5)))

#endif
