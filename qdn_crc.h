#ifndef G8_CRC_H
#define G8_CRC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define update_crc(d,a,t)		(a) = (uint16_t)(((a) << 8) ^ (t)[(a >> 8) ^ (d)])

uint16_t G8_CRC_BufferCalc(uint8_t const *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif
