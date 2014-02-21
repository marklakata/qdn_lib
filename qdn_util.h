#ifndef _QDN_UTIL_H_
#define _QDN_UTIL_H_

#include <stdint.h>

void UnpackUInt16LE(const void* ptr,uint16_t& value) ;
void UnpackUInt32LE(const void* ptr,uint32_t& value);
void PackUInt32LE( uint32_t value, void* ptr) ;

#endif

