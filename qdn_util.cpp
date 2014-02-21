#include "qdn_util.h"

void UnpackUInt16LE(const void* ptr,uint16_t& value) {
    const uint8_t* pu8 = static_cast<const uint8_t*>( ptr );
    value = 
            ((uint16_t)(pu8[0]) <<  0) |
            ((uint16_t)(pu8[1]) <<  8);
}

void UnpackUInt32LE(const void* ptr,uint32_t& value) {
    const uint8_t* pu8 = static_cast<const uint8_t*>( ptr );
    value = 
            ((uint32_t)(pu8[0]) <<  0) |
            ((uint32_t)(pu8[1]) <<  8) |
            ((uint32_t)(pu8[2]) << 16) |
            ((uint32_t)(pu8[3]) << 24);
}

void PackUInt32LE( uint32_t value, void* ptr) {
    uint8_t* pu8 = static_cast<uint8_t*>( ptr );
    pu8[0] = (uint8_t)(value & 0xFF);
    value>>=8;
    pu8[1] = (uint8_t)(value & 0xFF);
    value>>=8;
    pu8[2] = (uint8_t)(value & 0xFF);
    value>>=8;
    pu8[3] = (uint8_t)(value & 0xFF);
}



