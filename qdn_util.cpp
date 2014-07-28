/**************************************************************************
 *
 * Copyright (c) 2013, Qromodyn Corporation
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the FreeBSD Project.
 **************************************************************************/

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

void QDN_Exception(const char* message)
{
	__builtin_trap();
}

