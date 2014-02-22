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

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*Callback_t)(void);

/// Call this function first on application initialization.
void    QDN_ParamInit(void);

void    QDN_ParamSetDefault(uint16_t index, int32_t value, Callback_t cb);
void    QDN_ParamSetRaw(uint16_t index, int32_t value);
void    QDN_ParamSetFloatRaw(uint16_t index, float value);
void    QDN_ParamSetString(uint16_t index, uint16_t lastIndex, char* buffer, int32_t length);
void    QDN_ParamSetWithCallback(uint16_t index, int32_t value);
int32_t QDN_ParamInt32(uint16_t index);
float   QDN_ParamFloat(uint16_t index);
int     QDN_ParamString(uint16_t index, char* buffer, int32_t maxChars);
uint8_t QDN_ParamStatus(uint16_t index);
void    QDN_ParamEraseAll(void);
void    QDN_ParamMemoryCopy(uint8_t* buffer, uint16_t start, uint16_t end);
void    QDN_ParamErase(uint16_t index);
 
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
const int32_t& QDN_ParamRef(uint16_t index);
#endif
