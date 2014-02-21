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

#ifndef _QDN_FLASH_H_
#define _QDN_FLASH_H_

#include "qdn_cpu.h"

#include <stdint.h>

QDN_EXTERN_C
uint32_t ReadFlashU32(MemoryAddress_t address);
int16_t  FlashReadArray(MemoryAddress_t address, uint8_t* dst, uint32_t size);

int16_t  FlashWriteArray(MemoryAddress_t address, const uint8_t* src, uint32_t size);
int16_t  FlashErasePage(MemoryAddress_t page, uint32_t password);
int16_t  FlashErasePages(MemoryAddress_t startPage, MemoryAddress_t endPage, uint32_t password);

int16_t  FlashVerifyArray(MemoryAddress_t address, const uint8_t* src, uint32_t size);
uint32_t FlashPageSize(void);

#ifdef QDN_FLASH_RUN_FROM_RAM
void     FlashInstallRAM_Section(void);
#endif

QDN_END_EXTERN_C

#define FLASH_PASSWORD 0xDEADBEEF

#endif
