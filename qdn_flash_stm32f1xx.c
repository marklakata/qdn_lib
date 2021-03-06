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

#include "qdn_flash.h"

#include "stm32f10x_flash.h"
#include <string.h>
#include "errno.h"

#ifndef ERRNO_SUCCESS
#define ERRNO_SUCCESS 0
#endif
#ifndef ERRNO_FLASH_VERIFY_FAILED
#define ERRNO_FLASH_VERIFY_FAILED -1
#endif
#ifndef ERRNO_STM32_FLASH_BUSY
#define ERRNO_STM32_FLASH_BUSY - 2
#endif
#ifndef ERRNO_STM32_FLASH_ERROR_PG
#define ERRNO_STM32_FLASH_ERROR_PG -3
#endif
#ifndef ERRNO_STM32_FLASH_ERROR_WRP
#define ERRNO_STM32_FLASH_ERROR_WRP -4
#endif
#ifndef ERRNO_STM32_FLASH_TIMEOUT
#define ERRNO_STM32_FLASH_TIMEOUT -5
#endif
#ifndef ERRNO_FLASH_INCOMPLETE
#define ERRNO_FLASH_INCOMPLETE -6
#endif
#ifndef ERRNO_FLASH_BUFFER_ODD_SIZE
#define ERRNO_FLASH_BUFFER_ODD_SIZE -7
#endif

uint32_t ReadFlashU32(uint32_t address) {
    return *(uint32_t*)address;
}

int16_t FlashReadArray(MemoryAddress_t address, uint8_t* dst, uint32_t size) {
    memcpy(dst,(uint8_t*)address,size);
    return ERRNO_SUCCESS;
}

int16_t FlashVerifyArray(MemoryAddress_t address, const uint8_t* src, uint32_t size) {
    return memcmp(src,(uint8_t*)address,size)?ERRNO_FLASH_VERIFY_FAILED : ERRNO_SUCCESS;
}

static int16_t FlashStatus(FLASH_Status s) {
    switch (s) {
    case FLASH_COMPLETE:
        return ERRNO_SUCCESS;
    case FLASH_BUSY:
        return ERRNO_STM32_FLASH_BUSY;
    case FLASH_ERROR_PG: 
        return ERRNO_STM32_FLASH_ERROR_PG;
    case FLASH_ERROR_WRP :
        return ERRNO_STM32_FLASH_ERROR_WRP;
    case FLASH_TIMEOUT:
        return ERRNO_STM32_FLASH_TIMEOUT;
    default:
        return ERRNO_FLASH_INCOMPLETE;
    }
}

int16_t FlashWriteArray(MemoryAddress_t address, const uint8_t* src, uint32_t size) {
    if (address & 1 || size & 1) {
        return ERRNO_FLASH_BUFFER_ODD_SIZE;
    }


    // otherwise try to program it.
    // note that programming memory that is not 0xFF will fail, unless a zero is
    // programmed.
    FLASH_Status flashStatus = FLASH_COMPLETE;
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    for(int i=0;i<size && flashStatus == FLASH_COMPLETE;i+=sizeof(uint16_t)) {
        flashStatus = FLASH_ProgramHalfWord(address ,  *(uint16_t*)(src + i));
        address += sizeof(uint16_t);
    }
    FLASH_Lock();

    return FlashStatus(flashStatus);
}

int16_t FlashErasePage(MemoryAddress_t page, uint32_t password) {
    FLASH_Status flashStatus = FLASH_ERROR_PG;
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    if ( password == FLASH_PASSWORD) {
        flashStatus = FLASH_ErasePage(page);
    }
    FLASH_Lock();
    
    return FlashStatus(flashStatus);
}
