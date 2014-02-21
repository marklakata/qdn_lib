#include "qdn_flash.h"

#include "stm32f4xx_flash.h"
#include <string.h>
#include "errno.h"
#include "qdn_xos.h"

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
    return memcmp(src,(uint8_t*)address,size) ? ERRNO_FLASH_VERIFY_FAILED : ERRNO_SUCCESS;
}

static int16_t FlashStatus(FLASH_Status s) {
    switch (s) {
    case FLASH_COMPLETE:
        return ERRNO_SUCCESS;
    case FLASH_BUSY:
        return ERRNO_STM32_FLASH_BUSY;
    case FLASH_ERROR_PGS: 
    case FLASH_ERROR_PGP: 
    case FLASH_ERROR_PGA: 
    case FLASH_ERROR_PROGRAM:
        return ERRNO_STM32_FLASH_ERROR_PG;
    case FLASH_ERROR_WRP :
        return ERRNO_STM32_FLASH_ERROR_WRP;
    case FLASH_ERROR_OPERATION:
        return ERRNO_STM32_FLASH_TIMEOUT;
    default:
        return ERRNO_FLASH_INCOMPLETE;
    }
}

#define FLASH_Sector_End (FLASH_Sector_11 + 8) // a dummy sector
#define FLASH_Sector_Delta 8

static int GetSector(uint32_t addr)
{
  if (addr < 0x08000000) {
      return -1;
  } else if (addr < 0x08004000) {
      return FLASH_Sector_0;
  } else if (addr < 0x08008000) {
      return FLASH_Sector_1;
  } else if (addr < 0x0800C000) {
      return FLASH_Sector_2;
  } else if (addr < 0x08010000) {
      return FLASH_Sector_3;
  } else if (addr < 0x08020000) {
      return FLASH_Sector_4;
  } else if (addr < 0x08040000) {
      return FLASH_Sector_5;
  } else if (addr < 0x08060000) {
      return FLASH_Sector_6;
  } else if (addr < 0x08080000) {
      return FLASH_Sector_7;
  } else if (addr < 0x080A0000) {
      return FLASH_Sector_8;
  } else if (addr < 0x080C0000) {
      return FLASH_Sector_9;
  } else if (addr < 0x080E0000) {
      return FLASH_Sector_10;
  } else if (addr < 0x08100000) {
      return FLASH_Sector_11;
  } else if (addr == 0x08100000) {
      return FLASH_Sector_End;
  } else {
      return -1;
  }
}

      
int16_t FlashWriteArray(MemoryAddress_t address, const uint8_t* src, uint32_t size) {
    if (address & 3 || size & 3) {
        return ERRNO_FLASH_BUFFER_ODD_SIZE;
    }


    // otherwise try to program it.
    // note that programming memory that is not 0xFF will fail, unless a zero is
    // programmed.
    FLASH_Status flashStatus = FLASH_COMPLETE;
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP    | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR| FLASH_FLAG_PGSERR); 

    for(int i=0;i<size && flashStatus == FLASH_COMPLETE;i+=sizeof(uint32_t)) {
        XOS_EnterCritical();
        flashStatus = FLASH_ProgramWord(address, *(uint32_t*)(src + i));
        XOS_LeaveCritical();
        address += sizeof(uint32_t);
    }
    FLASH_Lock();

    return FlashStatus(flashStatus);
}

int16_t FlashErasePage(MemoryAddress_t page, uint32_t password) {
    FLASH_Status flashStatus = FLASH_ERROR_OPERATION;
    int sector = GetSector(page);
    if (sector != -1) {
        FLASH_DataCacheCmd(DISABLE);
        FLASH_Unlock();
        FLASH_ClearFlag(FLASH_FLAG_EOP    | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                        FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR| FLASH_FLAG_PGSERR); 
            if ( password == FLASH_PASSWORD) {
                flashStatus = FLASH_EraseSector(sector, VoltageRange_3);
            }
        
        FLASH_Lock();
        FLASH_DataCacheReset();
        FLASH_DataCacheCmd(ENABLE);
    }
    return FlashStatus(flashStatus);
}

// endPage is 1 byte past the end of the region to be erased
int16_t FlashErasePages(MemoryAddress_t startPage, MemoryAddress_t endPage, uint32_t password) {
    FLASH_Status flashStatus = FLASH_COMPLETE;
    int startSector = GetSector(startPage);
    int endSector   = GetSector(endPage-1); // move back 1 byte to be inside the page
    if (startSector != -1 && endSector != -1) {
        for(int sector=startSector;sector <= endSector && flashStatus == FLASH_COMPLETE;sector+= FLASH_Sector_Delta) {
        
            FLASH_DataCacheCmd(DISABLE);
            FLASH_Unlock();
            FLASH_ClearFlag(FLASH_FLAG_EOP    | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                            FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR| FLASH_FLAG_PGSERR); 
                if ( password == FLASH_PASSWORD) {
                    flashStatus = FLASH_EraseSector(sector, VoltageRange_3);
                }
            
            FLASH_Lock();
            FLASH_DataCacheReset();
            FLASH_DataCacheCmd(ENABLE);
        }
    }
    return FlashStatus(flashStatus);
}
