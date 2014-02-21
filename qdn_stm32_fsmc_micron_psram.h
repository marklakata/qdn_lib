#ifndef _G8_STM32_FSMC_MICRON_PSRAM_H_
#define _G8_STM32_FSMC_MICRON_PSRAM_H_

// support for Micron MT45W8MW16BGX-701IT PSRAM to STM32 FSMC

#include <stdint.h>


#define Bank1_SRAM2_ADDR  ((uint32_t)0x64000000)  
 

typedef void (*ProgressCallback_t)(void);
_EXTERN_C
void G8_PSRAM_Init(void);
void G8_PSRAM_WriteBuffer(uint16_t* pBuffer, uint32_t WriteAddr, uint32_t NumHalfwordToWrite);
void G8_PSRAM_ReadBuffer(uint16_t* pBuffer, uint32_t ReadAddr, uint32_t NumHalfwordToRead);
int  G8_PSRAM_MemoryTest1(ProgressCallback_t func);
int  G8_PSRAM_MemoryTest2(ProgressCallback_t func);
int  G8_PSRAM_MemoryTest3(ProgressCallback_t func);

_END_EXTERN_C

#endif /*_G8_STM32_FSMC_MICRON_PSRAM_H_H */
