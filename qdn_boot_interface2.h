#define APP_BOOT
#ifndef _APP_BOOT_H

#include "project.h"

#define USE_CRC16
// #define USE_CRC32

#include <stdint.h>

// this block is 256 bytes long
typedef struct {
    uint32_t crc;
    uint32_t boot0;
    uint32_t boot1;
    uint32_t reserved[61]; // should be 0xFFFFFFFF by default
} AppConfig_t;

extern volatile AppConfig_t appConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
const AppConfig_t* G8_GetAppConfig(int bank);
int                G8_GetCurrentBank(void);
int                G8_GetSpareBank(void);

int                G8_GetPreferredBank(void) ;
void               G8_SetOneTimeBootBank(int bank);
uint16_t           G8_CalculateBankCrcU16(int bank);
void               G8_JumpToBank(int bank);

uint32_t           G8_SpareMemStart(void);
uint32_t           G8_SpareMemEnd(void);


#if 0
    int  AppBootCompareCRC(void);
uint32_t AppBootGoldenCRC(void);
uint32_t AppBootCalculatedCRC(void);

uint32_t AppBootSpareGoldenCRC(void);
uint32_t AppBootSpareCalculatedCRC(void);

uint32_t AppBootConfigWord(int index);

void     AppBootSetBank(int bank);
void     AppBootClearBank(void);
int      AppBootGetSoftBootBank(int* pBank);
                                
uint32_t AppMemStart(void);

extern int config_block_size;


extern int config_start;
extern int code_bank_start;


#ifdef __ICCARM__

#pragma location = "APP_BOOT_RAM_SECTION"
extern uint32_t bootFlag;

#pragma location=".checksum"
extern uint32_t const __checksum;

#define CONFIG_BLOCK_SIZE ((uint32_t) &config_block_size)
#define CODE_SIZE()       (BANK_SIZE() - CONFIG_BLOCK_SIZE)

uint32_t BANK_0_START(void);
uint32_t BANK_1_START(void);
uint32_t BANK_SIZE(void);

#define CODE_BANK_START   ((uint32_t) &code_bank_start)
#define CONFIG_START      ((uint32_t) &config_start)


#endif

#endif


#ifdef __cplusplus
}
#endif

    
#endif

