#define APP_BOOT
#ifndef _APP_BOOT_H

// to use this file, you must use the ICF file

#include <stdint.h>


typedef struct {
    uint32_t crc;
    uint32_t boot0;
    uint32_t boot1;
    uint32_t reserved[61]; // should be 0xFFFFFFFF by default
} AppConfig_t;

extern AppConfig_t appConfig;

#ifdef __cplusplus
extern "C" {
#endif

    int  AppBootCompareCRC(void);
uint32_t AppBootGoldenCRC(void);
uint32_t AppBootCalculatedCRC(void);

uint32_t AppBootSpareGoldenCRC(void);
uint32_t AppBootSpareCalculatedCRC(void);

uint32_t AppCurrentBank(void);
uint32_t AppBootConfigWord(int index);

void     AppBootSetBank(int bank);
void     AppBootClearBank(void);
int      AppBootGetSoftBootBank(int* pBank);
                                
uint32_t SpareMemStart(void);
uint32_t SpareMemEnd(void);

uint32_t AppMemStart(void);

extern int config_block_size;


extern int config_start;
extern int code_bank_start;

extern int app_bank;

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

#define APP_BANK          ((uint32_t) &app_bank)

#endif




#ifdef __cplusplus
}
#endif

    
#endif

