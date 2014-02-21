#include "g8_boot_interface2.h"
#include "g8_crc.h"

#pragma location = ".bootflag"
__no_init uint32_t bootFlag;

extern void* map_MAIN_APP_0_start;
extern void* map_MAIN_APP_1_start;
extern void* map_MAIN_APP_size;
extern void* map_CONFIG_0_start;
extern void* map_CONFIG_1_start;

extern void* map_SPARE_start;
extern void* map_SPARE_size;
extern void* map_SPARE_end;

extern void* map_current_bank;
extern void* map_SPARE_bank;
     
#ifdef __ICCARM__
//#pragma location=".checksum"
//__root uint32_t __checksum;
//__root AppConfig_t appConfig = {0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF};
#endif

int G8_GetPreferredBank(void) {
    if ((bootFlag & 0xFFFFFF00) == 0xCAFEED00) {
        int bank = bootFlag & 0xFF;
        bootFlag = 0;
        if (bank == 0 || bank == 1) {
            return bank;
        }
    }
    
    if        (G8_GetAppConfig(0)->boot0 == 0 && G8_GetAppConfig(0)->boot1 == 0xFFFFFFFF) { // good install
        return 0;
    } else if (G8_GetAppConfig(1)->boot0 == 0 && G8_GetAppConfig(1)->boot1 == 0xFFFFFFFF) { // good install
        return 1;
    } else if (G8_GetAppConfig(0)->boot0 == 0 && G8_GetAppConfig(0)->boot1 == 0) { // disabled install, but better than nothing
        return 0;
    } else if (G8_GetAppConfig(1)->boot0 == 0 && G8_GetAppConfig(1)->boot1 == 0) { // disabled install, but better than nothing
        return 1;
    } else {
        return 0;
    }
}


void G8_SetOneTimeBootBank(int bank) {
    if (bank == 0 || bank == 1) {
        bootFlag = 0xCAFEED00 + bank;
    } else {
        bootFlag = 0;
    }
}

static void* G8_GetBankStart(int bank) {
    switch(bank) {
    case 0:
        return (void*)&map_MAIN_APP_0_start;
    case 1:
        return (void*)&map_MAIN_APP_1_start;
    default:
        return 0;
    }
}

static uint32_t G8_GetBankSize(int bank) {
    return (uint32_t)&map_MAIN_APP_size;  
}

const AppConfig_t* G8_GetAppConfig(int bank) {
    switch(bank) {
    case 0:
        return (void*)&map_CONFIG_0_start;
    case 1:
        return (void*)&map_CONFIG_1_start;
    default:
        return 0;
    }
}


uint16_t G8_CalculateBankCrcU16(int bank) {
    uint16_t crc = G8_CRC_BufferCalc(G8_GetBankStart(bank), G8_GetBankSize(bank));
    return crc;
}

typedef void (*FunctionPointer_t)(void);

#include "intrinsics.h"

void G8_JumpToBank(int bank) {
    // if CRC is ok, then we have the boot entry vector
    uint32_t* vectorTable = (uint32_t*)G8_GetBankStart(bank);
    uint32_t topOfStack           = vectorTable[0];
    FunctionPointer_t resetVector = (FunctionPointer_t) vectorTable[1];
    if (resetVector != 0) {
        __disable_interrupt();
        __set_MSP(topOfStack);
        resetVector();
    }

}

int G8_GetCurrentBank(void) {
    return (int)&map_current_bank;
}

int G8_GetSpareBank(void) {
    return (int)&map_SPARE_bank;
}


uint32_t G8_SpareMemStart(void) {
    return (uint32_t)&map_SPARE_start;
}

uint32_t G8_SpareMemEnd(void) {
    return (uint32_t)&map_SPARE_end;
}


#if 0
#define USE_CRC16
// #define USE_CRC32


#ifdef USE_CRC16
#include "g8_crc.h"
#elif defined(USE_CRC32)
#if 1
#include "stm32f10x_crc.h"
#else
#include "stm32f10x_rcc.h"
#endif
#endif



uint32_t AppBootGoldenCRC(void) {
#ifdef USE_CRC16
    return (uint32_t)(__checksum & 0xFFFF);
#else
    return __checksum;
#endif
}

uint32_t AppBootSpareGoldenCRC(void) {
#ifdef USE_CRC16
    return *(uint16_t*)SpareMemStart();
#else
    return *(uint32_t*)SpareMemStart();
#endif
}

#ifdef USE_CRC32
#if 0
u32 revbit(u32 data);
u32 CalcCRC32(u32 *dworddata,u32 dwordcount);

/**********************
reverse bits of DWORD
Input:
u32 data -- the input
Output:
u32 data -- the output
**********************/
u32 revbit(u32 data)
{
    asm("rbit r0,r0");
    return data;
};

//CopyRight:www.mcuisp.com
//Compile With IAR
/**********************
Calculate CRC32 of DWORD data array.
Input:
u32 *dworddata -- the array point
u32 dwordcount -- the data len in DWORD
Output:
u32 CRC32 -- the result
**********************/
u32 CalcCRC32(u32 *dworddata,u32 dwordcount)
{
    u32 ui32;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC,ENABLE);
    CRC->CR=1;
    asm("NOP");asm("NOP");asm("NOP");//delay for hardware ready
    for(;dwordcount>0;dwordcount--)
    {
        ui32=*dworddata;
        dworddata++;
        ui32=revbit(ui32);//reverse the bit order of input data
        CRC->DR=ui32;
    }
    ui32=CRC->DR;
    ui32=revbit(ui32);//reverse the bit order of output data
    
    ui32^=0xffffffff;//xor with 0xffffffff
    return ui32;//now the output is compatible with windows/winzip/winrar
}; 
#endif
#endif

uint32_t AppBootCalculatedCRC(void) {
    uint32_t crc;
#ifdef USE_CRC16
    crc = crc_buffer_calc((uint8_t*)CODE_BANK_START, CODE_SIZE());
#elif defined(USE_CR32)
#if 1
    crc = CRC_CalcBlockCRC(( u32*)0x08000020, 1);
    crc = CRC_CalcBlockCRC((const u32*)CODE_BANK_START, CODE_SIZE/sizeof(uint32_t));
#else
    crc = CalcCRC32(( u32*)0x08000020, 1);
    crc = CalcCRC32(( u32*)CODE_BANK_START, CODE_SIZE/sizeof(uint32_t));
#endif
#endif    
    return crc;
}


#endif
