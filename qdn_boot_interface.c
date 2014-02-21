#include "g8_boot_interface.h"

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

extern uint32_t spare_bank_start;
extern uint32_t spare_bank_end;

#ifdef BOOTLOADER
#define xbank0_start (*(uint32_t*)0x08000FF0)
#define xbank1_start (*(uint32_t*)0x08000FF4)
#define xbank_size   (*(uint32_t*)0x08000FF8)
#define xbank_cookie (*(uint32_t*)0x08000FFC)
#else
#define BANKMAP_VARIABLE _Pragma("location=\".bankmap\"") extern __root
BANKMAP_VARIABLE uint32_t xbank0_start;
BANKMAP_VARIABLE uint32_t xbank1_start;
BANKMAP_VARIABLE uint32_t xbank_size  ;
BANKMAP_VARIABLE uint32_t xbank_cookie;
#endif


uint32_t BANK_0_START(void) {
    if (xbank_cookie == 0xFEEDCAFE) {
        return xbank0_start;
    } else {
        return 0x8001000;
    }
}
uint32_t BANK_1_START(void) {
    if (xbank_cookie == 0xFEEDCAFE) {
        return xbank1_start;
    } else {
        return 0x800C000;
    }
}
uint32_t BANK_SIZE(void) {
    if (xbank_cookie == 0xFEEDCAFE) {
        return xbank_size;
    } else {
        return 0xB000;
    }
}


uint32_t AppCurrentBank(void) {
    return APP_BANK;
}

uint32_t AppBootConfigWord(int index) {
    return *(uint32_t*)(CONFIG_START + 4 + 4*index);
}

uint32_t SpareMemStart(void) {
    return (uint32_t)&spare_bank_start;
}

uint32_t SpareMemEnd(void) {
    return (uint32_t)&spare_bank_end;
}

uint32_t AppMemStart(void) {
    return CONFIG_START;
}


int AppBootCompareCRC(void) {
    return (AppBootGoldenCRC() == AppBootCalculatedCRC());
}

#pragma location = "APP_BOOT_RAM_SECTION"
uint32_t bootFlag;
void AppBootSetBank(int bank) {
    if (bank == 0 || bank == 1) {
        bootFlag = 0xCAFEED00 + bank;
    } else {
        bootFlag = 0;
    }
}


int      AppBootGetSoftBootBank(int* pBank) {
    int ret = 0;
    if ((bootFlag & 0xFFFFFF00) == 0xCAFEED00) {
        int bank = bootFlag & 0xFF;
        if (bank == 0 || bank == 1) {
            *pBank = bank;
            ret = 1;
        }
    }
    bootFlag = 0;
    return ret;
}

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

uint32_t AppBootSpareCalculatedCRC(void) {
    uint32_t crc;
#ifdef USE_CRC16
    crc = crc_buffer_calc((uint8_t*)(SpareMemStart() + CONFIG_BLOCK_SIZE), CODE_SIZE());
#elif defined(USE_CR32)
#error
#endif    
    return crc;
}
