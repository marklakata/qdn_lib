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
#include <msp430.h>

#ifndef ERRNO_SUCCESS
#define ERRNO_SUCCESS 0
#endif
#ifndef ERRNO_FLASH_VERIFY_FAILED
#define ERRNO_FLASH_VERIFY_FAILED -1
#endif
#ifndef ERRNO_FLASH_INCOMPLETE
#define ERRNO_FLASH_INCOMPLETE -6
#endif
#ifndef ERRNO_FLASH_BUFFER_ODD_SIZE
#define ERRNO_FLASH_BUFFER_ODD_SIZE -7
#endif


#ifdef QDN_ADDRESS_IS_16BIT

#define FLASH_READ_UINT32(a)    __data16_read_addr(a)
#define FLASH_READ_UINT8(a)     (*(uint8_t*)(a))
#define FLASH_WRITE_UINT8(a,b)  do { (*(uint8_t*)(a)) = (b); } while (0)
#define FLASH_WRITE_UINT16(a,b) do { (*(uint16_t*)(a)) = (b); } while (0)

#elif defined(QDN_ADDRESS_IS_32BIT)

#define FLASH_READ_UINT32(a)    __data20_read_long(a)
#define FLASH_READ_UINT8(a)     __data20_read_char(a)
#define FLASH_WRITE_UINT8(a,b)  __data20_write_char(a,b)
#define FLASH_WRITE_UINT16(a,b) __data20_write_short(a,b)

#else
#error
#endif

uint32_t ReadFlashU32(MemoryAddress_t address) {
	return FLASH_READ_UINT32(address);
}

typedef void (*func_ptr)(void);

int16_t FlashReadArray(MemoryAddress_t address, uint8_t* dst, uint32_t size) {
	uint32_t i;
	for(i=0;i<size;i++) {
		dst[i] = FLASH_READ_UINT8(address);
		address++;
	}
	return ERRNO_SUCCESS;
}

int16_t FlashVerifyArray(MemoryAddress_t address, const uint8_t* src, uint32_t size) {
	uint32_t i;
	for(i=0;i<size;i++) {
		uint8_t b = FLASH_READ_UINT8(address);
		if (b != src[i]) return ERRNO_FLASH_VERIFY_FAILED;
		address++;
	}
	return ERRNO_SUCCESS;
}


#if defined(QDN_FLASH_RUN_FROM_RAM) || defined(QDN_FLASH_RUN_FROM_FLASH)

#pragma FUNC_CANNOT_INLINE(FlashWriteArray)
#pragma CODE_SECTION(FlashWriteArray,".ramcode")
int16_t FlashWriteArray(MemoryAddress_t address, const uint8_t* src, uint32_t size) {
	if (address & 1 || size & 3) {
		return ERRNO_FLASH_BUFFER_ODD_SIZE;
	}

#ifdef QDN_FLASH_RUN_FROM_RAM
	int i;
	__istate_t istate;

	uint16_t* src16 = (uint16_t*)src;
	size >>= 1;

	while(FCTL3 & BUSY) ;

    istate = __get_interrupt_state();
	__disable_interrupt();

	FCTL3 = FWKEY;
	FCTL1 = FWKEY+BLKWRT+WRT;

	for(i=0;i<size;i++) {
		FLASH_WRITE_UINT16(address, src16[i]);
		address+= 2;
		while(!(FCTL3 & WAIT)) ;
	}
	FCTL1 = FWPW;                            // Clear WRT bit
	while(FCTL3 & BUSY) ;
	FCTL3 = FWPW+LOCK;                       // Set LOCK bit
    __set_interrupt_state(istate);

#elif defined(QDN_FLASH_RUN_FROM_FLASH)
	int i;
	FCTL3 = FWKEY;
	FCTL1 = FWKEY+WRT;

	for(i=0;i<size;i++) {
		FLASH_WRITE_UINT8(address, src[i]);
		address++;
	}
	FCTL1 = FWKEY;                            // Clear WRT bit
	FCTL3 = FWKEY+LOCK;                       // Set LOCK bit
#else
#error Please define QDN_FLASH_RUN_FROM_RAM or  QDN_FLASH_RUN_FROM_FLASH
#endif
	return 0;
}

#pragma CODE_SECTION(FlashErasePage,".ramcode")
#pragma FUNC_CANNOT_INLINE(FlashErasePage)
int16_t FlashErasePage(MemoryAddress_t page, uint32_t password) {
	if (password != FLASH_PASSWORD) return -1;

	while(FCTL3 & BUSY) ;
	FCTL3 = FWPW;                            // Clear Lock bit
	FCTL1 = FWPW+ERASE;                      // Set Erase bit
	FLASH_WRITE_UINT8(page,0);      		 // Dummy write to erase Flash seg
	while(FCTL3 & BUSY) ;
	FCTL3 = FWPW+LOCK;                       // Clear WRT bit

#if 0
	if (page == 0x0000FE00) {
		static const uint8_t resetVector[2] = { 0x00, 0x5C};
		// vector table was just erased... write the reset vector back as soon as possible!
		FlashWriteArray(0x0000FFFE,resetVector,2);
	}
#endif
	return 0;
}

#endif


#ifdef QDN_FLASH_RUN_FROM_RAM
void FlashInstallRAM_Section(void)
{
	extern void _flash_code_rn_start(void);                                 // where the .ramcode section is mapped to RAM.
	extern void _flash_code_ld_start(void);                                 // where the .ramcode section sits in flash.
	extern void _flash_code_size(void);                                     // this symbol "address" is actually the length of .ramcode section
	void* dst = &_flash_code_rn_start;
	void* src = &_flash_code_ld_start;
	uint32_t len = (uint32_t)&_flash_code_size;
	memcpy(dst,src,len);
}
#endif


int16_t FlashErasePages(MemoryAddress_t page, MemoryAddress_t end, uint32_t password) {
	for(;page < end; page += FlashPageSize()) {
		FlashErasePage(page,password);
	}
	return 0;
}

uint32_t FlashPageSize(void)
{
	return 0x200;
}


////////////

#if 0

//******************************************************************************
//  MSP430x54x Demo - Block Write and Memory Erase @ 0x10000 Executed from RAM
//
//  Description: This program first copies write_block_int function to RAM.
//  Copying to RAM requires a custom made .cmd command linker file. This file is
//  attached as part of the project called custom_lnk_msp430f5438.cmd. This
//  program then erases Flash address 0x10000 - 0x1FFFF which is executed from
//  RAM. Then, it writes to Bank 1 from 0x10000 to 0x1007F. This write shows the
//  advantage of block writing in which 2 consecutive word can be written.
//
//
//  Important Notes:
//
//  1. CCE automatically generates a new copy of linker file in the project
//     directory. The zip file has the attached required modified linker file.
//     See below on what was added to the linker file.
//
//       MEMORY
//       {
//         ...
//         RAM_MEM         : origin = 0x1C00, length = 0x0200
//         FLASH_MEM       : origin = 0x5C00, length = 0x0200
//         ...
//       }
//
//       SECTIONS
//       {
//         ...
//         .FLASHCODE : load = FLASH_MEM, run = RAM_MEM
//                                          /* CODE IN FLASH AND WILL BE COPIED
//                                             TO RAM AT EXECUTION HANDLED BY
//                                             USER                            */
//         .RAMCODE   : load = FLASH_MEM    /* CODE WILL BE IN RAM             */
//         ...
//       }
//
//  2. Define the allocated memory area that will be copied from FLASH to RAM.
//     In this case, user has to manually define the start address of FLASH and
//    RAM. These memory addresses has to be the same as defined in the linker
//     file origin address of FLASH_MEM and RAM_MEM. The FLASH_MEM_LENGTH can be
//     changed to however much the final compiled code size is.
//
//       FLASH_MEM_BEGIN   .equ   0x5C00     ; Flash code starting address
//       FLASH_MEM_LENGTH  .equ   0x0200     ; Function segment size to be copied
//       RAM_MEM_BEGIN     .equ   0x1C00     ; RAM code starting address
//
//  RESET the device to re-execute code. This is implemented to prevent
//  stressing of Flash unintentionally.
//  ACLK = REFO = 32kHz, MCLK = SMCLK = default DCO 1048576Hz
//
//                MSP430x54x
//            -----------------
//        /|\|              XIN|-
//         | |                 |
//         --|RST          XOUT|-
//           |                 |
//
//   W. Goh
//   Texas Instruments Inc.
//   July 2011
//   Built with Code Composer Studio Version: 4.2.3.00004
//******************************************************************************

#include "msp430x54x.h"
#include "string.h"

#define FLASH_MEM_BEGIN   0x5C00            // Flash code starting address
#define FLASH_MEM_LENGTH  0x0200            // Function segment size to be copied
#define RAM_MEM_BEGIN     0x1C00            // RAM code starting address

// Function prototypes
void copy_flash_to_RAM(void);
void write_block_int(void);

unsigned long value;

void main(void)
{

  WDTCTL = WDTPW+WDTHOLD;                   // Stop WDT

  copy_flash_to_RAM();                      // Copy flash to RAM function

  value = 0x12340000;                       // initialize Value

  write_block_int();                        // This portion of code is executed
                                            // in RAM
  while(1);                                 // Loop forever, SET BREAKPOINT HERE
}

//------------------------------------------------------------------------------
// Copy flash function to RAM.
//------------------------------------------------------------------------------
void copy_flash_to_RAM(void)
{
  unsigned char *flash_start_ptr;           // Initialize pointers
  unsigned char *RAM_start_ptr;

  //Initialize flash and ram start and end address
  flash_start_ptr = (unsigned char *)FLASH_MEM_BEGIN;
  RAM_start_ptr = (unsigned char *)RAM_MEM_BEGIN;

  // Copy flash function to RAM
  memcpy(RAM_start_ptr,flash_start_ptr,FLASH_MEM_LENGTH);
}

#pragma CODE_SECTION(write_block_int,".FLASHCODE")
//------------------------------------------------------------------------------
// This portion of the code is first stored in Flash and copied to RAM then
// finally executes from RAM.
//-------------------------------------------------------------------------------
void write_block_int(void)
{
  unsigned int i;
  unsigned long * Flash_ptr;
  Flash_ptr = (unsigned long *)0x10000;     // Initialize write address
  __disable_interrupt();                    // 5xx Workaround: Disable global
                                            // interrupt while erasing. Re-Enable
                                            // GIE if needed
  // Erase Flash
  while(BUSY & FCTL3);                      // Check if Flash being used
  FCTL3 = FWKEY;                            // Clear Lock bit
  FCTL1 = FWKEY+ERASE;                      // Set Erase bit
  *Flash_ptr = 0;                           // Dummy write to erase Flash seg
  while(BUSY & FCTL3);                      // Check if Erase is done

  // Write Flash
  FCTL1 = FWKEY+BLKWRT+WRT;                 // Enable block write

  for(i = 0; i < 32; i++)
  {
    *Flash_ptr++ = value++;                 // Write long int to Flash

    while(!(WAIT & FCTL3));                 // Test wait until ready for next byte
  }

  FCTL1 = FWKEY;                            // Clear WRT, BLKWRT
  while(BUSY & FCTL3);                      // Check for write completion
  FCTL3 = FWKEY+LOCK;                       // Set LOCK
}
#endif

