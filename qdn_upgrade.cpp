
#include <stdint.h>
#include "g8_upgrade.h"
#include "g8_boot_interface2.h"
#include "g8_flash.h"
#include "g8_stm32fxxx.h"
#include "g8_util.h"
#include <stdlib.h>

#include "firmware_decode.h"  // the application must declare "FirmwareDecode() in firmware_decode.h

#include <string.h>


UpgradeRegisters_t upgradeRegisters;

void G8_UpgradeInit(void) {
    upgradeRegisters.CurrentAppBank = G8_GetCurrentBank();

    const AppConfig_t* appConfig    = G8_GetAppConfig(G8_GetCurrentBank());
    upgradeRegisters.AppBoot0       = appConfig->boot0;
    upgradeRegisters.AppBoot1       = appConfig->boot1;
    upgradeRegisters.AppGoldenCRC   = appConfig->crc;
    upgradeRegisters.AppCalcedCRC   = G8_CalculateBankCrcU16(G8_GetCurrentBank());

    const AppConfig_t* spareConfig    = G8_GetAppConfig(G8_GetSpareBank());
    upgradeRegisters.SpareMemStart  = G8_SpareMemStart();
    upgradeRegisters.SpareMemEnd    = G8_SpareMemEnd();
    upgradeRegisters.SpareGoldenCRC = spareConfig->crc;
    upgradeRegisters.SpareCalcedCRC = G8_CalculateBankCrcU16(G8_GetSpareBank());
}

int16_t G8_UpgradeErase(uint8_t* cmdData, uint16_t sizeOfCmdData, uint16_t start, uint16_t end) {
    int16_t status;
    
    uint32_t minAddress = G8_SpareMemStart();
    uint32_t maxAddress = G8_SpareMemEnd();
    
    if (start == 0 && end == 4) {
        uint32_t address = *(uint32_t*)cmdData;
        
        address &= ~(FLASH_PAGE_SIZE-1); //truncate down to page size
        
        if (address >= minAddress && address < maxAddress ) {
            status = FlashErasePage(address, FLASH_PASSWORD);
        } else {
            status =  G8_UPGRADE_ERRNO_FLASH_ADDRESS_OUT_OF_SPARE;
        }
    } else {
        status =  G8_UPGRADE_ERRNO_BAD_PARAM;
    }
    return status;
}

extern "C"
int16_t G8_UpgradeEraseAll(void) {
    int16_t status = 0;
    
    uint32_t minAddress = G8_SpareMemStart();
    uint32_t maxAddress = G8_SpareMemEnd();

    status = FlashErasePages(minAddress,maxAddress,FLASH_PASSWORD);

    return status;
    
}


#ifndef MAXIMUM_UPGRADE_IMAGE_LINE_SIZE
#define MAXIMUM_UPGRADE_IMAGE_LINE_SIZE 272        
#endif


int16_t G8_UpgradeWriteData(const uint8_t* ptr0, uint16_t sizeOfData, uint32_t rawLength) {
    uint8_t workingBuffer[MAXIMUM_UPGRADE_IMAGE_LINE_SIZE + sizeof(uint32_t)];
    int16_t ret = G8_UPGRADE_ERRNO_SUCCESS;
    if (rawLength < 4 || rawLength > sizeof(workingBuffer) || rawLength > sizeOfData) {
        ret = G8_UPGRADE_ERRNO_BAD_PARAM;
    } else {
        uint32_t address;
        uint32_t length;
        const uint8_t* ptr;
         
        FirmwareDecode(workingBuffer, ptr0, rawLength);
        ptr = workingBuffer;
        UnpackUInt32LE(ptr,length);
        ptr+=4;
        UnpackUInt32LE(ptr,address);
        ptr+=4;


        if (address & 3) {
            // must be on a 32-bit word boundary.
            ret = G8_UPGRADE_ERRNO_FLASH_BAD_START_ADDRESS;
        } else if (length > sizeof(workingBuffer) -8) {
            ret = G8_UPGRADE_ERRNO_FLASH_BAD_LENGTH;
        } else if (length & 3) {
            // must be a multiple of a 32-bit word
            ret = G8_UPGRADE_ERRNO_FLASH_BUFFER_ODD_SIZE;
        } else {
            
            uint32_t minAddress = G8_SpareMemStart();
            uint32_t maxAddress = G8_SpareMemEnd();

            if (address >= minAddress && address + length <= maxAddress ) {
                FlashWriteArray(address, ptr, length); // ignore the failure. We don't care. We really care if the memory is correct or not.
                ret = FlashVerifyArray(address, ptr, length);    
            } else {
                ret = G8_UPGRADE_ERRNO_FLASH_ADDRESS_OUT_OF_SPARE;
            }
        }
    }

    return ret;
}

int16_t G8_UpgradeSwitch(void) {
    // switches to the spare bank
    int currentBank = G8_GetCurrentBank();
    if (currentBank == 0 || currentBank == 1) {
        G8_SetOneTimeBootBank(1 - currentBank);
        G8_SystemReset();
        // does not return!
    }
    return G8_UPGRADE_ERRNO_BAD_PARAM;
}

int16_t G8_UpgradeCommit(void) {
    int16_t ret;
    // commits the current page as the new default page

    const AppConfig_t* appConfig = G8_GetAppConfig(G8_GetCurrentBank());
    const AppConfig_t* spareConfig = G8_GetAppConfig(G8_GetSpareBank());
    
    upgradeRegisters.AppGoldenCRC = appConfig->crc;
    upgradeRegisters.AppCalcedCRC = G8_CalculateBankCrcU16(G8_GetCurrentBank());
   
    // first check the CRC
    if (upgradeRegisters.AppGoldenCRC != upgradeRegisters.AppCalcedCRC) {
        ret = G8_UPGRADE_ERRNO_BAD_FLASH_CRC;

        // check the config words. Both should be erased.
    } else if (appConfig->boot0 == 0xFFFFFFFF && appConfig->boot1 == 0xFFFFFFFF ) {
                
        uint32_t zero = 0;
        uint32_t addr0 = (uint32_t)&appConfig->boot0;
        ret = FlashWriteArray(addr0, (const uint8_t*)&zero,4);  // mark current bank as "installed"
        if (ret == G8_UPGRADE_ERRNO_SUCCESS) {
            uint32_t addr1 = (uint32_t)&spareConfig->boot1;
            ret = FlashWriteArray(addr1, (const uint8_t*)&zero,4); // mark other bank as "uninstalled"
        }
    } else {
        ret = G8_UPGRADE_ERRNO_BAD_PARAM;
    }

    upgradeRegisters.AppBoot0   = appConfig->boot0;
    upgradeRegisters.AppBoot1   = appConfig->boot1;
    upgradeRegisters.SpareBoot0 = spareConfig->boot0;
    upgradeRegisters.SpareBoot1 = spareConfig->boot1;
            
    return ret;
}


#define ByteOffset(str,field) (((uint32_t)&str.field) - (uint32_t)&str)

#if 0
int16_t G8_UpgradeReadRegister(uint8_t* cmdData,  uint16_t sizeOfCmdData,  uint16_t start, uint16_t end) {
	if (start < end  && (end - start) <= sizeOfCmdData) {
            if (end > sizeof(upgradeRegisters)) {
                end = sizeof(upgradeRegisters);
            }
            volatile uint32_t x = ByteOffset(upgradeRegisters,SpareGoldenCRC);
            if (start <= ByteOffset(upgradeRegisters,SpareGoldenCRC) && end > ByteOffset(upgradeRegisters,SpareGoldenCRC)+3) {
                upgradeRegisters.SpareGoldenCRC = AppBootSpareGoldenCRC();
            }
            if (start <= ByteOffset(upgradeRegisters,SpareCalcedCRC) && end > ByteOffset(upgradeRegisters,SpareCalcedCRC)+3) {
                upgradeRegisters.SpareCalcedCRC = AppBootSpareCalculatedCRC();
            }

            memcpy(cmdData, ((uint8_t*)&upgradeRegisters)+start, end-start);
            return G8_UPGRADE_ERRNO_SUCCESS;
	} else {
            return G8_UPGRADE_ERRNO_BAD_PARAM;
	}
}
#endif


// warning: destroys hexline
// returns number of bytes written
extern "C"
int16_t G8_UpgradeHexLine(char* hexline) {

    static uint32_t page_address = 0;
    
    uint8_t binary[1+2+1+16+1];
    if (hexline[0] != ':') return -1;
    
    int len = strlen(hexline);
    if (len % 2 == 0) len--;
    int nbytes = (len -1)/ 2;
    if (nbytes > sizeof(binary)) {
        return G8_UPGRADE_ERRNO_BAD_INTEL_HEX_LINE;
    }
    char* ptr = hexline + len - 2;
    for(int i= nbytes-1;i >=0; i--) {
        ptr[2] = 0; // terminate
        binary[i] = strtol(ptr,NULL,16);
        ptr -= 2;
    }
    
    len = binary[0];
    if (len > sizeof(binary)) {
        return G8_UPGRADE_ERRNO_BAD_INTEL_HEX_LINE;
    }
    
    uint16_t offset = ((uint16_t)binary[1] << 8) + binary[2];
    uint8_t cmd = binary[3];
    uint8_t checksum = 0;
    for(int i=0;i<len+5;i++) checksum += binary[i];
    int bytesWritten = 0;
    if (checksum == 0) {
        switch(cmd) {
        case 0x00:  // data record
            {
                uint32_t address = page_address + offset;
                uint32_t minAddress = G8_SpareMemStart();
                uint32_t maxAddress = G8_SpareMemEnd();
                if (address >= minAddress && address + len <= maxAddress) {
                    //printf("%X - %X WRITE\n",address,len);
                    FlashWriteArray(address, binary+4, len);
                    bytesWritten = len;
                } else {
                    //printf("%X - %X SKIP\n",address,len);
                }
            }
            break;
        case 0x02: ///   Extended segment address record
            page_address = (uint32_t)offset << 4;
            break;
        case 0x03: // segment start
            // ignore
            break;
        case 0x04: // Extended linear address record
            page_address = ((uint16_t)binary[4] << 24 | (uint16_t)binary[5] << 16);
            break;
        case 5: //    Start linear address record ?
            break;  // Ignore it, since we have no influence    on exectuion start address.
        case 1: //end of file
            return 10000;
            break;
        default:
            return G8_UPGRADE_ERRNO_BAD_INTEL_HEX_LINE;
        }
    } else {
        return G8_UPGRADE_ERRNO_BAD_CHECKSUM;
    }
    
    return bytesWritten;
}

void G8_CompleteUpgrade(void) {
    const AppConfig_t* spareConfig  = G8_GetAppConfig(G8_GetSpareBank());
    
    upgradeRegisters.SpareGoldenCRC = spareConfig->crc;
    upgradeRegisters.SpareCalcedCRC = G8_CalculateBankCrcU16(G8_GetSpareBank());
}
