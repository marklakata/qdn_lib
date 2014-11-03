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

#include "qdn_parameter.h"
#include "qdn_flash.h" // generic flash interface
#include "qdn_util.h"

//#include "memory_map.h"
#include <string.h>
#include "qdn_project.h"


static  int32_t    parameter[NUM_PARAMETERS] = {0};
static  Callback_t callbacks[NUM_PARAMETERS] = {0};
static  uint8_t    statusx[(NUM_PARAMETERS+3)/4] = {0};
typedef uint32_t   ParamIndex_t;

#define STATUS_UNKNOWN     0
#define STATUS_AT_DEFAULT  1
#define STATUS_MODIFIED    2
#define STATUS_ERASED      3


//lint +e651

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////


#define MAGIC_FLASH_PARAMETER 0xA5

#ifndef MAX_PARAMETER_NUMBER
	#ifdef NUM_PARAMETERS
		#define MAX_PARAMETER_NUMBER (NUM_PARAMETERS-1)
	#else
		#error Please define MAX_PARAMETER_NUMBER
	#endif
#endif
#ifndef PARAM_START
#error Please define PARAM_START in your project.h file (inclusive memory address)
#endif
#ifndef PARAM_END
#error Please define PARAM_END in your project.h file (inclusive memory address)
#endif
#ifndef FLASH_PAGE_SIZE
	#if defined(STM32F10X_XL)
		#define FLASH_PAGE_SIZE 0x800
	#else
		#error Please define FLASH_PAGE_SIZE in your project.h file.
	#endif
#endif
#ifndef NUM_PARAM_PAGES
	#define NUM_PARAM_PAGES (((PARAM_END)-(PARAM_START))/(FLASH_PAGE_SIZE))
#else
	#error Please define NUM_PARAM_PAGES in your project.h file.
#endif

#ifndef PAGE_ERASED
	#if defined(STM32F10X_XL) || defined(STM32F1xx) || defined(STM32F4xx)
		#define PAGE_ERASED 0xFF
	#elif defined(STM32L1xx)
		#define PAGE_ERASED 0x00
	#else
		#error please define PAGE_ERASED as the value a byte takes when flash memory is bulk erased (usually FF sometimes 00)
	#endif
#endif


#if PAGE_ERASED == 0xFF
#define U8_ERASED  0xFF
#define U8_INVALID 0x00
#define U16_ERASED 0xFFFF
#define U32_ERASED 0xFFFFFFFF
#elif PAGE_ERASED == 0x00
#define U8_ERASED  0x00
#define U8_INVALID 0xFF
#define U16_ERASED 0x0000
#define U32_ERASED 0x00000000
#else
#error
#endif

class ParamOverride_t {
public:
    uint32_t value;
    uint16_t index;
    uint8_t command;
    uint8_t checksum;
    
    bool ValidateChecksum(void) {
        if (command == COMMAND_COOKIE) {
            return (CalcChecksum() == checksum);
        } else {
            return false;
        }
    }
    
    bool IsEOF() {
        return (command == COMMAND_EOF && index == U16_ERASED && value == U32_ERASED);
    }
    

public:
    void Zero(uint32_t address) {
        // corrupted record. get rid of it.
        memset(this,PAGE_ERASED,sizeof(ParamOverride_t));
#if PAGE_ERASED == 0xFF
        FlashWriteArray(address,(uint8_t*)this,sizeof(ParamOverride_t));
#elif PAGE_ERASED == 0x00
        // the STM32L has weird behavior when writing over previously written data
        // this technique of writing each bit seems to work, although it is not proven
        // to work.
        if (sizeof(ParamOverride_t) == 8)
        {
            uint64_t pattern = 0x1111111111111111;
            for(int i=0;i<8;i++) {
                FlashWriteArray(address,(uint8_t*)&pattern,sizeof(ParamOverride_t));
                pattern <<=1;
            }
        } else {
        	QDN_Exception();;
        }
#endif
    }
    
    /*lint -e{662} inhibit complaints about size being wrong */
    void WriteFlash(MemoryAddress_t address) {
        command = COMMAND_COOKIE;
        checksum = CalcChecksum();
    
        // the command/checksum is the last half-word written, so that the value and index can not be 
        // confirmed until the command/checksum is written
        FlashWriteArray(address,(uint8_t*)this,sizeof(ParamOverride_t));
    }

    enum {
        COMMAND_INVALID         = U8_INVALID, // 00000000
        COMMAND_COOKIE          = 0xB0, // 10110000
        COMMAND_EOF             = U8_ERASED  // 11111111
    };

private:
    uint8_t CalcChecksum() {
        const uint8_t* d = (const uint8_t*)(this);
        uint16_t xsum = (uint16_t)d[0]+d[1]+d[2]+d[3]+d[4]+d[5]+d[6]; //lint !e415 !e416
        xsum = ((xsum) + (xsum>>8)) & 0xFF;
        return (uint8_t) xsum;
    }

};

///////////////////////////////////////////////////////////////////////////////////////////////////
ParamOverride_t poverride;


class ParameterDatabase {
public:
    uint16_t CheckSum(uint16_t index,uint32_t value) {
        uint32_t checksum = 0x1234 + index + (value & 0xFFFF) + (value >> 16);
        checksum += (checksum >> 16); 
        return (uint16_t)(checksum & 0xFFFF);
    }

    
    void SetParamDefault(uint16_t index, int32_t value, Callback_t callback) {
        if (index <= MAX_PARAMETER_NUMBER) {
            parameter[index] = value;
            callbacks[index] = callback;
            SetStatus(index,STATUS_AT_DEFAULT);
        }
    }
    
    void SetParameterRaw(uint16_t index, int32_t value) {
        if (index <= MAX_PARAMETER_NUMBER) {
            parameter[index] = value;
            SetStatus(index, STATUS_MODIFIED);
            uint8_t option = 0;
            ParamWriteToFlashWithCleanup(index,value,option);
        }
    }
    void SetParameterWithCallback(uint16_t index, int32_t value) {
    	SetParameterRaw(index,value);
        if (index <= MAX_PARAMETER_NUMBER) {
        	if (callbacks[index]!=NULL) {
        		(*callbacks[index])();
        	}
        }
    }
    void ParamErase(uint16_t index) {
        if (index <= MAX_PARAMETER_NUMBER) {
            SetStatus(index, STATUS_ERASED);
            uint8_t option = 1;
            ParamWriteToFlashWithCleanup(index,0,option);
        }
    }
    const int32_t& ReadParameter(uint16_t index) {
        if (index <= MAX_PARAMETER_NUMBER) {
            return parameter[index];
        } else {
            return zero;
        }
    }
    uint8_t ReadParameterStatus(uint16_t index) {
        uint8_t val = PAGE_ERASED;
        if (index <= MAX_PARAMETER_NUMBER) {
            val = GetStatus(index);
        }
        return val;
    }

    void Init(void) {
        MemoryAddress_t endOfPage;                          
    
        uint8_t totalFound = 0; 
        uint8_t ipage; 
        uint8_t found[NUM_PARAM_PAGES];
    
#if 1
        int actPages = (((PARAM_END - PARAM_START) + 1)/(FLASH_PAGE_SIZE));
        int desPages = NUM_PARAM_PAGES;

        if (actPages != desPages) {
        	QDN_Exception();;
        }
#endif
    
    //    HW_ResetWatchdog();
    //    HW_ResetExternalWatchdog();
    
        // see how many pages have been written. hopefully only 0 or 1.
        for(ipage =0; ipage<NUM_PARAM_PAGES;ipage++) {
            volatile void* ptr = &poverride;
            volatile uint8_t* ptr2 = (uint8_t*)ptr;
            FlashReadArray((ipage * FLASH_PAGE_SIZE)+PARAM_START,(uint8_t*)ptr2,sizeof(poverride));
            if  (!poverride.IsEOF()) {
                found[ipage]=1;
                totalFound++;
                paramCurrentPage = ipage;
            } else {
                found[ipage]=0;
            }
        }
        if (totalFound == 2) {
            // delete the newer page. the newer page is more likely to be incomplete.
            uint8_t erasurePage;
            if (paramCurrentPage == (NUM_PARAM_PAGES-1) && found[0]) { // wrap around
                erasurePage      = 0;
            } else {
                erasurePage      = paramCurrentPage;
                paramCurrentPage = paramCurrentPage-1;
            }
            if (erasurePage < NUM_PARAM_PAGES) {
                FlashErasePage((erasurePage * FLASH_PAGE_SIZE)+PARAM_START,FLASH_PASSWORD);
            }
        } else if (totalFound > 2) {
            // unable to recover. reset everything. this should never happen, but it might if there is a cosmic event. 
            ParamResetAll();
        } else if (totalFound == 1) {
            // ok. don't do anything.
        } else if (totalFound == 0) {
            paramCurrentPage = 0;
        }
            
         // Now load overrides from flash
        paramNextDefinitionAddress= ( paramCurrentPage    * FLASH_PAGE_SIZE)+PARAM_START;
        endOfPage                 = ((paramCurrentPage+1) * FLASH_PAGE_SIZE)+PARAM_START;
    
    //    HW_ResetWatchdog();
    //    HW_ResetExternalWatchdog();
    
        while(1) {
            ParamOverride_t poverride;
            // load the next override object
            FlashReadArray(paramNextDefinitionAddress,(uint8_t*)&poverride,sizeof(poverride));
    
            // do something with it
            switch(poverride.command ) {
            case (ParamOverride_t::COMMAND_EOF ):
                if (poverride.IsEOF()) {
                    goto exitNewParamLoop;
                } else {
                    // corrupted record. get rid of it.
                    poverride.Zero(paramNextDefinitionAddress);
                }
    
                break;
            case (ParamOverride_t::COMMAND_COOKIE ):
                if (poverride.ValidateChecksum()) {
                    if (poverride.index <= MAX_PARAMETER_NUMBER ) {
                        parameter[poverride.index] = poverride.value;
                        SetStatus(poverride.index,2);
                    }
                }
                break;
            default:
            case( ParamOverride_t::COMMAND_INVALID ):
                break;
            }
            paramNextDefinitionAddress+= sizeof(poverride);
            if (paramNextDefinitionAddress >= endOfPage) break;
    
    //        HW_ResetWatchdog();
    //        HW_ResetExternalWatchdog();
        }
    exitNewParamLoop:
        
        return;
    }
    void EraseAll(void) {
        uint32_t ipage;
        for(ipage =0; ipage<NUM_PARAM_PAGES;ipage++) {
            FlashErasePage((ipage * FLASH_PAGE_SIZE)+PARAM_START,FLASH_PASSWORD);
        }
        QDN_ParamInit();
    }

private:

    void SetStatus(int index, int flag) {
        uint8_t mask = 0x03;
        int shift = (index %4)*2;
        statusx[index/4] = (statusx[index/4] & ~(mask << shift)) | (flag << shift);
    }
    uint8_t GetStatus(int index) {
        uint8_t mask = 0x03;
        int shift = (index %4)*2;
        return (statusx[index/4]>>shift)&mask;
    }
        
    void ParamWriteToFlashWithCleanup(uint16_t num, uint32_t value, uint8_t resetDefaultFlag) {
       
    
    // New improved param method.  There are 4 pages. 1 page will support 512/4 = 128 parameters.
    // Only 1 page will be active at any time.
    // If current page n is full, then 
    //  erase next page  n+1(mod 4)
    //  condense page n to n+1, overwriting new param if found
    //  append new param to page n+1 if not found
    //  set current page to n+1
    // else
    //  nullify new param within page n if found
    //  append new param to page 
    
    // on power up, (which is the only time values are read), read all pages, and do any repairs necessary
    
        MemoryAddress_t oldPageAddress;
    
        oldPageAddress             = (paramCurrentPage * FLASH_PAGE_SIZE)+PARAM_START;
    
        // check to see if there is enough space to append new value in this page
    
        if ((paramNextDefinitionAddress - oldPageAddress) >= FLASH_PAGE_SIZE) { // start of a new page
            // time for garbage collection.
    #define MAX_PARAMS_PER_PAGE (FLASH_PAGE_SIZE/sizeof(ParamOverride_t))
            uint16_t iParam = 0;
            MemoryAddress_t tempAddress;
            uint8_t overriden = 0;
            uint8_t nextPage;
    
            // This condenses the parameter page, removing the 0x00000000 records
            // only copy valid records.
            // if the old param is found, replace it.
    
            nextPage                   = (paramCurrentPage +1)&(NUM_PARAM_PAGES-1);
    
            paramNextDefinitionAddress = (nextPage        * FLASH_PAGE_SIZE)+PARAM_START;
    
            tempAddress = oldPageAddress;
    
            while(1) {
                ParamOverride_t poverride;
                // load the next poverride object
                FlashReadArray(tempAddress,(uint8_t*)&poverride,sizeof(poverride));
    
                // do something with it
                switch(poverride.command ) {
                case (ParamOverride_t::COMMAND_INVALID ):
                    break;
                case (ParamOverride_t::COMMAND_EOF ):
                    goto endGarbageCollection;
    
                case (ParamOverride_t::COMMAND_COOKIE ):
                default:
                    if (poverride.index == num) {
                        // found it
                        if (resetDefaultFlag) {
                            // skip it, so that it gets erased
                        } else {
                            poverride.value = value;
                            overriden = 1;
                            poverride.WriteFlash(paramNextDefinitionAddress);
                            paramNextDefinitionAddress += sizeof(ParamOverride_t);
                        }
                    } else {
                        poverride.WriteFlash(paramNextDefinitionAddress);
                        paramNextDefinitionAddress += sizeof(ParamOverride_t);
                    }
                    break;
                }
                tempAddress += sizeof(ParamOverride_t);
                iParam++;
                if (iParam >= MAX_PARAMS_PER_PAGE) break;  // only look at one page.
    //            HW_ResetWatchdog();
    //            HW_ResetExternalWatchdog();
            }        
    
    
    endGarbageCollection:
        
            if (!overriden && !resetDefaultFlag) {
                ParamOverride_t poverride;
                poverride.index   = num;
                poverride.value   = value;
            
                poverride.WriteFlash(paramNextDefinitionAddress);
                paramNextDefinitionAddress += sizeof(ParamOverride_t);
            }
            paramCurrentPage = nextPage;
    
            FlashErasePage(oldPageAddress,FLASH_PASSWORD);
    
        } else {
            ParamOverride_t poverride;
            MemoryAddress_t scanAddress;
            ParamIndex_t count = 0;
            MemoryAddress_t staleAddress = 0;
            uint8_t watchdogCounter = 0;
        
            // find any old references, and invalidate them
    
            scanAddress             = oldPageAddress;
    
            while(1) {
                // load the next override object
                FlashReadArray(scanAddress,(uint8_t*)&poverride,sizeof(poverride));
        
                // do something with it
                switch(poverride.command) {
        
                case ParamOverride_t::COMMAND_INVALID:
                    break;
                case ParamOverride_t::COMMAND_EOF:
                    if (poverride.IsEOF()) {
                        goto foundEndOfParameters;
                    } else {
                        // corrupted record. get rid of it.
                        poverride.Zero(scanAddress);
                    }
                    break;
                case ParamOverride_t::COMMAND_COOKIE :
                    if (poverride.index == num) {
                        if (staleAddress) {     // wow, we've seen this before! this must be a result of junk, so kill it now.
                            poverride.Zero(staleAddress);
                        }
                        // don't kill it yet, since we want to write the new value in first
                        staleAddress = scanAddress;
                    }
                    break;
                default:
                    break;
                }
                scanAddress+= sizeof(poverride);
                count++;
                if (count > FLASH_PAGE_SIZE/sizeof(ParamOverride_t)) break;
                
                if (watchdogCounter++ > 50) {
    //                HW_ResetWatchdog();
    //                HW_ResetExternalWatchdog();
                    watchdogCounter = 0;
                }
            }
    
    foundEndOfParameters:
            if (!resetDefaultFlag) {
                poverride.index   = num;
                poverride.value   = value;
        
                poverride.WriteFlash(paramNextDefinitionAddress);
                paramNextDefinitionAddress += sizeof(ParamOverride_t);
            }
    
            if  (staleAddress) {
                poverride.Zero(staleAddress);
            }
        }
    
    }
    
    void ParamResetAll(void) {
        uint8_t ipage;
        for(ipage =0; ipage<NUM_PARAM_PAGES;ipage++) {
            FlashErasePage((ipage * FLASH_PAGE_SIZE)+PARAM_START,FLASH_PASSWORD);
        }
        paramNextDefinitionAddress=PARAM_START;
    
        paramCurrentPage = 0;
    }

private:
    uint32_t         paramError;                  // a variable used for debugging only
    MemoryAddress_t  paramNextDefinitionAddress;  // pointer to next available parameter address 
    uint32_t         paramCurrentPage;                    // the page that is currently being used.

    static int32_t zero ;
};

int32_t ParameterDatabase::zero = 0;

static ParameterDatabase paramDb;

// public interface functions
extern "C"  void QDN_ParamSetRaw(uint16_t index, int32_t value) {
    paramDb.SetParameterRaw(index,value);
}

extern "C"  void QDN_ParamSetFloatRaw(uint16_t index, float value) {
	union
	{
		float f32;
		int32_t i32;
	} buffer;
	buffer.f32 = value;
    paramDb.SetParameterRaw(index,buffer.i32);
}

extern "C"  void  QDN_ParamSetString(uint16_t firstIndex, uint16_t lastIndex, char* buffer, int32_t length)
{
	for(uint16_t index=firstIndex;index<=lastIndex;index++)
	{
		int32_t ivalue = 0;
		if (length >0)
		{
			memcpy(&ivalue,buffer,4);
			length-= 4;
			buffer += 4;
		}
		paramDb.SetParameterRaw(index,ivalue);
	}
}


extern "C"  void QDN_ParamSetWithCallback(uint16_t index, int32_t value) {
    paramDb.SetParameterWithCallback(index,value);
}

extern "C"  void QDN_ParamErase(uint16_t index) {
    paramDb.ParamErase(index);
}

// public interface functions
extern "C"  void QDN_ParamSetDefault(uint16_t index, int32_t value, Callback_t callback) {
    paramDb.SetParamDefault(index,value,callback);
}

extern "C" int32_t QDN_ParamInt32(uint16_t index) {
    return paramDb.ReadParameter(index);
}

extern "C" float QDN_ParamFloat(uint16_t index) {
	union
	{
		float f32;
		int32_t i32;
	} buffer;
	buffer.i32 = paramDb.ReadParameter(index);

    return buffer.f32;
}

// maxChars includes trailing null
extern "C" int QDN_ParamString(uint16_t index, char* buffer, int32_t maxChars) {
    if (maxChars < 1) return 1;
    char* ptr = (char*) &parameter[index];
    strncpy(buffer,ptr,maxChars);
    buffer[maxChars-1] = 0;
    return 0;
}

const int32_t& QDN_ParamRef(uint16_t index) {
    return paramDb.ReadParameter(index);
}

extern "C" uint8_t QDN_ParamStatus(uint16_t index) {
    return paramDb.ReadParameterStatus(index);
}


extern "C" void QDN_ParamInit(void){
    paramDb.Init();
}

extern "C" void QDN_ParamEraseAll(void)
{
    paramDb.EraseAll();
}

extern "C" void QDN_ParamMemoryCopy(uint8_t* buffer, uint16_t start, uint16_t end) {
    memset(buffer,0,end-start);
    if (start >= sizeof(parameter)) {
       return;
    }
    if (end > sizeof(parameter)) {
      end = sizeof(parameter);
    }
    if (end > start) {
        memcpy(buffer, ((uint8_t*)parameter) + start, end - start);
    }
}


////////////////////////////////////////////////////////////////////////////
// $Id: param.c 3719 2009-07-28 06:04:34Z mlakata $
//
// Flash parameter management.
//
// Flash parameters are stored on the 8051F060 internal flash memory. Parameters
// have a default value stored in code space that is loaded on initialization.
// Then the param storage area of the flash is read to override the default values.
//
// The starting state for the param storage pages are bytes of 0xFF. Once flash is written
// to non-0xFF, it can not be rewritten until the entire page is erased.
//
// The param storage area is a sequential list of records, stored in a circular list of
// pages.  New records are appended the end of the current page, and old records on that 
// page are written to zero to mark them as erased.
// When the parameter page is filled, a garbage collection occurs. The process copies
// the non-zero parameters to the next page, and then erases the old page to 0xFF in bulk.
//
// ParamOverride_t is the type of the record. 
//
// For STR912, the erase command for a block takes a long time (500 to 1000 ms). This is 
// an unacceptable delay for a single param write. To address this, the writing of parameters
// is done in a separate write/garbage collection task. The write requests are queued
// to the task, and the task does all of the grunt work. Note that the parameter values are immediately
// cached in RAM, so the value is immediately available to the rest of the world. The only thing that is
// delayed is the write-thru to flash memory.
// To enable this option, define PARAM_WRITE_TASK
////////////////////////////////////////////////////////////////////////////
