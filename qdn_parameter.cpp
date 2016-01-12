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

#include <string.h>
#include "qdn_project.h"


static  int32_t    parameter[NUM_PARAMETERS] = {0};
static  Callback_t callbacks[NUM_PARAMETERS] = {0};
static  uint8_t    statusx[(NUM_PARAMETERS+3)/4] = {0}; // 2 bits per parameter
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

#define MULTIPAGE

class ParamOverride_t {
public:
    uint32_t value;
    uint16_t index;
    uint8_t command;
    uint8_t checksum;
    
    bool ValidateChecksum(void) {
#ifdef MULTIPAGE
        if (command == COMMAND_PARAM )
#else
        if (command == COMMAND_COOKIE)
#endif
        {
            return (CalcChecksum() == checksum);
        } else {
            return false;
        }
    }
    
    bool IsEOF() {
        return (command == COMMAND_EOF && index == U16_ERASED && value == U32_ERASED);
    }
#ifdef MULTIPAGE
    bool IsCopyInProgress() {
        return (command == COMMAND_COPY_IN_PROGRESS && index == U16_ERASED);
    }
#endif

public:
    void Zero(uint32_t address) {
        // corrupted record. get rid of it.
        memset(this,U8_INVALID,sizeof(ParamOverride_t));
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
    
#ifdef MULTIPAGE
    /*lint -e{662} inhibit complaints about size being wrong */
    void WriteParamToFlash(MemoryAddress_t address) {
        command = COMMAND_PARAM;
        checksum = CalcChecksum();

        // the command/checksum is the last half-word written, so that the value and index can not be
        // confirmed until the command/checksum is written
        FlashWriteArray(address,(uint8_t*)this,sizeof(ParamOverride_t));
    }
#else
    /*lint -e{662} inhibit complaints about size being wrong */
    void WriteFlash(MemoryAddress_t address) {
        command = COMMAND_COOKIE;
        checksum = CalcChecksum();
    
        // the command/checksum is the last half-word written, so that the value and index can not be 
        // confirmed until the command/checksum is written
        FlashWriteArray(address,(uint8_t*)this,sizeof(ParamOverride_t));
    }
#endif

    enum {
        COMMAND_INVALID          = U8_INVALID, // 00000000
#ifdef MULTIPAGE
        COMMAND_PARAM            = 0xB0, // 10110000
        COMMAND_COPY_IN_PROGRESS = 0x30, // 00110000
#else
        COMMAND_COOKIE           = 0xB0, // 10110000
#endif
        COMMAND_EOF              = U8_ERASED  // 11111111
    };

private:
    uint8_t CalcChecksum() {
        const uint8_t* d = (const uint8_t*)(this);
        uint16_t xsum = (uint16_t)d[0]+d[1]+d[2]+d[3]+d[4]+d[5]+d[6]; //lint !e415 !e416
        xsum = ((xsum) + (xsum>>8)) & 0xFF;
        return (uint8_t) xsum;
    }

};

#define MAX_PARAMS_PER_PAGE (FLASH_PAGE_SIZE/sizeof(ParamOverride_t))


///////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef MULTIPAGE
#else
ParamOverride_t poverride;
#endif

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
        if (index <= MAX_PARAMETER_NUMBER)
        {
            SetStatus(index, STATUS_MODIFIED);
            if (parameter[index] != value)
            {
                parameter[index] = value;
                uint8_t option = 0;
                ParamWriteToFlashWithCleanup(index,value,option);
            }
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

    void assert_error(uint8_t errorCode)
    {
        flashFailed_ = true;
        errorCallback_(errorCode);
    }

    ParameterDatabase() :
        errorCallback_([](uint8_t ec){}) ,
        flashFailed_(false)
    {

    }

    void Init(std::function<void(uint8_t)> errorCallback)
    {
        errorCallback_ = errorCallback;
        flashFailed_ = false;

#if 1
        int actPages = (((PARAM_END - PARAM_START) + 1)/(FLASH_PAGE_SIZE));
        int desPages = NUM_PARAM_PAGES;

        if (actPages != desPages) {
        	QDN_Exception();;
        }
#endif
    
    //    HW_ResetWatchdog();
    //    HW_ResetExternalWatchdog();
    
#ifdef MULTIPAGE
        // first figure out if any pages have not been fully copied
        for(uint8_t ipage =0; ipage<NUM_PARAM_PAGES;ipage++)
        {
            ParamOverride_t pov;

            // look at last record of page
            FlashReadArray((ipage * FLASH_PAGE_SIZE)+PARAM_START + FLASH_PAGE_SIZE - sizeof(pov),reinterpret_cast<uint8_t*>(&pov),sizeof(pov));
            if  (pov.IsCopyInProgress())
            {
                uint8_t targetPage = pov.value;

                if (targetPage < NUM_PARAM_PAGES)
                {
                    FlashErasePage((targetPage * FLASH_PAGE_SIZE)+PARAM_START,FLASH_PASSWORD);
                }
            }
        }

   //    HW_ResetWatchdog();
   //    HW_ResetExternalWatchdog();

        paramNextDefinitionAddress  =0;
        paramCurrentPage = 0;

       // Now load overrides from flash
       for(uint8_t ipage =0; ipage<NUM_PARAM_PAGES;ipage++)
       {
           uint32_t addr = (ipage * FLASH_PAGE_SIZE)+PARAM_START;

           // process all parameter records
           for(uint32_t iparam =0;iparam < MAX_PARAMS_PER_PAGE ;iparam++)
           {
               ParamOverride_t pov;
               // load the next override object
               FlashReadArray(addr,reinterpret_cast<uint8_t*>(&pov),sizeof(pov));

               // do something with it
               switch(pov.command )
               {
                   case (ParamOverride_t::COMMAND_EOF ):
                       if (pov.IsEOF())
                       {
                           if (paramNextDefinitionAddress == 0)
                           {
                               if (iparam < MAX_PARAMS_PER_PAGE - 1) // NOT the last record. save this for later.
                               {
                                   paramNextDefinitionAddress = addr;
                                   paramCurrentPage = ipage;
                               }
                           }
                           iparam = MAX_PARAMS_PER_PAGE;
                       }

                       break;
                   case (ParamOverride_t::COMMAND_PARAM ):
                       if (pov.ValidateChecksum())
                       {
                           if (pov.index <= MAX_PARAMETER_NUMBER )
                           {
                               parameter[pov.index] = pov.value;
                               SetStatus(pov.index,STATUS_MODIFIED);
                           }
                       }
                       break;
                   case (ParamOverride_t::COMMAND_COPY_IN_PROGRESS ):
                   case( ParamOverride_t::COMMAND_INVALID ):
                   default:
                       // skip this record
                       break;
               }
               addr  += sizeof(pov);

   //        HW_ResetWatchdog();
   //        HW_ResetExternalWatchdog();
           }
        }

       if (paramNextDefinitionAddress == 0)
       {
           assert_error(5);
           return;
       }
#else
        MemoryAddress_t endOfPage;
        uint8_t totalFound = 0;
        uint8_t found[NUM_PARAM_PAGES];
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
                        SetStatus(poverride.index,STATUS_MODIFIED);
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
#endif
        
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
        
#ifdef MULTIPAGE
    bool PageIsEmpty(uint8_t ipage)
    {
        uint32_t addr = (ipage * FLASH_PAGE_SIZE)+PARAM_START;

        ParamOverride_t poverride;
        FlashReadArray(addr,reinterpret_cast<uint8_t*>(&poverride),sizeof(poverride));

        return (poverride.command == ParamOverride_t::COMMAND_EOF );
    }

    bool PageCanBeCompressed(uint8_t ipage)
    {
        uint32_t keeper = 0;
        uint32_t addr = (ipage * FLASH_PAGE_SIZE)+PARAM_START;

        // first see if the page is full and can't be garbage collected.
        for(uint16_t iParam = 0; iParam < MAX_PARAMS_PER_PAGE - 1; iParam++)
        {
            ParamOverride_t poverride;
            // load the next poverride object
            FlashReadArray(addr,reinterpret_cast<uint8_t*>(&poverride),sizeof(poverride));

            // do something with it
            switch(poverride.command ) {
            case (ParamOverride_t::COMMAND_INVALID ):
                goto endGarbageCollection1;
            case (ParamOverride_t::COMMAND_EOF ):
                goto endGarbageCollection1;

            case (ParamOverride_t::COMMAND_PARAM ):
            default:
                keeper++;
                break;
            }
            addr += sizeof(ParamOverride_t);
        }

      endGarbageCollection1:
        return keeper < MAX_PARAMS_PER_PAGE - 1;
    }

    uint8_t GetFreeLocationWithGarbageCollection()
    {
        uint8_t freePages = 0;
        uint8_t newPage = 0xFF;

        for(uint8_t ipage =0; ipage<NUM_PARAM_PAGES;ipage++)
        {
            if (PageIsEmpty(ipage)) {
                newPage = ipage;
                freePages++;
            }
        }

        if (freePages == 0)
        {
            return 1;
        }

        // start looking at the next page
        paramCurrentPage ++;

        if (freePages == 1)
        {
            if (newPage == 0xFF)
            {
                return 3;
            }

            // get movable page
            for(uint8_t ipage2 =0; ipage2<NUM_PARAM_PAGES; ipage2++)
            {
                uint8_t ipage = ( ipage2 + paramCurrentPage ) % NUM_PARAM_PAGES;

                if (! PageIsEmpty(ipage) && PageCanBeCompressed(ipage))
                {
                    MovePageToPage(ipage, newPage);
                    paramCurrentPage = newPage;
                    goto gotNewPage;
                }
            }

            // bad thing
            return 4;
        }

    gotNewPage:
        // find a free parameter slot, starting on the currentPage and looking forward, wrapping around if necessary
        for(uint8_t ipage2 =0; ipage2<NUM_PARAM_PAGES; ipage2++)
        {
            uint8_t ipage = ( ipage2 + paramCurrentPage ) % NUM_PARAM_PAGES;

            MemoryAddress_t addr = (ipage * FLASH_PAGE_SIZE)+PARAM_START;

            for(uint16_t iparam=0;iparam < MAX_PARAMS_PER_PAGE - 1 ; iparam++)
            {
                ParamOverride_t pov;
                // load the next ParamOverride_t object
                FlashReadArray(addr,reinterpret_cast<uint8_t*>(&pov),sizeof(pov));
                if (pov.command == ParamOverride_t::COMMAND_EOF)
                {
                    paramNextDefinitionAddress = addr;
                    paramCurrentPage = ipage;
                    return 0;
                }
                addr += sizeof(ParamOverride_t);
            }
        }

        // this should not happen!
        return 2;


    }

    void MovePageToPage(uint8_t oldPage, uint8_t newPage)
    {
        {
            // mark the old page as "copy in progress" and indicate which page is being written.
            MemoryAddress_t progressAddress = (oldPage * FLASH_PAGE_SIZE)+PARAM_START + FLASH_PAGE_SIZE - sizeof(ParamOverride_t);
            ParamOverride_t pov;
            pov.command = ParamOverride_t::COMMAND_COPY_IN_PROGRESS;
            pov.value   = newPage;
            pov.WriteParamToFlash(progressAddress);
        }

        MemoryAddress_t oldPageAddress = (oldPage * FLASH_PAGE_SIZE)+PARAM_START;
        MemoryAddress_t newPageAddress = (newPage * FLASH_PAGE_SIZE)+PARAM_START;

        for(uint16_t iparam=0;iparam < MAX_PARAMS_PER_PAGE - 1 ; iparam++)
        {
            ParamOverride_t pov;
            // load the next poverride object
            FlashReadArray(oldPageAddress,reinterpret_cast<uint8_t*>(&pov),sizeof(pov));

            // do something with it
            switch(pov.command ) {
            case (ParamOverride_t::COMMAND_INVALID ):
                // don't copy
                break;

            case (ParamOverride_t::COMMAND_EOF ):
                // done copying
                goto endGarbageCollection1;

            case (ParamOverride_t::COMMAND_PARAM ):
                // copy it!
                pov.WriteParamToFlash(newPageAddress);
                newPageAddress += sizeof(ParamOverride_t);
                break;

            default:
                //don't copy it
                break;
            }
            oldPageAddress += sizeof(ParamOverride_t);
        }

    endGarbageCollection1:
        oldPageAddress = (oldPage * FLASH_PAGE_SIZE)+PARAM_START;

        FlashErasePage(oldPageAddress,FLASH_PASSWORD);
    }

    void ParamWriteToFlashWithCleanup(uint16_t num, uint32_t value, uint8_t resetDefaultFlag)
    {
        if (flashFailed_) return;

        // 1. find first free spot
        // 2. if no free spot, garbage collect until free space found
        // 3. find page that has existing param, save info if found
        // 4. append param to ptr
        // 5. invalidate old parameter


        if (paramNextDefinitionAddress == 0)
        {
            uint8_t errorCode = GetFreeLocationWithGarbageCollection();
            if (errorCode != 0)
            {
                assert_error(errorCode);
                return;
            }
        }

        uint32_t existParamAddr = 0;
        ParamOverride_t pov;

        // find existing parameter
        for(uint8_t ipage =0; ipage<NUM_PARAM_PAGES;ipage++)
        {
            uint32_t addr = (ipage * FLASH_PAGE_SIZE)+PARAM_START;

            // read all parameter records, except the last one (the copy in progress indicator)
            for(uint32_t iparam =0;iparam < MAX_PARAMS_PER_PAGE - 1;iparam++)
            {
                // load the next override object
                FlashReadArray(addr,(uint8_t*)&pov,sizeof(pov));
                if (pov.index == num && pov.command == ParamOverride_t::COMMAND_PARAM && pov.ValidateChecksum())
                {
                    // found it
                    existParamAddr = addr;
                    goto foundExistingParam;
                }
                if (pov.command == ParamOverride_t::COMMAND_EOF)
                {
                    // skip to the next bank
                    break;
                }
                addr += sizeof(ParamOverride_t);
            }
        }

     foundExistingParam: ;

        if (!resetDefaultFlag)
        {
            if (paramNextDefinitionAddress ==0)
            {
                assert_error(6);
                return;
            }

            pov.index   = num;
            pov.value   = value;
    
            pov.WriteParamToFlash(paramNextDefinitionAddress);
            paramNextDefinitionAddress += sizeof(ParamOverride_t);
            if (((paramNextDefinitionAddress - PARAM_START) & (FLASH_PAGE_SIZE-1)) >= sizeof(ParamOverride_t)*(MAX_PARAMS_PER_PAGE-1))
            {
                paramNextDefinitionAddress = 0; // clean up next time
            }
        }
        
        if  (existParamAddr) {
            pov.Zero(existParamAddr);
        }
    }
            
#else
    void ParamWriteToFlashWithCleanup(uint16_t num, uint32_t value, uint8_t resetDefaultFlag)
    {

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
            uint16_t iParam = 0;
            MemoryAddress_t tempAddress;
            uint8_t overriden = 0;
            uint8_t nextPage;
            bool eraseOldPage = false;

            // This condenses the parameter page, removing the 0x00000000 records
            // only copy valid records.
            // if the old param is found, replace it.

            nextPage                   = (paramCurrentPage +1)&(NUM_PARAM_PAGES-1);

            paramNextDefinitionAddress = (nextPage        * FLASH_PAGE_SIZE)+PARAM_START;

            tempAddress = oldPageAddress;
            uint32_t keeper = 0;

            // first see if the page is full and can't be garbage collected.
            while(1) {
                ParamOverride_t poverride;
                // load the next poverride object
                FlashReadArray(tempAddress,(uint8_t*)&poverride,sizeof(poverride));

                // do something with it
                switch(poverride.command ) {
                case (ParamOverride_t::COMMAND_INVALID ):
                    goto endGarbageCollection1;
                case (ParamOverride_t::COMMAND_EOF ):
                    goto endGarbageCollection1;

                case (ParamOverride_t::COMMAND_COOKIE ):
                default:
                    if (poverride.index == num) {
                        goto endGarbageCollection1;
                    } else {
                        keeper++;
                    }
                    break;
                }
                tempAddress += sizeof(ParamOverride_t);
                iParam++;
                if (iParam >= MAX_PARAMS_PER_PAGE) break;  // only look at one page.
            }

     endGarbageCollection1:

            if (keeper <= MAX_PARAMS_PER_PAGE)
            {
                // page is not full and should be garbage collected.
                tempAddress = oldPageAddress;
                iParam = 0;
                eraseOldPage = true;

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
                                if (!overriden)
                                {
                                    overriden = 1;
                                    poverride.value = value;
                                    poverride.WriteFlash(paramNextDefinitionAddress);
                                    paramNextDefinitionAddress += sizeof(ParamOverride_t);
                                }
                                // else skip it. No point in writing it multiple times
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

            if (eraseOldPage)
            {
                FlashErasePage(oldPageAddress,FLASH_PASSWORD);
            }
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
#endif
    
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
    std::function<void(uint8_t)> errorCallback_;
    bool flashFailed_;

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

extern "C"  void  QDN_ParamSetString(uint16_t firstIndex, uint16_t lastIndex, const char* buffer, uint32_t length)
{
     QDN_ParamSetStringWithStride(firstIndex, lastIndex, buffer,  length, 1);
}

extern "C"  void  QDN_ParamSetStringWithStride(uint16_t firstIndex, uint16_t lastIndex, const char* buffer, uint32_t length, int stride)
{
	for(uint16_t index=firstIndex;index<=lastIndex;index+= stride)
	{
		int32_t ivalue = 0;
		if (length >0)
		{
		    uint32_t tocopy = length;
		    if (tocopy > sizeof(ivalue)) tocopy = sizeof(ivalue);
			memcpy(&ivalue,buffer,tocopy);
			length -= tocopy;
			buffer += tocopy;
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
extern "C" int QDN_ParamString(uint16_t index, char* buffer, int32_t maxChars)
{
    if (maxChars < 1) return 1;
    char* ptr = (char*) &parameter[index];
    strncpy(buffer,ptr,maxChars-1);
    buffer[maxChars-1] = 0;
    return 0;
}

// maxChars includes trailing null
extern "C" int QDN_ParamStringWithStride(uint16_t index, char* buffer, int32_t maxChars, int stride) {
    if (maxChars < 1) return 1;
    int written  = 0;
    while(written < maxChars - 1)
    {
        char* ptr = (char*) &parameter[index];
        strncpy(buffer + written,ptr,sizeof(int32_t));
        index += stride;
        written += sizeof(int32_t);
    }
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
    paramDb.Init([](uint8_t){});
}

void QDN_ParamInitWithCallback(std::function<void(uint8_t)> errorCallback){
    paramDb.Init(errorCallback);
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
