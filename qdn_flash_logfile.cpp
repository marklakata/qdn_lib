#include "g8_flash_logfile.h"
#include "g8_flash.h"
#include "errno.h"
#include "project.h"
#include <string.h>
#include <stdlib.h>


#define Assert(a)  assert(a,#a,__FILE__,__LINE__)

#define PAGE_COOKIE 0x9D

#ifndef PAGE_ERASED
#define PAGE_ERASED 0xFF
#endif

// This structure describes the contents of the flash memory page. If the page is not valid, then the cookie will not be the PAGE_COOKIE
// value. The offset is the byte count to the first record that starts in this page. 
// The serial number is increments from page to page, to determine which is the "last" page. For example, if the 6 pages have serial numbers that look like
//  123,124,125,126,121,122
// then page 3 is the last page and page 4 is the first page of the file.

class PerfPageHeader_t {
public:
    uint8_t cookie;
    uint8_t serial;
    uint16_t offset;
    uint8_t* RawDataPtr(int offset) {
        return ((uint8_t*)&(this->cookie) + offset);
    }
} ;

class G8_HeaderOnlyRecord : public G8_LogfileRecordBase {
public:
    virtual uint32_t CalculateSize() { return G8_LogfileRecordBase::HeaderSize(); }
};


/////////////////////////////////////////////////////////////


G8_Flash_Logfile::G8_Flash_Logfile(uint32_t startAddr0, uint32_t endAddr0) :
    disable(false),
    startAddr(startAddr0),
    endAddr(endAddr0) ,
    lastSerial(0)
{
    Init();
}

G8_Flash_Logfile::G8_Flash_Logfile() :
    disable(false),
    startAddr(0),
    endAddr(0) ,
    lastSerial(0)   
{
}

void G8_Flash_Logfile::SetMemory(uint32_t startAddr0,uint32_t endAddr0) {
    startAddr = startAddr0;
    endAddr = endAddr0;
    Init();
}


void G8_Flash_Logfile::Init() {
    lastSerial = 0;
    ParseLog();
}



#ifdef SIMULATOR
void* GetFlashPtr(uint32_t addr);
#define GET_PTR_TO_FLASH(type, addr)   ((type)GetFlashPtr(addr))
#else
#define GET_PTR_TO_FLASH(type, addr)   ((type)addr)
#endif


// if the current page is full (or there is no current page), this creates a new fresh page to be written,
// as well as wrapping the first-page pointer if necessary.
uint32_t G8_Flash_Logfile::StartNewPage(uint32_t addr, uint16_t offset) {
    if (disable) return 0;
    
    if (addr >= EndAddr()) {
        addr = StartAddr();
    }
    Assert((addr & (FLASH_PAGE_SIZE-1)) == 0);

    PerfPageHeader_t newPageHeader;
    newPageHeader.cookie = PAGE_COOKIE;
    newPageHeader.serial = lastSerial + 1;
    newPageHeader.offset = offset + sizeof(newPageHeader);
    lastSerial = newPageHeader.serial;

    if (firstPage == 0) {
        firstPage = addr;
    } else {
        // handle circular start-of-file page pointer
        if ((addr & ~(FLASH_PAGE_SIZE-1)) == (firstPage & ~(FLASH_PAGE_SIZE-1))) {
            firstPage += FLASH_PAGE_SIZE;
            if (firstPage >= EndAddr()) {
                firstPage = StartAddr();
            }
        }
    }
    lastPage = addr;

    FlashErasePage(addr, FLASH_PASSWORD );
    FlashWriteArray(addr, newPageHeader.RawDataPtr(0) , sizeof(newPageHeader) );
    addr += sizeof(newPageHeader); // move to the first empty spot on the page (just past the header)
    return addr;
}

// for read only access, this increments the pointer to find the next record on the next page
uint32_t G8_Flash_Logfile::IncrementDataPointer(uint32_t addr, uint32_t incr) {
    uint32_t oldPage = addr & ~(FLASH_PAGE_SIZE-1);
    addr += incr;
    
    // after the increment, addr *should* line up on a page boundary, because the only time this is 
    // called is on a page boundary.
    if ((addr & (FLASH_PAGE_SIZE-1)) == 0) {
        
        if (addr >= EndAddr()) {
            addr = StartAddr();
        }
        addr += sizeof(PerfPageHeader_t);
    } else {
        uint32_t newPage = (addr & ~(FLASH_PAGE_SIZE-1));
        Assert(oldPage == newPage);
    }
    return addr;
}

int G8_Flash_Logfile::IntegrityCheck(void) {
    for(uint32_t page  = StartAddr(); page < ( EndAddr()); page += FLASH_PAGE_SIZE) {
        PerfPageHeader_t* header = GET_PTR_TO_FLASH(PerfPageHeader_t*, page);
        if (header->cookie != PAGE_COOKIE && header->cookie != PAGE_ERASED) {
            return 2;
        }
    }
    return 0;
}



// this function finds the first and last pages of the log. It also
// repairs any corruption due to incomplete records being written.
int16_t G8_Flash_Logfile::ParseLog(void) {
    firstPage = 0;
    lastPage = 0;

    // find the point where the page serial number runs backwards. that is the transition between
    // the newest page and the oldest page.
    uint32_t previousPage = 0;
    bool first= true;
    for(uint32_t page  = StartAddr(); page < ( EndAddr()); page += FLASH_PAGE_SIZE) {
        PerfPageHeader_t* header = GET_PTR_TO_FLASH(PerfPageHeader_t*, page);
        if (header->cookie == PAGE_COOKIE) {
            if (first) {
                firstPage = page;
            } else {
                // look for break in sequence
                if (header->serial != (uint8_t)(lastSerial+1)) {
                    firstPage = page;
                    lastPage = previousPage;
                    break;
                }
                if ((page == EndAddr() - FLASH_PAGE_SIZE) && lastPage == 0) {
                    lastPage = page;
                }
            }
            lastSerial = header->serial;
            first = false;
            previousPage = page;
        } else if (header->cookie == PAGE_ERASED) {
            lastPage = previousPage;
        } else {
            Assert(0);
            return ERRNO_FLASH_VERIFY_FAILED;
        }
    }
    Assert((firstPage!=0) == (lastPage !=0)); // they both have to be zero or both non-zero

    // if nothing exists, then start a new file
    if (firstPage == 0) {
        nextLogRecordAddress = StartNewPage(StartAddr(),0);
    } else {
        G8_HeaderOnlyRecord readBuffer;
        nextLogRecordAddress = Rewind();
        uint32_t nextAddr;
        while ((nextAddr = ReadRecord(nextLogRecordAddress,&readBuffer, false)) != 0 ) {
            nextLogRecordAddress = nextAddr;
        }
    }
    
    return ERRNO_SUCCESS;
}

uint32_t G8_Flash_Logfile::ReadRecord(uint32_t addr0, G8_LogfileRecordBase* rec, bool loadPayload) {
    if (addr0 < startAddr || addr0 > endAddr) {
        return 0;
    }
    Assert(rec != NULL);

    uint32_t addr = addr0;
    uint32_t bytesLeftOnPage =  FLASH_PAGE_SIZE - (addr & (FLASH_PAGE_SIZE-1));

    // read fixed header. need to load it first to determine record length.
    if (bytesLeftOnPage < rec->HeaderSize()) {
        FlashReadArray(addr, rec->RawDataPtr(0), bytesLeftOnPage);
        addr = IncrementDataPointer(addr, bytesLeftOnPage);
        FlashReadArray(addr, rec->RawDataPtr(bytesLeftOnPage), rec->HeaderSize() - bytesLeftOnPage);
        addr = IncrementDataPointer(addr, rec->HeaderSize() - bytesLeftOnPage);
    } else {
        FlashReadArray(addr, rec->RawDataPtr(0), rec->HeaderSize());
        addr = IncrementDataPointer(addr, rec->HeaderSize() );
    }

    // read variable data
    if (rec->cookie == LOG_COOKIE) {
        uint32_t remaining = rec->size - rec->HeaderSize();
        uint32_t toread = rec->CalculateSize() - rec->HeaderSize();
        if (toread > remaining) toread = remaining;
        
        bytesLeftOnPage =  FLASH_PAGE_SIZE - (addr & (FLASH_PAGE_SIZE-1));
        if (bytesLeftOnPage < remaining) {
            if(loadPayload) {
                uint32_t toRead2 = toread;
                if (toRead2 > bytesLeftOnPage) toRead2 = bytesLeftOnPage;
                FlashReadArray(addr, rec->RawDataPtr(rec->HeaderSize()), toRead2);
                toread -= toRead2;
            }
            addr = IncrementDataPointer(addr, bytesLeftOnPage);
            if(loadPayload) {
                FlashReadArray(addr, rec->RawDataPtr(rec->HeaderSize() + bytesLeftOnPage), toread);
            }
            addr = IncrementDataPointer(addr, remaining - bytesLeftOnPage );
        } else {
            if(loadPayload) {
                FlashReadArray(addr, rec->RawDataPtr(rec->HeaderSize()), toread);
            }
            addr = IncrementDataPointer(addr, remaining );
        }
    } else {
        return 0;
    }

    // new addr is either 0, or points to the start of the next record (even if on the next page)
    return addr;
}

int G8_Flash_Logfile::Flush(G8_LogfileRecordBase* rec) {
    if (disable) return 1;

    Assert(rec != NULL);

    rec->cookie = LOG_COOKIE;
    rec->size = rec->CalculateSize();

    uint32_t bytesLeftOnPage =  FLASH_PAGE_SIZE - (nextLogRecordAddress & (FLASH_PAGE_SIZE-1));
    if (rec->size > bytesLeftOnPage) {
        FlashWriteArray(nextLogRecordAddress, rec->RawDataPtr(0), bytesLeftOnPage);
        nextLogRecordAddress += bytesLeftOnPage;
        uint16_t offset = rec->size - bytesLeftOnPage;
        nextLogRecordAddress = StartNewPage(nextLogRecordAddress, offset);
        FlashWriteArray(nextLogRecordAddress, rec->RawDataPtr(bytesLeftOnPage), rec->size - bytesLeftOnPage);
        nextLogRecordAddress += rec->size - bytesLeftOnPage;
    } else {
        FlashWriteArray(nextLogRecordAddress, rec->RawDataPtr(0), rec->size);
        nextLogRecordAddress += rec->size;
    }

    // check if the address pointer is pointing to the start of the next page. If so,
    // initialize the next page and increment the pointer.
    if ((nextLogRecordAddress & (FLASH_PAGE_SIZE-1)) == 0) {
        nextLogRecordAddress = StartNewPage(nextLogRecordAddress, 0);
    }
    
    return IntegrityCheck();
}

// this function is called OverwriteZero, but it actually
// writes the pattern that can not be rewritten. For example, conventional
// flash memory is erased to 0xFF and written to 0x00. Some flash memory (STM32L)
// is erased to 0x00 and written to 0xFF.
void G8_Flash_Logfile::OverwriteZero(uint32_t base, int offset, int count) {
    uint32_t address = base + offset;
    uint32_t page0 = base    & ~(FLASH_PAGE_SIZE-1);
    uint32_t page1 = address & ~(FLASH_PAGE_SIZE-1);
    if (page0 != page1) {
        address += sizeof(PerfPageHeader_t);
        if (address >  EndAddr()) {
            address -= EndAddr();
            address += StartAddr();
        }
    }
    
    if (count > 4) while(1);
    
    if ((address & 1) == 0) {
#if PAGE_ERASED == 0xFF
        uint8_t value[4] = {0,0,0,0};
        FlashWriteArray(address, value, count);
#elif PAGE_ERASED == 0x00
        uint32_t value = 0x01010101;
        for(int j=0;j<8;j++) {
            FlashWriteArray(address, (uint8_t*)&value, count);
            value <<=1;
        }
#else
#error
#endif
    }
}

void G8_Flash_Logfile::EraseAll() {
    for(uint32_t page  = StartAddr(); page < ( EndAddr()); page += FLASH_PAGE_SIZE) {
        FlashErasePage(page, FLASH_PASSWORD );
    }
    Init();
}

uint32_t G8_Flash_Logfile::Rewind() {
    uint32_t addr = 0;
    if (firstPage != 0) {
        PerfPageHeader_t* header = GET_PTR_TO_FLASH(PerfPageHeader_t*, firstPage);
        addr = (firstPage + header->offset);
    }
    return addr;
}


