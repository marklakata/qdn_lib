#ifndef _G8_FLASH_LOGFILE_H_
#define _G8_FLASH_LOGFILE_H_

#include <stdint.h>

#define LOG_COOKIE 0x106C


class G8_LogfileRecordBase {
public:
    uint16_t cookie;
    uint16_t size;  // this is the total size of the record, including this subclass members (cookie and size).
public:
    static uint32_t HeaderSize() {return 4;}
    bool ValidCookie() const { return cookie == LOG_COOKIE; }
    virtual uint32_t CalculateSize() = 0;
    uint8_t* RawDataPtr(int offset) {
        return ((uint8_t*)&(this->cookie) + offset);
    }
};

class G8_Flash_Logfile {
public:
    // public functions
    G8_Flash_Logfile();
    G8_Flash_Logfile(uint32_t startAddr0,uint32_t endAddr0); // warning: this checks the files at constructor time, which may not be desireable
    void SetMemory(uint32_t startAddr0,uint32_t endAddr0); // info: this checks the file at run time.
        
    void EraseAll();
    int  Flush(G8_LogfileRecordBase* rec);
    uint32_t Rewind();
    
    uint32_t ReadRecord(uint32_t addr0, G8_LogfileRecordBase* rec, bool loadPayload = true);

    void OverwriteZero(uint32_t base, int offset, int count);

    // public data
public:
    bool disable;

protected:
    uint32_t nextLogRecordAddress;
    uint32_t firstPage;
    uint32_t lastPage;
    uint32_t startAddr;
    uint32_t endAddr;
private:    
    uint8_t lastSerial;
    int16_t  ParseLog(void);
    int      IntegrityCheck(void);
    uint32_t IncrementDataPointer(uint32_t addr, uint32_t incr);
    uint32_t StartNewPage(uint32_t addr, uint16_t offset);
    void Init();

    uint32_t StartAddr() {
        return startAddr;
    }
    uint32_t EndAddr() {
        return endAddr;
    }        
    virtual void assert(bool flag, const char* message, const char* file, int linenumber) {
        if (!flag) {
            disable = true;
        }
    }
} ;

//
// These 2 classes supported an archive "bit" for each record, that can be cleared so 
// that the record can be marked for deletion.

class G8_ArchivedLogRecord :  public G8_LogfileRecordBase {
    friend class G8_ArchivedLogFile;
protected:
    G8_ArchivedLogRecord() : archived(
#if PAGE_ERASED == 0x00
        0x00000000
#elif PAGE_ERASED == 0xFF
        0xFFFFFFFF
#else 
#error Fix me. Set PAGE_ERASED
#endif
            ) { }
        
public:
    bool IsNew() {
#if PAGE_ERASED == 0x00
        return archived == 0x00000000;
#elif PAGE_ERASED == 0xFF
        return archived == 0xFFFFFFFF;
#else 
#error Fix me. Set PAGE_ERASED
#endif
    }
protected:
    uint32_t     archived;
    
    virtual uint32_t CalculateSize() { return HeaderSize() + sizeof(archived); } 
};

class G8_ArchivedLogFile :  public G8_Flash_Logfile {
public:
    G8_ArchivedLogFile() : unread(0) {}
    G8_ArchivedLogFile(uint32_t startAddr0,uint32_t endAddr0) 
        :  G8_Flash_Logfile(startAddr0,endAddr0)
        ,  unread(0)
    { 
    }
    
private:
    void Flush(G8_LogfileRecordBase* rec);
    uint32_t ReadRecord(uint32_t addr0, G8_LogfileRecordBase* rec, bool loadPayload = true);
public:
    int Flush(G8_ArchivedLogRecord* rec) {
        int stat = G8_Flash_Logfile::Flush(rec);
        if (stat == 0) {
            unread++;
        }
        return stat;
    }
    uint32_t ReadRecord(uint32_t addr0, G8_ArchivedLogRecord* rec, bool loadPayload = true) {
        return G8_Flash_Logfile::ReadRecord(addr0,rec,loadPayload);
    }
    
    void MarkRead(uint32_t address) 
    {
        G8_ArchivedLogRecord* ptr = (G8_ArchivedLogRecord*)address;
        int offset =  (uint32_t) &ptr->archived - (uint32_t)&ptr->cookie;
#if PAGE_ERASED == 0x00
        ptr->archived = 0xFFFFFFFF; // write to RAM
#elif PAGE_ERASED == 0xFF
        ptr->archived = 0; // write to RAM
#else 
#error Fix me. Set PAGE_ERASED
#endif
        OverwriteZero((uint32_t)ptr,offset,sizeof(ptr->archived));
    }
    
    uint32_t RewindAndCount()
    {
        uint32_t ptr;
        ptr = Rewind();
        unread = 0;
        while(ptr != 0) {
            G8_ArchivedLogRecord rec;
            ptr = ReadRecord(ptr, &rec, true);
            if (ptr != 0 && rec.IsNew()) {
                unread++;
            }
        }
        return Rewind();
    }
    
    // ptr is updated to point to the "next" record.
    // the return value is the actual ptr of the record just read.
    uint32_t ReadNextNewRecord(uint32_t& ptr, G8_ArchivedLogRecord* pRec, bool copy)
    {
        if (ptr == 0) {
            ptr = Rewind();
        }

        do {
            uint32_t current = ptr;
            ptr = ReadRecord(ptr, pRec, copy);
            if (ptr !=0 && pRec->IsNew()) {
                return current;
            }
        } while (ptr !=0);

        return 0;
    }
    
    void ReleaseRecord(uint32_t ptr)
    {
        if (ptr >= startAddr && ptr <= endAddr ) {
            G8_ArchivedLogRecord rec;
            uint32_t nextReadPointer = ReadRecord(ptr, &rec, true);
            if (nextReadPointer !=0) {
                if (rec.IsNew()) {
                    if (unread >0) unread--;
                    MarkRead(ptr);
                }
            }
        }    
    }
    uint32_t unread;
};

#endif // _G8_FLASH_LOGFILE_H_