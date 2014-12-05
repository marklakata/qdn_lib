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

#include <qdn_onewire.h>
#include <qdn_ds24b33.h>
#include <qdn_cpu.h>
#include <qdn_xos.h>
#include <stdint.h>
#include <cstdlib>
#include <cstring>

#define DS24B33WRITESCRATCHPAD  0x0F
#define DS24B33READSCRATCHPAD   0xAA
#define DS24B33COPYSCRATCHPAD   0x55
#define DS24B33READMEMORY       0xF0

#define FAMILY_CODE 0x23

#define SCRATCH_SIZE 32
#define MAX_READ_BUF 512




QDN_DS24B33_EEPROM::QDN_DS24B33_EEPROM(QDN_OneWire& owire0)
    :owire(owire0)
    , multiSlave(false)
{
}

void QDN_DS24B33_EEPROM::SelectRomCode(const uint64_t& romCode0)
{
    romAddress = romCode0;
    multiSlave = true;
}

bool QDN_DS24B33_EEPROM::IsEEPROM(const uint64_t& romCode)
{
    const uint8_t* pRomCode = reinterpret_cast<const uint8_t*>(&romCode);
    if (pRomCode[0] == FAMILY_CODE)
    {
        uint8_t crc = QDN_OneWire::Crc8(pRomCode,7);
        if (crc == pRomCode[7]) return true;
    }
    return false;
}

bool QDN_DS24B33_EEPROM::Startup()
{
    bool present = owire.Reset();
    if (present)
    {
        if (multiSlave)
        {
            owire.Select(romAddress);
        } else {
            owire.Skip();
        }
    }
    return present;
}

bool QDN_DS24B33_EEPROM::WriteEEPROM(  uint16_t address, const uint8_t* data, uint16_t len)
{
    uint16_t toGo = len;
    bool r = true;
    while (toGo > 0)
    {
        uint16_t thisTime = toGo;
        if (thisTime > SCRATCH_SIZE) thisTime = SCRATCH_SIZE;

        if (WriteScratchPad( address,data,thisTime))
        {
            if (VerifyScratchPad( data, thisTime))
            {
                r &= CopyScratchPad();
            } else {
                r = false;
            }
        } else {
            r = false;
        }
        data += thisTime;
        address += thisTime;
        toGo -= thisTime;
    }
    return r;
}

bool QDN_DS24B33_EEPROM::ReadEEPROM(  uint16_t address,  uint8_t* dst,  uint16_t len)
{
    uint16_t toGo = len;
    while (toGo > 0)
    {
        uint16_t thisTime = toGo;
        if (thisTime > MAX_READ_BUF) thisTime = MAX_READ_BUF;

        if (!Startup()) return false;
        owire.Write(DS24B33READMEMORY);                                              // Read Memory
        Address addr;
        addr.TA = address;
        owire.WriteBytes(addr.u8,2);

        owire.ReadBytes(dst,thisTime);

        dst += thisTime;
        address += thisTime;
        toGo -= thisTime;
    }
    return true;
}

// scratch pad is 32 bytes
bool QDN_DS24B33_EEPROM::WriteScratchPad( uint16_t address, const uint8_t* src,  uint8_t len)
{
    if (!Startup()) return false;
    owire.Write(DS24B33WRITESCRATCHPAD);
    Address addr;
    addr.TA = address;
    owire.WriteBytes(addr.u8,2);
    owire.WriteBytes(src,len);
    return true;
}

bool QDN_DS24B33_EEPROM::VerifyScratchPad( const uint8_t* src,  uint8_t len)
{
    uint8_t scratch[SCRATCH_SIZE];
    if (!Startup()) return false;
    owire.Write(DS24B33READSCRATCHPAD);
    owire.ReadBytes(addr.u8,3);
    bool PFFlag = !!(addr.ES & 0x20);
    if (PFFlag) return false;

    owire.ReadBytes(scratch,len);
    bool r= memcmp(scratch,src,len) == 0;
    return r;
}

bool QDN_DS24B33_EEPROM::CopyScratchPad()
{
    for(int attempt=1;attempt <= 2;attempt++)
    {
        if (Startup())
        {
            owire.Write(DS24B33COPYSCRATCHPAD);
            owire.WriteBytes(addr.u8,3);

            XOS_DelayMs(7);

            // look for toggling of bit
            bool s = owire.ReadBit();
            bool t = owire.ReadBit();
            if (s != t)
            {
                owire.Reset();
                return true;
            }
        } else {
            XOS_DelayMs(7);
        }
    }
    return false;
}

