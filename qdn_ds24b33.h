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

#ifndef __QDN_DS24B33_EEPROM_H
#define __QDN_DS24B33_EEPROM_H

#include <stdint.h>

class QDN_OneWire;

class QDN_DS24B33_EEPROM
{
private:
    struct Address {
        union {
            struct {
                uint16_t TA; // this is little endian, so this should work natively against TA1
                uint8_t ES;
            };
            uint8_t u8[3];
        };
    };

    QDN_OneWire& owire;
    uint64_t romAddress;
    bool multiSlave;
    Address addr;

    bool Startup();

public:
    const static uint32_t SCRATCH_SIZE = 32;

    QDN_DS24B33_EEPROM(QDN_OneWire& owire0);
    static bool IsEEPROM(const uint64_t& romCode);
    void SelectRomCode(const uint64_t& romCode);

    // high level access
    bool WriteEEPROM(  uint16_t address, const uint8_t* data,  uint16_t len);
    bool ReadEEPROM(  uint16_t address, uint8_t* dst,  uint16_t len);

    // low level access
    bool WriteScratchPad(  uint16_t address, const uint8_t* data, uint8_t len);
    bool VerifyScratchPad( const uint8_t* data, uint8_t len);
    bool CopyScratchPad();
};

#endif

////////////////////////////////////////////////////////////////////////////////////////End of File
