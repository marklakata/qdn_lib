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

#ifndef __QDN_ONEWIRE_H
#define __QDN_ONEWIRE_H

#include <stdint.h>

#include <stdint.h>
#include <stm32f10x.h>

// you can exclude onewire_search by defining that to 0
#ifndef ONEWIRE_SEARCH
#define ONEWIRE_SEARCH 1
#endif

// You can exclude CRC checks altogether by defining this to 0
#ifndef ONEWIRE_CRC
#define ONEWIRE_CRC 1
#endif

// Select the table-lookup method of computing the 8-bit CRC
// by setting this to 1.  The lookup table enlarges code size by
// about 250 bytes.  It does NOT consume RAM (but did in very
// old versions of OneWire).  If you disable this, a slower
// but very compact algorithm is used.
#ifndef ONEWIRE_CRC8_TABLE
#define ONEWIRE_CRC8_TABLE 1
#endif

// You can allow 16-bit CRC checks by defining this to 1
// (Note that ONEWIRE_CRC must also be 1.)
#ifndef ONEWIRE_CRC16
#define ONEWIRE_CRC16 1
#endif

class QDN_OneWire
{
private:
  GPIO_TypeDef*         m_Port;
  __IO uint32_t*        m_Register;
  uint32_t              m_RegMask;
  uint32_t              m_InputMask;
  uint32_t              m_OutputMask;
  uint16_t              m_BitMask;

  // global search state
  union
  {
      unsigned char ROM_NO[8];
      uint64_t address;
  } romAddress;
  uint8_t LastDiscrepancy;
  uint8_t LastFamilyDiscrepancy;
  bool    LastDeviceFlag;

  uint8_t numDevices;
public:
    QDN_OneWire(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
    void Init();

    void Input();
    void Output();
    uint8_t ReadPin();
    void WriteHigh();
    void WriteLow();
    uint8_t Reset();


    void Reset_search();
    bool Search(uint64_t& address);
    void Select(const uint64_t& address);
    void Skip();

    void Write_bit( uint8_t v);
    void Write(uint8_t v);
    void Write_bytes(const uint8_t *buf, uint16_t count);
    uint8_t Read_bit();
    uint8_t Read();
    void Read_bytes(uint8_t *buf, uint16_t count);

    void Depower();

#ifdef ONEWIRE_CRC
    static uint8_t Crc8( const uint8_t *addr, uint8_t len);
#ifdef ONEWIRE_CRC16
    static uint8_t Check_crc16(const uint8_t* input, uint16_t len, uint8_t* inverted_crc);
    static uint16_t Crc16(const uint8_t* input, uint16_t len);
#endif

#endif
    static void Interrupts(void);
    static void NoInterrupts(void);
};

#endif

////////////////////////////////////////////////////////////////////////////////////////End of File
