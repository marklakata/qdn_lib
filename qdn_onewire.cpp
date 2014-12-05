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

#include "qdn_onewire.h"

// inspired by
// https://my.st.com/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/DispForm.aspx?ID=27035&Source=/public/STe2ecommunities/mcu/Tags.aspx?tags=Delay


#include "stm32f10x.h"
#include "qdn_xos.h"
#include "qdn_util.h"

#define delayMicroseconds(x) XOS_Delay100Ns ((x)*10)


QDN_OneWire::QDN_OneWire(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
  : m_Port(GPIOx)
  , m_BitMask( GPIO_Pin)
  , numDevices(0)
{

}

void QDN_OneWire::Init()
{
  uint32_t PeriphClock;

  if (m_Port == GPIOA)
  {
    PeriphClock = RCC_APB2Periph_GPIOA;
  }
  else if (m_Port == GPIOB)
  {
    PeriphClock = RCC_APB2Periph_GPIOB;
  }
  else if (m_Port == GPIOC)
  {
    PeriphClock = RCC_APB2Periph_GPIOC;
  }
  else if (m_Port == GPIOD)
  {
    PeriphClock = RCC_APB2Periph_GPIOD;
  }
  else if (m_Port == GPIOE)
  {
    PeriphClock = RCC_APB2Periph_GPIOE;
  }
  else if (m_Port == GPIOF)
  {
    PeriphClock = RCC_APB2Periph_GPIOF;
  }
  else if (m_Port == GPIOG)
  {
      PeriphClock = RCC_APB2Periph_GPIOG;
  }
  else
  {
      QDN_Exception();
      return;
  }


  RCC_APB2PeriphClockCmd(PeriphClock, ENABLE);

  GPIO_InitTypeDef  GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin   = m_BitMask;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

  GPIO_Init(m_Port, &GPIO_InitStructure);

  uint32_t pinpos = 0x00, pos = 0x00, currentpin = 0x00;

  uint8_t RegShift = 0;

  if((m_BitMask & (uint32_t)0x00FF) > 0)
  {
    m_Register = &m_Port->CRL;

    for (pinpos = 0x00; pinpos < 0x08; pinpos++)
    {
      pos = ((uint32_t)0x01) << pinpos;
      /* Get the port pins position */
      currentpin = (uint16_t)((m_BitMask) & pos);
      if (currentpin == pos)
      {
        RegShift = (pinpos*4);
        m_RegMask = ((uint32_t)0x0F) << (pinpos*4);
        break;
      }
     }
  }
  else
  {
    m_Register = &m_Port->CRH;

    for (pinpos = 0x00; pinpos < 0x08; pinpos++)
    {
      pos = ((uint32_t)0x01) << (pinpos + 0x08);
      /* Get the port pins position */
      currentpin = (uint16_t)((m_BitMask) & pos);
      if (currentpin == pos)
      {
        RegShift = (pinpos*4);
        m_RegMask = ((uint32_t)0x0F) << (pinpos*4);
        break;
      }
    }
  }

  m_InputMask = (((GPIO_Mode_IN_FLOATING) << RegShift) & m_RegMask);
  m_OutputMask = (((uint32_t)GPIO_Mode_Out_OD|(uint32_t)GPIO_Speed_50MHz) << RegShift) & m_RegMask;

}

void QDN_OneWire::Input()
{
  *m_Register &= ~m_RegMask;
  *m_Register |= m_InputMask;
}
void QDN_OneWire::Output()
{
  *m_Register &= ~m_RegMask;
  *m_Register |= m_OutputMask;

}

uint8_t QDN_OneWire::ReadPin()
{
  return (uint8_t)((m_Port->IDR & m_BitMask) > 0 ? 1 : 0);
}

void QDN_OneWire::WriteHigh()
{
  m_Port->BSRR = m_BitMask;
}

void QDN_OneWire::WriteLow()
{
  m_Port->BRR = m_BitMask;
}

void QDN_OneWire::NoInterrupts(void)
{
  __asm("cpsid i");
}

void QDN_OneWire::Interrupts(void)
{
  __asm("cpsie i");
}

uint8_t QDN_OneWire::Reset()
{
    uint8_t r;
    uint8_t retries = 125;

    NoInterrupts();
    Input();
    Interrupts();
    // wait until the wire is high... just in case
    do {
        if (--retries == 0) return 0;
        delayMicroseconds(2);
    } while ( !ReadPin());

    NoInterrupts();
    WriteLow();
    Output();    // drive output low
    Interrupts();
    delayMicroseconds(500);
    NoInterrupts();
    Input(); // allow it to float
    delayMicroseconds(80);
    r = !ReadPin();
    Interrupts();
    delayMicroseconds(420);
    return r;
}

void QDN_OneWire::WriteBit(uint8_t v)
{
    if (v & 1) {
        NoInterrupts();
        WriteLow();
        Output();    // drive output low
        delayMicroseconds(10);
        WriteHigh(); // drive output high
        Interrupts();
        delayMicroseconds(55);
    } else {
        NoInterrupts();
        WriteLow();
        Output();    // drive output low
        delayMicroseconds(65);
        WriteHigh(); // drive output high
        Interrupts();
        delayMicroseconds(5);
    }
}

uint8_t QDN_OneWire::ReadBit()
{
    uint8_t r;

    NoInterrupts();
    WriteLow();
    Output();
    delayMicroseconds(3);
    Input(); // let pin float, pull up will raise
    delayMicroseconds(10);
    r = ReadPin();
    Interrupts();
    delayMicroseconds(53);
    return r;
}

void QDN_OneWire::Write( uint8_t v) {
    uint8_t bitMask;

    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
        WriteBit((bitMask & v)?1:0);
    }

    NoInterrupts();
    Input();
    WriteLow();
    Interrupts();
}

void QDN_OneWire::WriteBytes(const uint8_t *buf, uint16_t count) {
  for (uint16_t i = 0 ; i < count ; i++)
    Write(buf[i]);


  NoInterrupts();
  Input();
  WriteLow();
  Interrupts();

}

uint8_t QDN_OneWire::Read()
{
    uint8_t bitMask;
    uint8_t r = 0;

    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
        if ( ReadBit()) r |= bitMask;
    }
    return r;
}

void QDN_OneWire::ReadBytes(uint8_t *buf, uint16_t count)
{
  for (uint16_t i = 0 ; i < count ; i++)
    buf[i] = Read();
}

#define OW_MATCH_ROM  0x55
#define OW_SKIP_ROM   0xCC
#define OW_SEARCH_ROM 0xF0

void QDN_OneWire::Select(const uint64_t& address)
{
    Write(OW_MATCH_ROM);
    const uint8_t* rom = reinterpret_cast<const uint8_t*>(&address);
    for(int i = 0; i < 8; i++) Write(rom[i]);
}

void QDN_OneWire::Skip()
{
    Write(OW_SKIP_ROM);
}

void QDN_OneWire::Depower()
{
    NoInterrupts();
    Input();
    Interrupts();
}


void QDN_OneWire::ResetSearch()
{
    // must be called before a series Search commands
  LastDiscrepancy = 0;
  LastDeviceFlag = false;
  LastFamilyDiscrepancy = 0;
  romAddress.address = 0;
  numDevices = 0;
}

// call Reset_search first, then
// call this function until it returns false
bool QDN_OneWire::Search(uint64_t& foundAddress)
{
   uint8_t id_bit_number;
   uint8_t last_zero, rom_byte_number;
   bool search_result;
   uint8_t id_bit, cmp_id_bit;

   unsigned char rom_byte_mask, search_direction;

   // initialize for search
   id_bit_number = 1;
   last_zero = 0;
   rom_byte_number = 0;
   rom_byte_mask = 1;
   search_result = false;

   // if the last call was not the last one
   if (!LastDeviceFlag)
   {
      // 1-Wire reset
      if (!Reset())
      {
         // reset the search
         LastDiscrepancy = 0;
         LastDeviceFlag = false;
         LastFamilyDiscrepancy = 0;
         return false;
      }

      // issue the search command
      Write(OW_SEARCH_ROM);

      // loop to do the search
      do
      {
         // read a bit and its complement
         id_bit = ReadBit();
         cmp_id_bit = ReadBit();

         // check for no devices on 1-wire
         if ((id_bit == 1) && (cmp_id_bit == 1))
            break;
         else
         {
            // all devices coupled have 0 or 1
            if (id_bit != cmp_id_bit)
               search_direction = id_bit;  // bit write value for search
            else
            {
               // if this discrepancy if before the Last Discrepancy
               // on a previous next then pick the same as last time
               if (id_bit_number < LastDiscrepancy)
                  search_direction = ((romAddress.ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
               else
                  // if equal to last pick 1, if not then pick 0
                  search_direction = (id_bit_number == LastDiscrepancy);

               // if 0 was picked then record its position in LastZero
               if (search_direction == 0)
               {
                  last_zero = id_bit_number;

                  // check for Last discrepancy in family
                  if (last_zero < 9)
                     LastFamilyDiscrepancy = last_zero;
               }
            }

            // set or clear the bit in the ROM byte rom_byte_number
            // with mask rom_byte_mask
            if (search_direction == 1)
              romAddress.ROM_NO[rom_byte_number] |= rom_byte_mask;
            else
              romAddress.ROM_NO[rom_byte_number] &= ~rom_byte_mask;

            // serial number search direction write bit
            WriteBit(search_direction);

            // increment the byte counter id_bit_number
            // and shift the mask rom_byte_mask
            id_bit_number++;
            rom_byte_mask <<= 1;

            // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
            if (rom_byte_mask == 0)
            {
                rom_byte_number++;
                rom_byte_mask = 1;
            }
         }
      }
      while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

      // if the search was successful then
      if (!(id_bit_number < 65))
      {
         // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
         LastDiscrepancy = last_zero;

         // check for last device
         if (LastDiscrepancy == 0)
            LastDeviceFlag = true;

         search_result = true;
      }
   }

   // if no device found then reset counters so next 'search' will be like a first
   if (!search_result || ! romAddress.ROM_NO[0])
   {
      LastDiscrepancy = 0;
      LastDeviceFlag = false;
      LastFamilyDiscrepancy = 0;
      search_result = false;
   }
   foundAddress = romAddress.address;
   if (search_result) numDevices++;
   return search_result;
}


#ifdef ONEWIRE_CRC
// The 1-Wire CRC scheme is described in Maxim Application Note 27:
// "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"
//

#ifdef ONEWIRE_CRC8_TABLE
// This table comes from Dallas sample code where it is freely reusable,
// though Copyright (C) 2000 Dallas Semiconductor Corporation
static const uint8_t dscrc_table[] = {
      0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
    157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
     35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
    190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
     70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
    219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
    101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
    248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
    140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
     17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
    175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
     50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
    202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
     87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
    233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
    116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53};

//
// Compute a Dallas Semiconductor 8 bit CRC. These show up in the ROM
// and the registers.  (note: this might better be done without to
// table, it would probably be smaller and certainly fast enough
// compared to all those delayMicrosecond() calls.  But I got
// confused, so I use this table from the examples.)
//
uint8_t QDN_OneWire::Crc8( const uint8_t *addr, uint8_t len)
{
    uint8_t crc = 0;

    while (len--) {
        crc = dscrc_table[(crc ^ *addr++)];
    }
    return crc;
}
#else
//
// Compute a Dallas Semiconductor 8 bit CRC directly.
// this is much slower, but much smaller, than the lookup table.
//
uint8_t QDN_OneWire::Crc8( const uint8_t *addr, uint8_t len)
{
    uint8_t crc = 0;

    while (len--) {
        uint8_t inbyte = *addr++;
        for (uint8_t i = 8; i; i--) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crc;
}
#endif

#ifdef ONEWIRE_CRC16
uint8_t QDN_OneWire::CheckCrc16(const uint8_t* input, uint16_t len, uint8_t* inverted_crc)
{
    uint16_t crc = ~Crc16(input, len);
    return (crc & 0xFF) == inverted_crc[0] && (crc >> 8) == inverted_crc[1];
}

uint16_t QDN_OneWire::Crc16(const uint8_t* input, uint16_t len)
{
    static const uint8_t oddparity[16] =
        { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };
    uint16_t crc = 0;    // Starting seed is zero.

    for (uint16_t i = 0 ; i < len ; i++) {
      // Even though we're just copying a byte from the input,
      // we'll be doing 16-bit computation with it.
      uint16_t cdata = input[i];
      cdata = (cdata ^ (crc & 0xff)) & 0xff;
      crc >>= 8;

      if (oddparity[cdata & 0x0F] ^ oddparity[cdata >> 4])
          crc ^= 0xC001;

      cdata <<= 6;
      crc ^= cdata;
      cdata <<= 1;
      crc ^= cdata;
    }
    return crc;
}
#endif // CRC16
#endif // CRC
