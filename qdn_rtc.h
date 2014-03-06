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

#ifndef _QDN_RTC_H_
#define _QDN_RTC_H_

#include <stdint.h>

#include "qdn_cpu.h"

QDN_EXTERN_C

#ifndef EPOCH_YEAR
#define EPOCH_YEAR 2000
#endif

typedef struct QDN_RTC_DateTime_s {
  uint8_t  century;
  uint8_t  year;
  uint8_t  month;
  uint8_t  day;
  uint8_t  hours;
  uint8_t  minutes;
  uint8_t  seconds;
  uint16_t milliseconds;
} QDN_RTC_DateTime_t;

int  QDN_TimeStampConversionBinary2Integer   (const QDN_RTC_DateTime_t* timestamp,uint32_t* integer);
int  QDN_TimeStampConversionBinary2LongASCII (const QDN_RTC_DateTime_t* timestamp,char* string); // 26 bytes always written
int  QDN_TimeStampConversionBinary2ShortASCII(const QDN_RTC_DateTime_t* timestamp,char* string); // 15 bytes always written

int  QDN_TimeStampConversionInteger2Binary    (uint64_t integer,   QDN_RTC_DateTime_t* timestamp);
int  QDN_TimeStampConversionShortASCII2Binary (const char* string, QDN_RTC_DateTime_t* timestamp);
int  QDN_TimeStampConversionShortASCII2Integer(const char* string, uint32_t* value);
int  QDN_TimeStampConversionInteger2ShortASCII(uint32_t value,     char* string );

    
QDN_END_EXTERN_C

#endif
