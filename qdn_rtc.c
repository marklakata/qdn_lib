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

#define _CRT_SECURE_NO_DEPRECATE

#include <stdint.h>
#include <assert.h>
#include <string.h>
#include "qdn_rtc.h"
#include <stdio.h>


//      31,29,31,30,31,30,31,31,30,31,30,31,
//      31,28,31,30,31,30,31,31,30,31,30,31,
//      31,28,31,30,31,30,31,31,30,31,30,31,
//      31,28,31,30,31,30,31,31,30,31,30,31

static const uint16_t daysInMonths[4][12] = {
  { 0,31,60,91,121,152,182,213,244,274,305,335},
  {0,31,59,90,120,151,181,212,243,273,304,334},
  {0,31,59,90,120,151,181,212,243,273,304,334},
  {0,31,59,90,120,151,181,212,243,273,304,334},
};

int  QDN_TimeStampConversionBinary2Integer(const QDN_RTC_DateTime_t* timestamp,uint32_t* integer){
  uint32_t t;
  uint32_t days;
  uint32_t month;

  uint32_t year = timestamp->century*100 + timestamp->year;
  if (year < EPOCH_YEAR) { 
    *integer = 0; // failure
    return 0;
  }
  year -= EPOCH_YEAR;

 
  days = year * 365;
  days += ((year+3) / 4); // add in leap years, not including the current year. EPOCH_YEAR => 0, EPOCH_YEAR+1 =>1, 2002=>1,2003=>1,2004=>1,2005=>2

  month = timestamp->month-1;
  days += daysInMonths[year %4][month];
  days += timestamp->day - 1;
    
  t = days*(24*3600);
  
  t +=  timestamp->hours*3600; // hours
  t +=  timestamp->minutes*60;   // mins
  t +=  timestamp->seconds;      // seconds
    
//  Time->milliseconds =(u16)(RTC->MILR&0xFFF);
  *integer = t;
  return 1;
}

int  QDN_TimeStampConversionInteger2Binary(  uint64_t integer, QDN_RTC_DateTime_t* timestamp) {
  int ret = 1;
  uint32_t yearmod;
  uint32_t day;
  uint32_t month;
  uint32_t year;
  uint32_t leapgroup;  

  // first split into days, hours, minutes, seconds
  
  timestamp->seconds = integer % 60;
  integer /= 60;
  timestamp->minutes = integer % 60;
  integer /= 60;
  timestamp->hours   = integer % 24;
  integer /= 24;
  // we are left with days since Jan 1, EPOCH_YEAR
  yearmod = integer % (366 + 365 + 365 + 365 );


#define YEARS_1 (366 )
#define YEARS_2 (366 + 365)
#define YEARS_3 (366 + 365 + 365)
  
  if (yearmod >= YEARS_3) {
    day = yearmod - YEARS_3;
    yearmod = 3;
  } else if (yearmod >= YEARS_2) {
    day = yearmod - YEARS_2;
    yearmod = 2;
  } else if (yearmod >= YEARS_1) {
    day = yearmod - YEARS_1;
    yearmod = 1;
  } else {
    day = yearmod;
    yearmod = 0;
  }

  for(month=0; (month < 11) ? (day >= daysInMonths[yearmod][month+1]) : 0 ; month++ ) ;
    
  if (month < 12) {
    timestamp->day = (uint8_t)((day - daysInMonths[yearmod][month]) + 1);
    timestamp->month = (uint8_t)(month + 1);
  } else {
    timestamp->day = 1;
    timestamp->month = 1;
    ret = 0;
  }

  assert(timestamp->day <= 31);
  
  leapgroup = integer / (365 + 365 + 365 + 366);
  year = EPOCH_YEAR + leapgroup * 4 + yearmod;
  timestamp->year = (uint8_t)( year % 100);
  year /= 100;
  timestamp->century = (uint8_t)year;
  
  // not used
  timestamp->milliseconds=0;
  return ret;
}


int QDN_TimeStampConversionBinary2LongASCII(const QDN_RTC_DateTime_t* timestamp,char* string) { // 26 bytes always written
  assert(string);
  assert(timestamp);
  if (timestamp && string) {

    sprintf(string   ,"%02d%02d-%02d-%02d, %2d:%02d:%02d.%03d", 
            timestamp->century, timestamp->year, timestamp->month, timestamp->day,
            timestamp->hours, timestamp->minutes, timestamp->seconds, timestamp->milliseconds);
    return 1;
  } else {
    return 0;
  }
}


int  QDN_TimeStampConversionBinary2ShortASCII(const QDN_RTC_DateTime_t* timestamp,char* string) { // 15 bytes always written
  assert(string);
  assert(timestamp);

  if(timestamp && string) {
    if (timestamp->century > 99 ||
        timestamp->year > 99 ||
        timestamp->month > 12 ||  
          timestamp->day > 31) {
      strcpy(string,"00000000000000");
    } else {
      sprintf(string  ,"%02d%02d%02d%02d%02d%02d%02d", 
        timestamp->century, timestamp->year, timestamp->month,timestamp->day,
        timestamp->hours, timestamp->minutes, timestamp->seconds);
      return 1;
    }
  }
  return 0;
}

int QDN_TimeStampConversionShortASCII2Binary(const char* string,QDN_RTC_DateTime_t* timestamp) {
  assert(string);
  assert(timestamp);
  if (timestamp && string) {
    timestamp->milliseconds = 0;

    int century,year,month,day,hours,minutes,seconds;
    if (sscanf(string,"%2d%2d%2d%2d%2d%2d%2d", &century,&year,&month,&day,&hours,&minutes,&seconds) !=7) {
        return 0;
    }
    timestamp->century = century;
    timestamp->year    = year;
    timestamp->month   = month;
    timestamp->day     = day;
    timestamp->hours   = hours;
    timestamp->minutes = minutes;
    timestamp->seconds = seconds;
    return 1;
  }
  return 0;
}


int QDN_TimeStampConversionShortASCII2Integer(const char* string, uint32_t* value) {
  QDN_RTC_DateTime_t timestamp;
  
  if(QDN_TimeStampConversionShortASCII2Binary(string,&timestamp)) {
    return QDN_TimeStampConversionBinary2Integer(&timestamp,value);
  }
  return 0;
}

int  QDN_TimeStampConversionInteger2ShortASCII(uint32_t value,char* string ) {
  QDN_RTC_DateTime_t timestamp;
  
  if(QDN_TimeStampConversionInteger2Binary(value,&timestamp)) {
    return QDN_TimeStampConversionBinary2ShortASCII(&timestamp,string);
  }
  return 0;
}
