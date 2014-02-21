#ifndef _G8_RTC_H_
#define _G8_RTC_H_

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
