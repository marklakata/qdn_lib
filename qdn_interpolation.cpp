////////////////////////////////////////////////////////////////////////////
//
// data calculated from NTC data sheets from Vishay, see ntc_adc_table.xlsx in doc folder.


#include "qdn_project.h"
#include "qdn_interpolation.h"

#ifndef ADC_SIZE
#error please define ADC_SIZE in your project.h. Should be the bit width of the ADC, assuming it is right-aligned.
#endif


extern "C" 
uint16_t G8_InterpolateADC(uint16_t adc, const int16_t* table, int logTableSize) {
    uint16_t t;    
    uint8_t index;
    int logDelta;
    int delta;
    
    if (adc >= (1<<ADC_SIZE)) {
        // value too big to read from table! a bogus ADC measurement, no doubt.
        return 0;
    }
    logDelta = ADC_SIZE - logTableSize;
    index = (adc >> (logDelta));    // get the table entry from the high bits
    delta = (1L<<(logDelta));
    adc  &= (delta-1);  // get low bits to interpolate between table entries   
        
    t    = (table[index] * (delta-adc) + table[index+1] * adc) >> logDelta;

    return t;
}
