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

#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "qdn_stm32f10x.h"
#include "misc.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_rtc.h"
#include "qdn_rtc.h"
#include "qdn_util.h"

#include <stdint.h>

static void     QDN_RTC_Configure(u32 value);
static void     QDN_RTC_UnlockConfiguration(void);
static void     QDN_RTC_StandardInit(void);

static uint32_t GPIO_Number(GPIO_TypeDef* GPIOx) {
    uint32_t address = (uint32_t)GPIOx;
    address -= GPIOA_BASE;
    // each GPIO uses 0x400 in address space
    address >>= 10;
    return address;
}


void QDN_GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct)
{
    uint32_t num = GPIO_Number(GPIOx);
  
    RCC->APB2ENR  |=   (RCC_APB2Periph_GPIOA<<num);
    RCC->APB2RSTR &=  ~(RCC_APB2Periph_GPIOA<<num);
    GPIO_Init(GPIOx,GPIO_InitStruct);
}


  

#ifdef __GNUC__
	//#pragma CODE_SECTION(QDN_VectorInit,".intvec")
	//extern void QDN_VectorInit(void) __attribute__ ((section (".intvec")));
#elif __IAR__
	#pragma section = ".intvec"
#else
	#error "fix me"
#endif


void QDN_VectorInit(void)
{
    uint32_t base;
    uint32_t offset;
    uint32_t size;
    
#ifdef __GNUC__
    {
    	// these 2 symbols are defined in a custom linker (ld) file, near .isr_vector :
    	// if not, you'll to add PROVIDE statements.
    	extern void __isr_vector_start();
    	extern void __isr_vector_end();

    	offset = (uint32_t)  &__isr_vector_start;
    	size   = ((uint32_t) &__isr_vector_end) - offset;
    }
#elif __IAR__
    offset = (uint32_t)__section_begin(".intvec");
    size   = __section_size(".intvec");
#endif

    
    if (offset >= SRAM_BASE) {
        base = SCB_VTOR_TBLBASE;
    } else {
        base = 0;
    }
#if defined(STM32F10X_XL) || defined(STM32F10X_MD)
#define MAX_VTOR_SIZE 484
#else
#error fix this logic.
#define MAX_VTOR_SIZE 288 // size of stm32f100
#endif
    if (size > MAX_VTOR_SIZE) {
        // not supported!
        QDN_Exception("not supported");
    } else if (size > 256) {
        // alignment to 512 byte page
        if (offset & (0x1FF)) {
            QDN_Exception("not supported");
        }
    } else if (size > 0x80) {
        // alignment to 256 byte page
        if (offset & (0xFF)) {
            QDN_Exception("not supported");
        }
    } else {
        // alignment to 128 byte page
        if (offset & 0x7F) {
            QDN_Exception("not supported");
        }
    }
    
    SCB->VTOR = (base | offset); //     NVIC_SetVectorTable(base, offset );

    /// this enables 4 preemption priorities (0-3) and 4 subpriorities (0-3)
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

////////////////////////////////////////////////

#include <stdarg.h>
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"

static void QDN_ADC_EnableAndCalibrate(ADC_TypeDef* adc) {
    ADC_Cmd(adc, ENABLE);
    ADC_ResetCalibration(adc);
    while(ADC_GetResetCalibrationStatus(adc));
    ADC_StartCalibration(adc);
    while(ADC_GetCalibrationStatus(adc));
}

static void QDN_ADC_dmaToArray(DMA_Channel_TypeDef *dma, ADC_TypeDef* adc, volatile uint16_t* dst, uint32_t size) {
    DMA_InitTypeDef DMA_InitStructure;

    DMA_DeInit(dma);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &adc->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t) dst;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Low;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_InitStructure.DMA_BufferSize         = size;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord; //DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular; // DMA_Mode_Normal;

    DMA_Init(dma, &DMA_InitStructure);  
    DMA_Cmd(dma, ENABLE);
}

void QDN_ADC_ConfigureDMAAndMuxMany(DMA_Channel_TypeDef *dma, volatile uint16_t* dst, ADC_TypeDef * adc,...) {
    uint8_t channels[16];
    uint32_t numChannels = 0;
    
    va_list channelList;
    va_start(channelList, adc);

    do {
        int channel = va_arg(channelList,int);
        if (channel < 0 || numChannels >= 16) break;
        channels[numChannels++] = channel;
    } while (1);

    QDN_ADC_dmaToArray(dma, adc, dst, numChannels);
    
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode       = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel       = numChannels;
    ADC_Init(adc, &ADC_InitStructure);
    
    int i;
    for(i=0;i<numChannels;i++) {
        ADC_RegularChannelConfig(adc, channels[i], i+1, ADC_SampleTime_239Cycles5);    
    }

    ADC_DMACmd(adc, ENABLE);
    QDN_ADC_EnableAndCalibrate(adc);
    ADC_SoftwareStartConvCmd(adc, ENABLE);
    va_end(channelList);
}


#define HIGH_RTC_WORD_BACKUP_REGISTER BKP_DR2

uint64_t QDN_RTC_GetIntegerTimeStamp(void) {
    uint64_t rtcCounter = RTC_GetCounter();
    uint16_t high = BKP_ReadBackupRegister(HIGH_RTC_WORD_BACKUP_REGISTER);
    if ( RTC_GetFlagStatus(RTC_FLAG_OW)) {
        // arg. this code will be executed once every 70 years ... but it could happen.
        high++;
        RTC_ClearFlag(RTC_FLAG_OW);
        BKP_WriteBackupRegister(HIGH_RTC_WORD_BACKUP_REGISTER, high);
    }
    rtcCounter |= ((uint64_t)high) << 32;
    return rtcCounter;
}

void QDN_RTC_SetTime(uint64_t value64) {
    uint32_t lower = (uint32_t)(value64 & 0xFFFFFFFF);
    uint16_t high = (uint16_t)((value64 >> 32) & 0xFFFF);
    QDN_RTC_UnlockConfiguration();
    QDN_RTC_Configure(lower);
    RTC_ClearFlag(RTC_FLAG_OW);
    BKP_WriteBackupRegister(HIGH_RTC_WORD_BACKUP_REGISTER, high);
    QDN_RTC_StandardInit();
}


#include "stm32f10x_bkp.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_pwr.h"

#define LSERDY_ASSERTED() ((RCC->BDCR & RCC_BDCR_LSERDY) != 0)

void QDN_RTC_Configure(u32 value) {
    if (BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5)
    {
        /* Backup data register value is not correct or not yet programmed (when
        the first time the program is executed) */
        
        /* Enable PWR and BKP clocks */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
        
        /* Allow access to BKP Domain */
        PWR_BackupAccessCmd(ENABLE);
        
        /* Reset Backup Domain */
        BKP_DeInit();
        
        /* Enable LSE */
        RCC_LSEConfig(RCC_LSE_ON);
        /* Wait till LSE is ready */
        while (! LSERDY_ASSERTED()) {
            // this may need a watchdog feeding here...
        }        
        /* Select LSE as RTC Clock Source */
        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
        
        /* Enable RTC Clock */
        RCC_RTCCLKCmd(ENABLE);
        
        /* Wait for RTC registers synchronization */
        RTC_WaitForSynchro();
        
        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
        
        /* Enable the RTC Second */
        RTC_ITConfig(RTC_IT_SEC, ENABLE);
        
        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
        
        /* Set RTC prescaler: set RTC period to 1sec */
        RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
        
        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
    
        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
        /* Change the current time */
        RTC_SetCounter(value);
        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
        
        BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);
    }
}

void QDN_RTC_UnlockConfiguration(void) {
    /* Enable PWR and BKP clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    
    /* Allow access to BKP Domain */
    PWR_BackupAccessCmd(ENABLE);
    
    /* Reset Backup Domain */
    BKP_DeInit();
    
    BKP_WriteBackupRegister(BKP_DR1, 0xFFFF);
}

void QDN_RTC_StandardInit(void) {
    if (BKP_ReadBackupRegister(BKP_DR1) == 0xA5A5) {
#if 0        
        /* Check if the Power On Reset flag is set */
        if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
        {
            printf("\r\n\n Power On Reset occurred....");
        }
        /* Check if the Pin Reset flag is set */
        else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
        {
            printf("\r\n\n External Reset occurred....");
        }
#endif
        
        /* Wait for RTC registers synchronization */
        RTC_WaitForSynchro();
        
        /* Enable the RTC Second */
        //    RTC_ITConfig(RTC_IT_SEC, ENABLE);
        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
    }
}



// returns 1 on success, 0 on failure
uint8_t QDN_RTC_GetBinaryTimeStamp(struct QDN_RTC_DateTime_s* pTimestamp) {
    uint64_t rtcCounter = QDN_RTC_GetIntegerTimeStamp();
    return QDN_TimeStampConversionInteger2Binary(rtcCounter,pTimestamp);
}


void QDN_GetShortASCIITimeStamp(char* string, int len) {
    QDN_RTC_DateTime_t timeStamp;
    QDN_RTC_GetBinaryTimeStamp(&timeStamp);
    QDN_TimeStampConversionBinary2ShortASCII(&timeStamp,string);
}
