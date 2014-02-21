#include "qdn_stm32f4xx.h"
#include "qdn_rtc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_rtc.h"
#include "project.h"
#include <stdarg.h>

// not supported yet. this is a template

//#include "misc.h"
//#include "stm32f10x_bkp.h"
//#include "stm32f10x_rtc.h"

//#include <stdint.h>

//static void     QDN_RTC_Configure(u32 value);
//static void     QDN_RTC_UnlockConfiguration(void);
//static void     QDN_RTC_StandardInit(void);




#pragma section = ".intvec"

void QDN_VectorInit(void) {
    uint32_t base;
    uint32_t offset;
    uint32_t size;
    
    offset = (uint32_t)__section_begin(".intvec");
    size   = __section_size(".intvec");
      
    if (offset >= SRAM_BASE) {
        base = NVIC_VectTab_RAM;
    } else {
        base = NVIC_VectTab_FLASH;
    }
    if (size > 392) {
        // not supported!
        while(1);
    } else if (size > 256) {
        // alignment to 512 byte page
        if (offset & (0x1FF)) {
            while(1);
        }
    } else if (size > 0x80) {
        // alignment to 256 byte page
        if (offset & (0xFF)) {
            while(1);
        }
    } else {
        // alignment to 128 byte page
        if (offset & 0x7F) {
            while(1);
        }
    }
    
    SCB->VTOR = (base | offset); //     NVIC_SetVectorTable(base, offset );
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
}

#if 0

////////////////////////////////////////////////

#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"

static void QDN_ADC_EnableAndCalibrate(ADC_TypeDef* adc) {
    ADC_Cmd(adc, ENABLE);
    ADC_ResetCalibration(adc);
    while(ADC_GetResetCalibrationStatus(adc));
    ADC_StartCalibration(adc);
    while(ADC_GetCalibrationStatus(adc));
}

#endif

#if 0
#define DMA_STREAM               DMA2_Stream0
#define DMA_CHANNEL              DMA_Channel_0
#define DMA_STREAM_IRQ           DMA2_Stream0_IRQn
#define DMA_IT_TCIF              DMA_IT_TCIF0
#define DMA_STREAM_IRQHANDLER    DMA2_Stream0_IRQHandler

#endif

static void QDN_ADC_dmaToArray(DMA_Stream_TypeDef *dma, ADC_TypeDef* adc, volatile uint16_t* dst, uint32_t size) {
    switch((uint32_t)dma & 0xFFFFFF00 ) {
    case DMA1_BASE:  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE); break;
    case DMA2_BASE:  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); break;
    default: while(1) ;
    }

    uint32_t channel = 0;
    switch((uint32_t)dma & 0x000000FF ) {
    case (DMA1_Stream0_BASE&0xFF): channel = DMA_Channel_0; break;
    case (DMA1_Stream1_BASE&0xFF): channel = DMA_Channel_1; break;
    case (DMA1_Stream2_BASE&0xFF): channel = DMA_Channel_2; break;
    case (DMA1_Stream3_BASE&0xFF): channel = DMA_Channel_3; break;
    case (DMA1_Stream4_BASE&0xFF): channel = DMA_Channel_4; break;
    case (DMA1_Stream5_BASE&0xFF): channel = DMA_Channel_5; break;
    case (DMA1_Stream6_BASE&0xFF): channel = DMA_Channel_6; break;
    case (DMA1_Stream7_BASE&0xFF): channel = DMA_Channel_7; break;
    default: while(1);
    }    
    DMA_InitTypeDef DMA_InitStructure;
    DMA_StructInit(&DMA_InitStructure);
    DMA_DeInit(dma);
    while (DMA_GetCmdStatus(dma) != DISABLE) ; // this is useful for debugging. 
    
    DMA_InitStructure.DMA_Channel            = channel;  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &adc->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t) dst;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize         = size;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord; //DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular; // DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Low;
//?    DMA_InitStructure.DMA_FIFOMode       = DMA_FIFOMode_Disable;         
//?  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Enable;         
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;

    DMA_Init(dma, &DMA_InitStructure);  
    DMA_Cmd(dma, ENABLE);
    
    
      /* Check if the DMA Stream has been effectively enabled.
     The DMA Stream Enable bit is cleared immediately by hardware if there is an 
     error in the configuration parameters and the transfer is no started (ie. when
     wrong FIFO threshold is configured ...) */
    int Timeout = 10000;
    while ((DMA_GetCmdStatus(dma) != ENABLE) && (Timeout-- > 0)) ;

    /* Check if a timeout condition occurred */
    if (Timeout == 0)
    {
        while (1) ;
    }
}

    

void QDN_ADC_ConfigureDMAAndMuxMany(DMA_Stream_TypeDef *dma, volatile uint16_t* dst, ADC_TypeDef * adc,...) {
    uint8_t channels[16];
    uint32_t numChannels = 0;
    
    va_list channelList;
    va_start(channelList, adc);

    do {
        int channel = va_arg(channelList,int);
        if (channel < 0 || numChannels >= sizeof(channels)) break;
        channels[numChannels++] = channel;
    } while (1);
    if (numChannels == sizeof(channels)) {
        while(1);
    }

    QDN_ADC_dmaToArray(dma, adc, dst, numChannels);
    
    switch((uint32_t) adc) {
    case ADC1_BASE: RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); break;
    case ADC2_BASE: RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE); break;
    case ADC3_BASE: RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE); break;
    default: while (1);
    }
                                               
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler        = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_1;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    ADC_InitTypeDef ADC_InitStructure;
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Resolution           = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode         = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode   = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign            = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion      = numChannels;
    ADC_Init(adc, &ADC_InitStructure);
    
    int i;
    for(i=0;i<numChannels;i++) {
        ADC_RegularChannelConfig(adc, channels[i], i+1, ADC_SampleTime_480Cycles); // slowest setting 

        if (channels[i] == ADC_Channel_Vbat )       ADC_VBATCmd(ENABLE); 
        if (channels[i] == ADC_Channel_TempSensor ) ADC_TempSensorVrefintCmd(ENABLE); 
        if (channels[i] == ADC_Channel_Vrefint )    ADC_TempSensorVrefintCmd(ENABLE); 
    }

    ADC_DMACmd(adc, ENABLE);
    ADC_Cmd(adc, ENABLE);
    ADC_DMARequestAfterLastTransferCmd(adc, ENABLE);
    
    ADC_SoftwareStartConv(adc);

    va_end();
}


#if 0
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

#endif

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
  
    RCC->AHB1ENR  |=   (RCC_AHB1Periph_GPIOA<<num);
    RCC->AHB1RSTR &=  ~(RCC_AHB1Periph_GPIOA<<num);
    GPIO_Init(GPIOx,GPIO_InitStruct);
}



  

void QDN_RTC_Config(void)
{
  /* Enable the PWR clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* Allow access to RTC */
  PWR_BackupAccessCmd(ENABLE);

#if defined (RTC_CLOCK_SOURCE_LSI)  /* LSI used as RTC source clock*/
/* The RTC Clock may varies due to LSI frequency dispersion. */
  /* Enable the LSI OSC */ 
  RCC_LSICmd(ENABLE);

  /* Wait till LSI is ready */  
  while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
  {
  }

  /* Select the RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

  SynchPrediv = 0xFF;
  AsynchPrediv = 0x7F;

#elif defined (RTC_CLOCK_SOURCE_LSE) /* LSE used as RTC source clock */
  /* Enable the LSE OSC */
  RCC_LSEConfig(RCC_LSE_ON);

  /* Wait till LSE is ready */  
  while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
  {
  }

  /* Select the RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

//  SynchPrediv = 0xFF;
//  AsynchPrediv = 0x7F;
    
#else
  #error Please select the RTC Clock source (RTC_CLOCK_SOURCE_LSE or RTC_CLOCK_SOURCE_LSI) inside the project.h file
#endif /* RTC_CLOCK_SOURCE_LSI */

  /* Enable the RTC Clock */
  RCC_RTCCLKCmd(ENABLE);

  /* Wait for RTC APB registers synchronisation */
  RTC_WaitForSynchro();

  /* Enable The TimeStamp */
  RTC_TimeStampCmd(RTC_TimeStampEdge_Falling, ENABLE);    
  
    /* Configure the RTC data register and RTC prescaler */
  RTC_InitTypeDef RTC_InitStructure;
  RTC_InitStructure.RTC_AsynchPrediv = 0x7F;
  RTC_InitStructure.RTC_SynchPrediv  = 0xFF;
  RTC_InitStructure.RTC_HourFormat   = RTC_HourFormat_24;
  RTC_Init(&RTC_InitStructure);
}


/**

  */
int QDN_RTC_SetBinaryTimeStamp(QDN_RTC_DateTime_t* dt)
{
    int errors = 0;
    
    RTC_TimeTypeDef RTC_TimeStructure;
    RTC_DateTypeDef RTC_DateStructure;

    RTC_TimeStructInit(&RTC_TimeStructure);
    RTC_TimeStructure.RTC_H12     = RTC_H12_AM;
    RTC_TimeStructure.RTC_Hours   = dt->hours;
    RTC_TimeStructure.RTC_Minutes = dt->minutes;
    RTC_TimeStructure.RTC_Seconds = dt->seconds;

    if(RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure) == ERROR)
    {
        errors |= 1;
    } 

    RTC_DateStructInit(&RTC_DateStructure);
//    RTC_DateStructure.RTC_WeekDay = dt->weekday;
    RTC_DateStructure.RTC_Date    = dt->day;
    RTC_DateStructure.RTC_Month   = dt->month;
    RTC_DateStructure.RTC_Year    = dt->year;

    if (dt->century != 20) {
        errors |= 4;
    }


    if(RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure) == ERROR)
    {
        errors |= 2;
    } 
    
    if (errors == 0) {
        RTC_WriteBackupRegister(RTC_BKP_DR0, 0x32F2);
    }

    return errors;
}

// returns 1 on success, 0 on failure
uint8_t QDN_RTC_GetBinaryTimeStamp(QDN_RTC_DateTime_t* pTimestamp) {
    RTC_TimeTypeDef RTC_TimeStruct;
    RTC_DateTypeDef RTC_DateStruct1;
    RTC_DateTypeDef RTC_DateStruct;

    RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct1);
    RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
    RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);
    //uint32_t RTC_GetSubSecond(void);
    if (RTC_DateStruct1.RTC_Date !=    RTC_DateStruct.RTC_Date) { 
        // check for data change. If so, get time again, since it is just after midnight.
        RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
    }
    
    pTimestamp-> century      =  20; // yikes
    pTimestamp-> year         =  RTC_DateStruct.RTC_Year;
    pTimestamp-> month        =  RTC_DateStruct.RTC_Month;
    pTimestamp-> day          =  RTC_DateStruct.RTC_Date;
    pTimestamp-> hours        =  RTC_TimeStruct.RTC_Hours;
    pTimestamp-> minutes      =  RTC_TimeStruct.RTC_Minutes;
    pTimestamp-> seconds      =  RTC_TimeStruct.RTC_Seconds;
    pTimestamp-> milliseconds = 0;

    return 1; 
}


void QDN_GetShortASCIITimeStamp(char* string, int len) {
    QDN_RTC_DateTime_t timeStamp;
    QDN_RTC_GetBinaryTimeStamp(&timeStamp);
    QDN_TimeStampConversionBinary2ShortASCII(&timeStamp,string);
}



void QDN_SystemReset(void) {
#ifdef ENABLE_IWDG
    // set watchdog interval to maximum
    IWDG_ReloadCounter();

    while (IWDG_GetFlagStatus(IWDG_FLAG_PVU) == SET) {
        // wait until the Prescalar update is done
    }
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_256); // should give 26 second watchdog timeout
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);
    IWDG_ReloadCounter();
#endif
        
    NVIC_SystemReset();
}
