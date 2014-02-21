#include "g8_stm32_fsmc_micron_psram.h"
#include "g8_serial.h"

#include "stm32f4xx_fsmc.h"

void G8_PSRAM_Init(void)
{
  FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
  FSMC_NORSRAMTimingInitTypeDef  p;
  GPIO_InitTypeDef GPIO_InitStructure; 
  RCC_ClocksTypeDef RCC_Clocks;
  
  /* Enable GPIOs clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF |
                         RCC_AHB1Periph_GPIOG, ENABLE);

  /* Enable FSMC clock */
  RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE); 
  
  
    RCC_AHB3PeriphResetCmd(RCC_AHB3Periph_FSMC, ENABLE);
    RCC_AHB3PeriphResetCmd(RCC_AHB3Periph_FSMC, DISABLE);

    
    RCC_GetClocksFreq(&RCC_Clocks);
  
/*-- GPIOs Configuration -----------------------------------------------------*/
/*
 +-------------------+--------------------+------------------+------------------+
 | PD0  <-> FSMC_D2  | PE0  <-> FSMC_NBL0 | PF0 <-> FSMC_A0  | PG0 <-> FSMC_A10 |
 | PD1  <-> FSMC_D3  | PE1  <-> FSMC_NBL1 | PF1 <-> FSMC_A1  | PG1 <-> FSMC_A11 |
 | PD4  <-> FSMC_NOE | PE2  <-> FSMC_A23  | PF2 <-> FSMC_A2  | PG2 <-> FSMC_A12 |
 | PD5  <-> FSMC_NWE | PE3  <-> FSMC_A19  | PF3 <-> FSMC_A3  | PG3 <-> FSMC_A13 |
 | PD8  <-> FSMC_D13 | PE4  <-> FSMC_A20  | PF4 <-> FSMC_A4  | PG4 <-> FSMC_A14 |
 | PD9  <-> FSMC_D14 | PE5  <-> FSMC_A21  | PF5 <-> FSMC_A5  | PG5 <-> FSMC_A15 |
 | PD10 <-> FSMC_D15 | PE6  <-> FSMC_A22  | PF12 <-> FSMC_A6 | PG9 <-> FSMC_NE2 |
 | PD11 <-> FSMC_A16 | PE7  <-> FSMC_D4   | PF13 <-> FSMC_A7 |------------------+
 | PD12 <-> FSMC_A17 | PE8  <-> FSMC_D5   | PF14 <-> FSMC_A8 |
 | PD13 <-> FSMC_A18 | PE9  <-> FSMC_D6   | PF15 <-> FSMC_A9 |
 | PD14 <-> FSMC_D0  | PE10 <-> FSMC_D7   |------------------+
 | PD15 <-> FSMC_D1  | PE11 <-> FSMC_D8   |
 +-------------------| PE12 <-> FSMC_D9   |
                     | PE13 <-> FSMC_D10  |
                     | PE14 <-> FSMC_D11  |
                     | PE15 <-> FSMC_D12  |
                     +--------------------+
*/

  /* GPIOD configuration */
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC); 
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  | GPIO_Pin_1  | GPIO_Pin_4  | GPIO_Pin_5  | 
                                GPIO_Pin_8  | GPIO_Pin_9  | GPIO_Pin_10 | GPIO_Pin_11 |
                                GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

  GPIO_Init(GPIOD, &GPIO_InitStructure);


  /* GPIOE configuration */
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource0 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource1 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource2 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource3 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource4 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource5 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource6 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource7 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource8 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource10 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource12 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource13 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource14 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource15 , GPIO_AF_FSMC);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  | GPIO_Pin_1  | GPIO_Pin_2  | GPIO_Pin_3 |  
                                GPIO_Pin_4  | GPIO_Pin_5  | GPIO_Pin_6  | GPIO_Pin_7 |
                                GPIO_Pin_8  | GPIO_Pin_9  | GPIO_Pin_10 | GPIO_Pin_11|
                                GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;

  GPIO_Init(GPIOE, &GPIO_InitStructure);


  /* GPIOF configuration */
  GPIO_PinAFConfig(GPIOF, GPIO_PinSource0 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOF, GPIO_PinSource1 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOF, GPIO_PinSource2 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOF, GPIO_PinSource3 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOF, GPIO_PinSource4 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOF, GPIO_PinSource5 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOF, GPIO_PinSource12 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOF, GPIO_PinSource13 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOF, GPIO_PinSource14 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOF, GPIO_PinSource15 , GPIO_AF_FSMC);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  | GPIO_Pin_1  | GPIO_Pin_2  | GPIO_Pin_3  | 
                                GPIO_Pin_4  | GPIO_Pin_5  | GPIO_Pin_12 | GPIO_Pin_13 |
                                GPIO_Pin_14 | GPIO_Pin_15;      

  GPIO_Init(GPIOF, &GPIO_InitStructure);


  /* GPIOG configuration */
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource0 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource1 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource2 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource3 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource4 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource5 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource9 , GPIO_AF_FSMC);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  | GPIO_Pin_1  | GPIO_Pin_2  | GPIO_Pin_3 | 
                                GPIO_Pin_4  | GPIO_Pin_5  |GPIO_Pin_9;      

  GPIO_Init(GPIOG, &GPIO_InitStructure);

/*-- FSMC Configuration ------------------------------------------------------*/
#if 1
  if (RCC_Clocks.HCLK_Frequency == 150000000) {
      // tests to make PSRAM more reliable? slower?
      p.FSMC_AddressSetupTime =  3; // 2 definitely does not work
      p.FSMC_AddressHoldTime = 0; // 0 was the starting value. 1 and 2 worked, but did not fix the hard fault issue. I think this is a "don't care" for async access.
      p.FSMC_DataSetupTime = 9; //4 does not work at all (100% failure). 5 is poor (~50% success). 6,7 works (0% failure).
                                // 9 works, and fixes the hard fault! 11 causes the code to vector to __exit, with no call stack. This value can go from 1 to 255. I think it needs to be at least 4.
      p.FSMC_BusTurnAroundDuration = 2; // for non-muxed memory, is "don't care"
      p.FSMC_CLKDivision = 1; // for async, this is "don't care".  For sync access, the division is +1 this number , ie. (1-15) => (2-16). 0 is not allowed.
      p.FSMC_DataLatency = 1; // for PSRAM, this is "don't care"
      p.FSMC_AccessMode = FSMC_AccessMode_A; // since the EXTMOD =0, this is "don't care"
  } else {
      G8_SerialDebugWrite("HCLK frequency not supported.");
      // note that this code could support other frequencies, but it just is done yet.
      while(1);
  }
  
  double tRC = (p.FSMC_AddressSetupTime + p.FSMC_DataSetupTime)*1.0e9 / RCC_Clocks.HCLK_Frequency;
  if (tRC < 70.0 /* ns */) {
      G8_SerialDebugWrite("FSMC timing for Micron MT45... memory is too fast. 70 ns tRC");
      while(1);
  }
  
  //// WAIT SIGNAL 
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_FSMC); /// MTL. hook up WAIT signal
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /// CRE signal
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOG,GPIO_Pin_6); // deassert

  FSMC_NORSRAMStructInit(&FSMC_NORSRAMInitStructure);
  
  FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM2;
  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
  FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_PSRAM;
  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
  FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;  
//  FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Enable;  
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
  FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
  FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
//  FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Enable;
  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;

#else
  p.FSMC_AddressSetupTime = 3;
  p.FSMC_AddressHoldTime = 0;
  p.FSMC_DataSetupTime = 6;
  p.FSMC_BusTurnAroundDuration = 1;
  p.FSMC_CLKDivision = 0;
  p.FSMC_DataLatency = 0;
  p.FSMC_AccessMode = FSMC_AccessMode_A;

  FSMC_NORSRAMStructInit(&FSMC_NORSRAMInitStructure);
  
  FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM2;
  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
  FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_PSRAM;
  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
  FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;  
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
  FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
  FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;

#endif
  

  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 

  /*!< Enable FSMC Bank1_SRAM2 Bank */
  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM2, ENABLE); 

#if 0
  // try to to read out BCR, etc registers
  {
      static struct {
          uint16_t RCR;
          uint16_t DIDR;
          uint16_t BCR;
      } micron;

      GPIO_SetBits(GPIOG,GPIO_Pin_6); // assert CRE
      
#define MICRON_RCR__PageModeEnable        0x80
#define MICRON_RCR__DeepPowerDownDisable  0x10
#define MICRON_RCR__RefreshMask           0x07
#define MICRON_RCR__RefreshFullArray      0x00
      
      micron.RCR  = *(uint16_t*)(0x64000000 | (0 << (18+1))); // was 0x0010
      
#define MICRON_DIDR__RowLength                 0x8000 // 0 = 128 words, 1 = 256 words
#define MICRON_DIDR__DeviceVersionMask         0x7800 // 
#define MICRON_DIDR__DeviceDensityMask         0x0700 // 3 = 128MBit
#define MICRON_DIDR__CellularRAMGenerationMask 0x00D0 // 2 = CellularRAM1.5
#define MICRON_DIDR__VendorIDMask              0x001F // 3 = Micron
      
      micron.DIDR = *(uint16_t*)(0x64000000 | (1 << (18+1))); // was 0x1B43

#define MICRON_BCR__OperationMode               0x8000 // 0 = sync, 1= async
#define MICRON_BCR__InitialAccessLatency        0x4000 // 0 = variable, 1=fixed
#define MICRON_BCR__LatencyCounterMask          0x3800 // 0 = Code8, 1 = Code1, ... 7 = Code7 Hmmm. Default is clamed to  be 3, but I see 2
#define MICRON_BCR__WaitPolarity                0x0400 // 0 = active low, 1 = active high
#define MICRON_BCR__WaitConfiguration           0x0100 // 0 = asserted during delay, 1 = asserted one data cycle before delay
#define MICRON_BCR__DriveStengthMask            0x0030 // 0 = full, 1=half, 2=quarter,3=reserved      
#define MICRON_BCR__BurstWrap                   0x0008 // 0 = wraps, 1= no wrap
#define MICRON_BCR__BurstLengthMask             0x0003 //  1 = 4 words, 2 = 8 words, 3 = 16 words, 4 = 32 words, 7 continuous,      
      micron.BCR  = *(uint16_t*)(0x64000000 | (2 << (18+1))); /// was 0x9D1F

      GPIO_ResetBits(GPIOG,GPIO_Pin_6); // deassert CRE
  }

#endif
}

/**
  * @brief  Writes a Half-word buffer to the FSMC SRAM memory.
  * @param  pBuffer : pointer to buffer.
  * @param  WriteAddr : SRAM memory internal address from which the data will be
  *         written.
  * @param  NumHalfwordToWrite : number of half-words to write.
  * @retval None
  */

void G8_PSRAM_WriteBuffer(uint16_t* pBuffer, uint32_t WriteAddr, uint32_t NumHalfwordToWrite)
{
  for (; NumHalfwordToWrite != 0; NumHalfwordToWrite--) /* while there is data to write */
  {
    /* Transfer data to the memory */
    *(uint16_t *) (Bank1_SRAM2_ADDR + WriteAddr) = *pBuffer++;

    /* Increment the address*/
    WriteAddr += 2;
  }
}

/**
  * @brief  Reads a block of data from the FSMC SRAM memory.
  * @param  pBuffer : pointer to the buffer that receives the data read from the
  *         SRAM memory.
  * @param  ReadAddr : SRAM memory internal address to read from.
  * @param  NumHalfwordToRead : number of half-words to read.
  * @retval None
  */
void G8_PSRAM_ReadBuffer(uint16_t* pBuffer, uint32_t ReadAddr, uint32_t NumHalfwordToRead)
{
    // this could be improved to do a DMA access and burst mode and go really fast, but 
    // it is not implemented.
  for (; NumHalfwordToRead != 0; NumHalfwordToRead--) /* while there is data to read */
  {
    /* Read a half-word from the memory */
    *pBuffer++ = *(__IO uint16_t*) (Bank1_SRAM2_ADDR + ReadAddr);

    /* Increment the address*/
    ReadAddr += 2;
  }
}


// simple linear test of memory
int G8_PSRAM_MemoryTest1(ProgressCallback_t func) {
    uint32_t start = 0;
    uint32_t end = 0x800000;
    
    int failures=0;
    
    while(start < end) {
#define NUM_PSRAM_TEST 1024    
        static uint16_t buffer[NUM_PSRAM_TEST];
        for(int i=0;i<NUM_PSRAM_TEST;i++) buffer[i] = i;
        G8_PSRAM_WriteBuffer(buffer,start,NUM_PSRAM_TEST);
        for(int i=0;i<NUM_PSRAM_TEST;i++) buffer[i] = 0;
        G8_PSRAM_ReadBuffer(buffer,start,NUM_PSRAM_TEST);
        for(int i=0;i<NUM_PSRAM_TEST;i++) if (buffer[i] != i) failures++;
        
        start += NUM_PSRAM_TEST;
        
        func();
    }
    
    return failures;
}

// faster test of memory using pointers instead of function calls
int G8_PSRAM_MemoryTest2(ProgressCallback_t func) {
    uint32_t* start = (uint32_t*)0x64000000;
    uint32_t* end   = (uint32_t*)0x65000000;
    
    int failures=0;

    int i = 0;
    uint32_t* ptr = start;
    uint32_t* end1 = start + 0x10000;
    while(ptr < end) {
        while(ptr < end1) {
            *(ptr++) = i++;
        }
        end1 += 0x10000;
        func();
    }
    i=0;
    ptr = start;
    end1 = start + 0x10000;
    while(ptr < end) {
        while(ptr < end1) {
            if (*(ptr++) != i++) failures++;
        }
        end1 += 0x10000;
        func();
    }    
    
    return failures;
}

// more involved memory test that rights pseudo-random data
// and reads the data in a pseudo-random order

int G8_PSRAM_MemoryTest3(ProgressCallback_t func) {
    uint32_t* start = (uint32_t*)0x64000000;
    uint32_t* end   = (uint32_t*)0x65000000;
    
    int failures=0;

    uint32_t* ptr = start;
    uint32_t* end1 = start + 0x20000;
    while(ptr < end) {
        while(ptr < end1) {
            uint32_t rand  = (1103515245 * (uint32_t)ptr) + 12345;
            *(ptr++) = rand;
        }
        end1 += 0x10000;
        func();
    }

    ptr = start;
    uint32_t istop = 0 + 0x100000/4;
    for(int i=0;i<0x1000000/4;) {
        for(;i<istop;i++) {
//random rewrite
            uint32_t* randAddrW = start + ((i * 1103515245) & 0x003FFFFF);
            uint32_t randW  = (1103515245 * (uint32_t)randAddrW) + 12345;
            *(randAddrW) = randW;
            
            uint32_t* randAddrR =  start + ((i * 22695477) & 0x003FFFFF);
            uint32_t rand  = (1103515245 * (uint32_t)randAddrR) + 12345;
            if (*randAddrR != rand) failures++;
        }
        istop += 0x100000/4;
        func();
    }    
    
    return failures;
}
