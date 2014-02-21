#ifndef _G8_SERIAL_COMMS_H
#define _G8_SERIAL_COMMS_H

#include "qdn_xos.h"
#include "qdn_cpu.h"
#ifdef STM32F10X_XL
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "qdn_stm32f10x.h"
#else
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "qdn_stm32f4xx.h"
#endif
#include <stdint.h>

#include "qdn_project.h"

#define BARE_TX_QUEUE

#ifdef BARE_TX_QUEUE
typedef struct {
    volatile uint8_t buffer[128];
    volatile int wptr;
    volatile int rptr;
    int overflows;
} BareQueue_t;
#endif

#define  SOFTWARE_RTS  1

typedef struct {
    USART_TypeDef* uart;
    GPIO_TypeDef* txPort;
    uint16_t      txPin_;
    GPIO_TypeDef* rxPort;
    uint16_t      rxPin_;
    GPIO_TypeDef* rtsPort;
    uint16_t      rtsPin_;
    GPIO_TypeDef* ctsPort;
    uint16_t      ctsPin_;
    int32_t       interruptPriority;
    USART_Helper_t helper;
    /* The queue used to hold received characters. */
    XOS_FixedQueue_t rxQueue;
#if SOFTWARE_RTS == 1
    uint16_t lowWater;
    uint16_t highWater;
#endif
#ifdef BARE_TX_QUEUE
    BareQueue_t txQueue;
#else
    XOS_FixedQueue_t txQueue;
#endif
} ComPortHandle_t;


QDN_EXTERN_C
ComPortHandle_t*     G8_SerialPortInit        ( USART_TypeDef* uart,  uint32_t ulWantedBaud, uint32_t uxQueueLength ); // configure and enable
ComPortHandle_t*     G8_SerialPortInitEx      ( USART_TypeDef* uart,  uint32_t ulWantedBaud, uint32_t uxQueueLength ); // configure but don't enable
void                 G8_SerialPortEnable      ( ComPortHandle_t* pxPort );
void                 G8_SerialPutString       ( ComPortHandle_t* pxPort, const uint8_t * const pcString );
uint32_t             G8_SerialWriteBuffer     ( ComPortHandle_t* pxPort, const uint8_t * const buffer, uint16_t usStringLength );
int32_t              G8_SerialGetChar         ( ComPortHandle_t* pxPort, uint8_t* pcRxedChar, int32_t xBlockTime );
int32_t              G8_SerialPutChar         ( ComPortHandle_t* pxPort, uint8_t  cOutChar );
int32_t              G8_SerialWaitForSemaphore( ComPortHandle_t* xPort );
void                 G8_SerialClose           ( ComPortHandle_t* xPort );
#ifdef DEBUG_UART
void                 G8_SerialDebugWrite      (const char* data);
#else
#define              G8_SerialDebugWrite(x)
#endif

QDN_END_EXTERN_C

#endif

