#ifndef _QDN_SERIAL_COMMS_H
#define _QDN_SERIAL_COMMS_H

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
#include "qdn_gpio.h"
#include "qdn_project.h"

#define BARE_TX_QUEUE

#ifdef BARE_TX_QUEUE
typedef struct {
    volatile uint8_t buffer[128];
    volatile uint32_t wptr;
    volatile uint32_t rptr;
    int overflows;
} BareQueue_t;
#endif

#define  SOFTWARE_RTS  1

typedef struct {
    IRQn_Type      irqn;
    uint8_t        altFunc;
} USART_Helper_t;

typedef struct {
    USART_TypeDef* uart;

    QDN_Pin*     txPin;
    QDN_Pin*     rxPin;
    QDN_Pin*     rtsPin;
    QDN_Pin*     ctsPin;

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
USART_Helper_t       QDN_USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct);


ComPortHandle_t*     QDN_SerialPortInit        ( USART_TypeDef* uart,  uint32_t ulWantedBaud, uint32_t uxQueueLength ); // configure and enable
ComPortHandle_t*     QDN_SerialPortInitEx      ( USART_TypeDef* uart,  uint32_t ulWantedBaud, uint32_t uxQueueLength ); // configure but don't enable
void                 QDN_SerialPortEnable      ( ComPortHandle_t* pxPort );
void                 QDN_SerialPutString       ( ComPortHandle_t* pxPort, const uint8_t * const pcString );
uint32_t             QDN_SerialWriteBuffer     ( ComPortHandle_t* pxPort, const uint8_t * const buffer, uint16_t usStringLength );
int32_t              QDN_SerialGetChar         ( ComPortHandle_t* pxPort, uint8_t* pcRxedChar, int32_t xBlockTime );
int32_t              QDN_SerialPutChar         ( ComPortHandle_t* pxPort, uint8_t  cOutChar );
int32_t              QDN_SerialWaitForSemaphore( ComPortHandle_t* xPort );
void                 QDN_SerialClose           ( ComPortHandle_t* xPort );
#ifdef DEBUG_UART
void                 QDN_SerialDebugWrite      (const char* data);
#else
#define              QDN_SerialDebugWrite(x)
#endif

QDN_END_EXTERN_C

#endif

