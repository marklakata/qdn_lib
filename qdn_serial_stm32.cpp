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

//
// Serial port handler
// Interfaces between application level and hardware (interrupts) using
// RTOS of choice.
//
#include "qdn_xos.h"
#include "qdn_util.h"
#ifdef STM32F10X_XL
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "qdn_stm32f10x.h"
#else
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "qdn_stm32f4xx.h"
#endif
#include "qdn_stm32fxxx.h"
#include "qdn_serial.h"
#include <string.h>
/*-----------------------------------------------------------*/

// if a USART is not used, set the port,pin pair to 0,0
ComPortHandle_t ports[] = {
#include "qdn_serial_port_config.c"
    {0,     0,0} // leave this line
};
ComPortHandle_t* UART1_Port = NULL;
ComPortHandle_t* UART2_Port = NULL;
ComPortHandle_t* UART3_Port = NULL;
ComPortHandle_t* UART4_Port = NULL;
ComPortHandle_t* UART5_Port = NULL;
ComPortHandle_t* UART6_Port = NULL;

ComPortHandle_t* QDN_SerialPortInit( USART_TypeDef* uart, uint32_t ulWantedBaud, uint32_t uxQueueLength )
{
    ComPortHandle_t* port = QDN_SerialPortInitEx(uart,ulWantedBaud,uxQueueLength);
    QDN_SerialPortEnable(port);
    return port;
}

ComPortHandle_t* QDN_SerialPortInitEx( USART_TypeDef* uart, uint32_t ulWantedBaud, uint32_t uxQueueLength )
{
    ComPortHandle_t* port;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    for(port = &ports[0];port->uart != 0;port++) {
        if (port->uart == uart) {
            break;
        }
    }
    if (port->uart == 0) {
        QDN_Exception(); 
    }


#ifdef BARE_RX_QUEUE
    memset(&port->rxQueue,0,sizeof(port->rxQueue));
#else
    XOS_FixedQueueCreate(port->rxQueue ,uxQueueLength, sizeof(uint8_t));
	if( ( port->rxQueue = 0 ) )
	{
        QDN_Exception();
    }
#endif

#ifdef BARE_TX_QUEUE
    memset(&port->txQueue,0,sizeof(port->txQueue));
#else
    XOS_FixedQueueCreate(port->txQueue, uxQueueLength, sizeof(uint8_t));
    if ( port->txQueue == 0 ) {
        QDN_Exception();
    }
#endif
	
	{
        USART_InitTypeDef USART_InitStructure;

        USART_StructInit(&USART_InitStructure);
		USART_InitStructure.USART_BaudRate = ulWantedBaud;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
#if SOFTWARE_RTS == 1
        port->lowWater  = (uxQueueLength * 1)/8;
        port->highWater = (uxQueueLength * 7)/8;
#else   
        if (port->rtsPin !=0 ){
    		USART_InitStructure.USART_HardwareFlowControl |= USART_HardwareFlowControl_RTS;
        }
#endif
        if (port->ctsPin !=0 ){
    		USART_InitStructure.USART_HardwareFlowControl |= USART_HardwareFlowControl_CTS;
        }
        
		USART_InitStructure.USART_Mode = 0;
        if (port->txPin != 0) {
            USART_InitStructure.USART_Mode |= USART_Mode_Tx;
        }
        if (port->rxPin != 0) {
            USART_InitStructure.USART_Mode |= USART_Mode_Rx;
        }
        port->helper = QDN_USART_Init(port->uart,  &USART_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = port->helper.irqn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = port->interruptPriority;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; /* Not used as 4 bits are used for the pre-emption priority. */;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		QDN_NVIC_Init( &NVIC_InitStructure );
	}


	return port;
}
/*-----------------------------------------------------------*/


USART_Helper_t QDN_USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct)
{
    USART_DeInit(USARTx);
    uint32_t mask;
    uint32_t bus;
    USART_Helper_t helper;

    switch((uint32_t)USARTx) {
#ifdef STM32F10X_XL
    case USART1_BASE: mask = RCC_APB2Periph_USART1; bus = 2; helper.irqn = USART1_IRQn; break;
    case USART2_BASE: mask = RCC_APB1Periph_USART2; bus = 1; helper.irqn = USART2_IRQn; break;
    case USART3_BASE: mask = RCC_APB1Periph_USART3; bus = 1; helper.irqn = USART3_IRQn; break;
    case UART4_BASE:  mask = RCC_APB1Periph_UART4;  bus = 1; helper.irqn = UART4_IRQn ; break;
    case UART5_BASE:  mask = RCC_APB1Periph_UART5;  bus = 1; helper.irqn = UART5_IRQn ; break;
#else
    case USART1_BASE: mask = RCC_APB2Periph_USART1; bus = 2; helper.irqn = USART1_IRQn; helper.altFunc = GPIO_AF_USART1; break;
    case USART2_BASE: mask = RCC_APB1Periph_USART2; bus = 1; helper.irqn = USART2_IRQn; helper.altFunc = GPIO_AF_USART2; break;
    case USART3_BASE: mask = RCC_APB1Periph_USART3; bus = 1; helper.irqn = USART3_IRQn; helper.altFunc = GPIO_AF_USART3; break;
    case UART4_BASE:  mask = RCC_APB1Periph_UART4;  bus = 1; helper.irqn = UART4_IRQn ; helper.altFunc = GPIO_AF_UART4; break;
    case UART5_BASE:  mask = RCC_APB1Periph_UART5;  bus = 1; helper.irqn = UART5_IRQn ; helper.altFunc = GPIO_AF_UART5; break;
    case USART6_BASE: mask = RCC_APB2Periph_USART6; bus = 2; helper.irqn = USART6_IRQn; helper.altFunc = GPIO_AF_USART6; break;
#endif
    default: QDN_Exception("not supported"); return helper;
    }

    if (bus == 1) {
      RCC_APB1PeriphClockCmd(mask, ENABLE);
    } else if (bus == 2) {
      RCC_APB2PeriphClockCmd(mask, ENABLE);
    }

    USART_Init(USARTx, USART_InitStruct);
    return helper;
}



void QDN_SerialPortEnable( ComPortHandle_t* port ) {
    if (port->txPin != 0) {
#ifdef STM32F10X_XL
    	port->txPin->SetMode(GPIO_Mode_AF_PP);
#else
#error
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        port->txPin->SetAF(GPIO_PinAFConfig(port->txPort, port->txPin_, port->helper.altFunc);
#endif
    	port->txPin->Init();
    }
    
    if (port->rxPin !=0 ){
#ifdef STM32F10X_XL
    	port->rxPin->SetMode(GPIO_Mode_IPU);
#else
#error
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        port->rxPin->SetAF(port->helper.altFunc);
#endif
        port->rxPin->Init();
    }

    if (port->rtsPin !=0 ){
#ifdef STM32F10X_XL
    	port->rtsPin->SetMode(SOFTWARE_RTS ? GPIO_Mode_Out_PP : GPIO_Mode_AF_PP);
#else
#error
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Pin   = (1<<port->rtsPin_);
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
#if SOFTWARE_RTS == 0
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        port->rtsPin->SetAF(port->helper.altFunc);
#else
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
#endif
#endif
        port->rtsPin->Init();

    }

    if (port->ctsPin !=0 ){
    	port->ctsPin->SetMode( GPIO_Mode_IN_FLOATING);
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
 //       GPIO_InitStructure.GPIO_Pin = (1<<port->ctsPin_);
 //       GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
 //       QDN_GPIO_Init(port->ctsPort, &GPIO_InitStructure);
 //       GPIO_PinAFConfig(port->ctsPort, port->ctsPin_, port->helper.altFunc);
    	port->ctsPin->Init();
    }
    
    // install interrupt helpers
    switch((int)(port->uart)) {
    case USART1_BASE: UART1_Port = port; break;
    case USART2_BASE: UART2_Port = port; break;
    case USART3_BASE: UART3_Port = port; break;
    case UART4_BASE:  UART4_Port = port; break;
    case UART5_BASE:  UART5_Port = port; break;
#ifndef STM32F10X_XL
    case USART6_BASE: UART6_Port = port; break;
#endif
    }
    
    USART_Cmd(port->uart, ENABLE);

    USART_ITConfig( port->uart, USART_IT_RXNE, ENABLE );
}

int32_t QDN_SerialGetChar( ComPortHandle_t* pxPort, uint8_t* pcRxedChar, int32_t xBlockTime )
{
	/* Get the next character from the buffer.  Return false if no characters
	are available, or arrive before xBlockTime expires. */
#ifdef BARE_RX_QUEUE
	uint32_t t0 = XOS_GetTime32();
	while( XOS_MillisecondElapsedU32(t0) < (uint32_t)xBlockTime)
	{
		volatile uint32_t nextPtr = pxPort->rxQueue.rptr;
		nextPtr++;
		if (nextPtr >= sizeof(pxPort->rxQueue.buffer)) nextPtr = 0;

		if (pxPort->rxQueue.wptr != pxPort->rxQueue.rptr) {
			int32_t c = pxPort->rxQueue.buffer[pxPort->rxQueue.rptr];
			pxPort->rxQueue.rptr = nextPtr;

#if SOFTWARE_RTS == 1
			if(pxPort->rtsPin != NULL) {
				// Handle hardware flow control here. If we've emptied the queue enough to drop
				// below the low water mark, we deassert RTS and let the peer resume transmission
				int32_t bytesPending = pxPort->rxQueue.wptr - pxPort->rxQueue.rptr;
				if (bytesPending <=0) bytesPending += sizeof(pxPort->rxQueue.buffer);
				if (bytesPending <  pxPort->lowWater) {
				// The user space code only cleans this bit. The ISR sets it.
	//                pxPort->rtsPort->BSRRH = (1<<pxPort->rtsPin_);
					pxPort->rtsPin->Deassert();
				}
			}
#endif

			return c;
		}
	}
	return -1;

#else
	if( XOS_FixedQueueReceiveTimed( pxPort->rxQueue, pcRxedChar, xBlockTime )  == XOS_TIMEOUT_RESULT)
	{
		return -1;
	}
	else
	{
#if SOFTWARE_RTS == 1
        if(pxPort->rtsPin != NULL) {
            if (uxQueueMessagesWaiting(pxPort->rxQueue) <  pxPort->lowWater) {
            // The user space code only cleans this bit. The ISR sets it.
//                pxPort->rtsPort->BSRRH = (1<<pxPort->rtsPin_);
            	pxPort->rtsPin->Deassert();
            }
        }
#endif
		return *pcRxedChar;
	}
#endif
}
/*-----------------------------------------------------------*/

void QDN_SerialPutString( ComPortHandle_t* pxPort, const uint8_t * const pcString )
{
    const uint8_t *pxNext = pcString;
	while( *pxNext )
	{
		QDN_SerialPutChar( pxPort, *pxNext );
		pxNext++;
	}
}

uint32_t QDN_SerialWriteBuffer( ComPortHandle_t* pxPort, const uint8_t * const buffer, uint16_t usStringLength )
{
    uint32_t sent = 0;
	for(int i=0;i<usStringLength;i++)
	{
		if (QDN_SerialPutChar( pxPort, buffer[i]) >=0) {
            sent ++;
        }
	}
    return sent;
}

/*-----------------------------------------------------------*/

int32_t QDN_SerialPutChar( ComPortHandle_t* pxPort, uint8_t cOutChar )
{
#ifdef BARE_TX_QUEUE
    uint32_t next = pxPort->txQueue.wptr + 1;
    if (next >=  sizeof(pxPort->txQueue.buffer)) {
        next = 0;
    }
#if 1
    while (next ==  pxPort->txQueue.rptr) XOS_DelayMs(2);
#endif
    if (next !=  pxPort->txQueue.rptr) {
         pxPort->txQueue.buffer[ pxPort->txQueue.wptr] = cOutChar;
         pxPort->txQueue.wptr = next;
         pxPort->uart->CR1 |= USART_FLAG_TXE;
    } else {
         pxPort->txQueue.overflows++;
    }
    return 0;
#else
    int32_t xReturn;

	if( XOS_FixedQueueSend(pxPort->txQueue, &cOutChar ) == XOS_TIMEOUT_RESULT )
	{
		xReturn = -1;
	}
	else
	{
		xReturn = cOutChar;
		USART_ITConfig( pxPort->uart, USART_IT_TXE, ENABLE );
	}

	return xReturn;
#endif
}
/*-----------------------------------------------------------*/

void QDN_SerialClose(  ComPortHandle_t* port )
{
	if (port != NULL) {
		USART_ITConfig( port->uart, USART_IT_TXE, DISABLE );
        USART_ITConfig( port->uart, USART_IT_RXNE, DISABLE );
        USART_Cmd(port->uart, DISABLE);


        if (port->txPin  != 0) port->txPin->HiZ();
        if (port->rxPin  != 0) port->rxPin->HiZ();
        if (port->rtsPin != 0) port->rtsPin->HiZ();
        if (port->ctsPin != 0) port->ctsPin->HiZ();
    
        // uninstall interrupt helpers
        switch((int)port->uart) {
        case USART1_BASE: UART1_Port = NULL; break;
        case USART2_BASE: UART2_Port = NULL; break;
        case USART3_BASE: UART3_Port = NULL; break;
        case UART4_BASE:  UART4_Port = NULL; break;
        case UART5_BASE:  UART5_Port = NULL; break;
#ifndef STM32F10X_XL
        case USART6_BASE: UART6_Port = NULL; break;
#endif
        }
    }
}
/*-----------------------------------------------------------*/

static void USARTx_IRQHandler(ComPortHandle_t* port)
{
    if (port == NULL) {
        static int portFailure = 0;
        portFailure++;
        return;
    }
#ifdef FREERTOS
    signed long xHigherPriorityTaskWoken = 0; // only used by FREERTOS
#endif

    uint8_t cChar;

#ifdef BARE_TX_QUEUE
    if (port->uart->SR & USART_FLAG_TXE) {
        if (port->txQueue.wptr != port->txQueue.rptr) {
            port->uart->DR = port->txQueue.buffer[port->txQueue.rptr];
            uint32_t next = port->txQueue.rptr + 1;
            if (next >=  sizeof(port->txQueue.buffer)) {
                next = 0;
            }
            port->txQueue.rptr = next;
        } else {
             port->uart->CR1 &= ~USART_FLAG_TXE;
        }
    }
#else
	if( USART_GetITStatus( port->uart, USART_IT_TXE ) == SET )
	{
		if( XOS_FixedQueueReceiveFromISR( port->txQueue, &cChar, &xHigherPriorityTaskWoken ) == XOS_TIMEOUT_RESULT )
		{
			USART_ITConfig( port->uart, USART_IT_TXE, DISABLE );		
		}
		else
		{
#pragma diag_suppress=PE549
			USART_SendData( port->uart, cChar );
#pragma diag_default=PE549
		}		
	}
#endif
	
#ifdef BARE_RX_QUEUE
    if (port->uart->SR & USART_FLAG_RXNE) {
		cChar = port->uart->DR;
        uint32_t next = port->rxQueue.wptr + 1;
        if (next >=  sizeof(port->rxQueue.buffer)) {
            next = 0;
        }
        if (next != port->rxQueue.rptr) {
            port->rxQueue.buffer[port->txQueue.wptr] = cChar;
            port->rxQueue.wptr = next;
        }
#if SOFTWARE_RTS == 1
        if(port->rtsPin != NULL) {
			int32_t bytesPending = port->rxQueue.wptr - port->rxQueue.rptr;
			if (bytesPending <=0) bytesPending += sizeof(port->rxQueue.buffer);

            if(bytesPending >  port->highWater ) {
                // the ISR only sets this bit. The user space code has to clear it.
                //port->rtsPort->BSRRL = (1<<port->rtsPin_);
            	port->rtsPin->Assert();
            }
        }
#endif
    }
#else
    if (port->uart->SR & USART_FLAG_RXNE)
	{
		cChar = port->uart->DR;
		XOS_FixedQueueSendFromISREx( port->rxQueue, &cChar, &xHigherPriorityTaskWoken );
        
#if SOFTWARE_RTS == 1
        if(port->rtsPin != NULL) {
            if(uxQueueMessagesWaitingFromISR(port->rxQueue) >  port->highWater ) {
                // the ISR only sets this bit. The user space code has to clear it.
                //port->rtsPort->BSRRL = (1<<port->rtsPin_);
            	port->rtsPin->Assert();
            }
        }
#endif
	}	
#endif

#ifdef FREERTOS
	/* If sending or receiving from a queue has caused a task to unblock, and
	the unblocked task has a priority equal to or higher than the currently 
	running task (the task this ISR interrupted), then xHigherPriorityTaskWoken 
	will have automatically been set to pdTRUE within the queue send or receive 
	function.  portEND_SWITCHING_ISR() will then ensure that this ISR returns 
	directly to the higher priority unblocked task. */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
#endif
}

void USART1_IRQHandler( void );
void USART1_IRQHandler( void )
{
    USARTx_IRQHandler(UART1_Port);
}

void USART2_IRQHandler( void );
void USART2_IRQHandler( void )
{
    USARTx_IRQHandler(UART2_Port);
}

void USART3_IRQHandler( void );
void USART3_IRQHandler( void )
{
    USARTx_IRQHandler(UART3_Port);
}

void UART4_IRQHandler( void );
void UART4_IRQHandler( void )
{
    USARTx_IRQHandler(UART4_Port);
}

void UART5_IRQHandler( void );
void UART5_IRQHandler( void )
{
    USARTx_IRQHandler(UART5_Port);
}

void USART6_IRQHandler( void );
void USART6_IRQHandler( void )
{
    USARTx_IRQHandler(UART6_Port);
}


#include "qdn_project.h"

#ifdef DEBUG_UART
// this assumes that the debug UART is ready for data
void QDN_SerialDebugWrite(const char* data) {
    __disable_interrupt();
    int len = strlen(data);
    for(int i=0;i<len;i++) {
        while((DEBUG_UART->SR & USART_FLAG_TXE) == 0) ;
        DEBUG_UART->DR = data[i];
    }
}
#endif

	
