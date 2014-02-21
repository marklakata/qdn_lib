//
// Gener8 serial port handler
// Interfaces between application level and hardware (interrupts) using
// RTOS of choice.
//
#include "qdn_xos.h"
#ifdef STM32F10X_XL
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "qdn_stm32f10x.h"
#else
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "qdn_stm32f4xx.h"
#endif
#include "qdn_serial.h"
#include <string.h>
/*-----------------------------------------------------------*/

// if a USART is not used, set the port,pin pair to 0,0
ComPortHandle_t ports[] = {
#include "g8_serial_port_config.c"
    {0,     0,0} // leave this line
};
ComPortHandle_t* UART1_Port = NULL;
ComPortHandle_t* UART2_Port = NULL;
ComPortHandle_t* UART3_Port = NULL;
ComPortHandle_t* UART4_Port = NULL;
ComPortHandle_t* UART5_Port = NULL;
ComPortHandle_t* UART6_Port = NULL;

ComPortHandle_t* G8_SerialPortInit( USART_TypeDef* uart, uint32_t ulWantedBaud, uint32_t uxQueueLength )
{
    ComPortHandle_t* port = G8_SerialPortInitEx(uart,ulWantedBaud,uxQueueLength);
    G8_SerialPortEnable(port);
    return port;
}

ComPortHandle_t* G8_SerialPortInitEx( USART_TypeDef* uart, uint32_t ulWantedBaud, uint32_t uxQueueLength )
{
    ComPortHandle_t* port;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    for(port = &ports[0];port->uart != 0;port++) {
        if (port->uart == uart) {
            break;
        }
    }
    if (port->uart == 0) {
        while(1); 
    }


    XOS_FixedQueueCreate(port->rxQueue ,uxQueueLength, sizeof(uint8_t));
#ifdef BARE_TX_QUEUE
    memset(&port->txQueue,0,sizeof(port->txQueue));
#else
    XOS_FixedQueueCreate(port->txQueue, uxQueueLength, sizeof(uint8_t));
    if ( port->txQueue == 0 ) {
        while(1);
    }
#endif
	
	if( ( port->rxQueue != 0 ) )
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
        if (port->rtsPort !=0 ){
    		USART_InitStructure.USART_HardwareFlowControl |= USART_HardwareFlowControl_RTS;
        }
#endif
        if (port->ctsPort !=0 ){
    		USART_InitStructure.USART_HardwareFlowControl |= USART_HardwareFlowControl_CTS;
        }
        
		USART_InitStructure.USART_Mode = 0;
        if (port->txPort != 0) {
            USART_InitStructure.USART_Mode |= USART_Mode_Tx;
        }
        if (port->rxPort != 0) {
            USART_InitStructure.USART_Mode |= USART_Mode_Rx;
        }
        port->helper = G8_USART_Init(port->uart,  &USART_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = port->helper.irqn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = port->interruptPriority;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; /* Not used as 4 bits are used for the pre-emption priority. */;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		G8_NVIC_Init( &NVIC_InitStructure );
	}
	else
	{
        while(1);
	}

	return port;
}
/*-----------------------------------------------------------*/

void G8_SerialPortEnable( ComPortHandle_t* port ) {
    if (port->txPort != 0) {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Pin   = (1<<port->txPin_);
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        G8_GPIO_Init(port->txPort, &GPIO_InitStructure);
        GPIO_PinAFConfig(port->txPort, port->txPin_, port->helper.altFunc);
    }
    
    if (port->rxPort !=0 ){
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_StructInit(&GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Pin = (1<<port->rxPin_);
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        G8_GPIO_Init(port->rxPort, &GPIO_InitStructure);
        GPIO_PinAFConfig(port->rxPort, port->rxPin_, port->helper.altFunc);
    }

    if (port->rtsPort !=0 ){
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Pin   = (1<<port->rtsPin_);
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
#if SOFTWARE_RTS == 0
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        G8_GPIO_Init(port->rtsPort, &GPIO_InitStructure);
        GPIO_PinAFConfig(port->rtsPort, port->rtsPin_, port->helper.altFunc);
#else
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
        G8_GPIO_Init(port->rtsPort, &GPIO_InitStructure);
#endif
    }

    if (port->ctsPort !=0 ){
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Pin = (1<<port->ctsPin_);
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        G8_GPIO_Init(port->ctsPort, &GPIO_InitStructure);
        GPIO_PinAFConfig(port->ctsPort, port->ctsPin_, port->helper.altFunc);
    }
    
    // install interrupt helpers
    switch((int)port->uart) {
    case (int)USART1: UART1_Port = port; break;
    case (int)USART2: UART2_Port = port; break;
    case (int)USART3: UART3_Port = port; break;
    case (int)UART4:  UART4_Port = port; break;
    case (int)UART5:  UART5_Port = port; break;
    case (int)USART6: UART6_Port = port; break;
    }
    
    USART_Cmd(port->uart, ENABLE);

    USART_ITConfig( port->uart, USART_IT_RXNE, ENABLE );
}

int32_t G8_SerialGetChar( ComPortHandle_t* pxPort, uint8_t* pcRxedChar, int32_t xBlockTime )
{
	/* Get the next character from the buffer.  Return false if no characters
	are available, or arrive before xBlockTime expires. */
	if( XOS_FixedQueueReceiveTimed( pxPort->rxQueue, pcRxedChar, xBlockTime )  == XOS_TIMEOUT_RESULT)
	{
		return -1;
	}
	else
	{
#if SOFTWARE_RTS == 1
        if(pxPort->rtsPort != NULL) {
            if (uxQueueMessagesWaiting(pxPort->rxQueue) <  pxPort->lowWater) {
            // The user space code only cleans this bit. The ISR sets it.
                pxPort->rtsPort->BSRRH = (1<<pxPort->rtsPin_);
            }
        }
#endif
		return *pcRxedChar;
	}
}
/*-----------------------------------------------------------*/

void G8_SerialPutString( ComPortHandle_t* pxPort, const uint8_t * const pcString )
{
    const uint8_t *pxNext = pcString;
	while( *pxNext )
	{
		G8_SerialPutChar( pxPort, *pxNext );
		pxNext++;
	}
}

uint32_t G8_SerialWriteBuffer( ComPortHandle_t* pxPort, const uint8_t * const buffer, uint16_t usStringLength )
{
    uint32_t sent = 0;
	for(int i=0;i<usStringLength;i++)
	{
		if (G8_SerialPutChar( pxPort, buffer[i]) >=0) {
            sent ++;
        }
	}
    return sent;
}

/*-----------------------------------------------------------*/

int32_t G8_SerialPutChar( ComPortHandle_t* pxPort, uint8_t cOutChar )
{
#ifdef BARE_TX_QUEUE
    int next = pxPort->txQueue.wptr + 1;
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

void G8_SerialClose(  ComPortHandle_t* port )
{
	if (port != NULL) {
		USART_ITConfig( port->uart, USART_IT_TXE, DISABLE );
        USART_ITConfig( port->uart, USART_IT_RXNE, DISABLE );
        USART_Cmd(port->uart, DISABLE);

        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

        if (port->txPort != 0) {
            GPIO_InitStructure.GPIO_Pin   = (1<<port->txPin_);
            G8_GPIO_Init(port->txPort, &GPIO_InitStructure);
        }
        
        if (port->rxPort !=0 ){
            GPIO_InitStructure.GPIO_Pin = (1<<port->rxPin_);
            G8_GPIO_Init(port->rxPort, &GPIO_InitStructure);
        }

        if (port->rtsPort !=0 ){
            GPIO_InitStructure.GPIO_Pin   = (1<<port->rtsPin_);
            G8_GPIO_Init(port->rtsPort, &GPIO_InitStructure);
        }

        if (port->ctsPort !=0 ){
            GPIO_InitStructure.GPIO_Pin = (1<<port->ctsPin_);
            G8_GPIO_Init(port->ctsPort, &GPIO_InitStructure);
        }
       
    
    
        // uninstall interrupt helpers
        switch((int)port->uart) {
        case (int)USART1: UART1_Port = NULL; break;
        case (int)USART2: UART2_Port = NULL; break;
        case (int)USART3: UART3_Port = NULL; break;
        case (int)UART4:  UART4_Port = NULL; break;
        case (int)UART5:  UART5_Port = NULL; break;
        case (int)USART6: UART6_Port = NULL; break;
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
    signed long xHigherPriorityTaskWoken = 0; // only used by FREERTOS

    uint8_t cChar;

#ifdef BARE_TX_QUEUE
    if (port->uart->SR & USART_FLAG_TXE) {
        if (port->txQueue.wptr != port->txQueue.rptr) {
            port->uart->DR = port->txQueue.buffer[port->txQueue.rptr];
            int next = port->txQueue.rptr + 1;
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
	
    if (port->uart->SR & USART_FLAG_RXNE)
	{
		cChar = port->uart->DR;
		XOS_FixedQueueSendFromISREx( port->rxQueue, &cChar, &xHigherPriorityTaskWoken );
        
#if SOFTWARE_RTS == 1
        if(port->rtsPort != NULL) {
            if(uxQueueMessagesWaitingFromISR(port->rxQueue) >  port->highWater ) {
                // the ISR only sets this bit. The user space code has to clear it.
                port->rtsPort->BSRRL = (1<<port->rtsPin_);
            }
        }
#endif
            
	}	

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


#include "project.h"

#ifdef DEBUG_UART
// this assumes that the debug UART is ready for data
void G8_SerialDebugWrite(const char* data) {
    __disable_interrupt();
    int len = strlen(data);
    for(int i=0;i<len;i++) {
        while((DEBUG_UART->SR & USART_FLAG_TXE) == 0) ;
        DEBUG_UART->DR = data[i];
    }
}
#endif

	
