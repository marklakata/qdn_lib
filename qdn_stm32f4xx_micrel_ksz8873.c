#include "g8_stm32f4xx_micrel_ksz8873.h"

#include "stm32f4xx_gpio.h"
#include "g8_stm32f4xx.h"


#define pd(port,pin) do { pullDown.GPIO_Pin = (1<<pin); G8_GPIO_Init(port, &pullDown);} while (0)
#define pu(port,pin) do { pullUp.GPIO_Pin = (1<<pin); G8_GPIO_Init(port, &pullUp);} while (0)


void G8_MicrelKSZ8873_StrapPins(void) {
    GPIO_InitTypeDef pullUp;
    GPIO_InitTypeDef pullDown;

    GPIO_StructInit(&pullUp); 
    GPIO_StructInit(&pullDown); 
   
    pullUp.GPIO_PuPd    = GPIO_PuPd_UP;
    pullUp.GPIO_Mode    = GPIO_Mode_IN;
    pullUp.GPIO_Speed   = GPIO_Speed_2MHz;

    pullDown.GPIO_PuPd  = GPIO_PuPd_DOWN;
    pullDown.GPIO_Mode  = GPIO_Mode_IN;
    pullDown.GPIO_Speed = GPIO_Speed_2MHz;

    pd(GPIOE,2); // pin 20 SMTXD33 PD  disable REFCLKO_3 output
    pu(GPIOA,7); // pin 28 SMRXDV3 PU  PU=PHY mode, PD=MAC mode
    pu(GPIOB,1); // pin 29 SMRXD33 PU  PU = enable P2ANEN (auto negotiation on P2)
    pu(GPIOB,0); // pin 30 SMRXD32 PU  PU force port 2 to 100BT if P2ANEN = 0
    pu(GPIOC,5); // pin 31 SMRXD31 PU  PU = port 2 default ot full duplex
    pd(GPIOC,4); // pin 33 SMRXD30 PD // PD = port 2 flow control is enabled by auto nego      

#if 0
???         pin 39 SPIQ PU // PU = 25 MHz
???         pin 58 P1LED1 PU // PU = always enable port 3 flow control feature
500ohm low  pin 59 P1LED0 PD // PD = port 3 to full duplex mode
???         pin 60,61 P2LED1 and P2LED0 PU // 1,1 = register access thru MDC and MDIO pins

could also be 0,1 (i2C slave) or 1,0 (SPI slave), 11 = SMI mode
#endif

}
