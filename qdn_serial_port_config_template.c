// this file is included from g8_serial.c
// copy this file to your local project as "g8_serial_port_config.c" and edit as necessary.

#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5


//   UART    TX Port,Pin  RX Port,Pin  RTS Port,Pin  CTS Port,Pin   priority
    {USART1, GPIOA,  9,   GPIOA, 10,   GPIOA,12,     GPIOA,11,      configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY},
//    {USART2, 0,0,         0,0,         0,0,          0,0,         configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY},
//    {USART3, GPIOB, 10,   GPIOB, 11,   GPIOB,14,     GPIOB,13,    configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY},
//    {USART3, GPIOD, 8,    GPIOD, 9,    GPIOD,12,     GPIOD,11,    configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY},
//    {UART4 , GPIOC, 10,   GPIOC, 11,   0,0,          0,0,         configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY},
//    {UART5 , GPIOC, 12,   GPIOD,  2,   0,0,          0,0,         configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY},
//    {USART6, GPIOC,  6,   GPIOC,  7,   GPIOG,8,      GPIOG,15,    configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY},
//    {USART6, GPIOC, 6,    GPIOG, 9,    0,0,          0,0,         configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY},
