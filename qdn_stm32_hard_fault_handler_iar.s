        NAME HardFault_Handler

        AAPCS BASE,INTERWORK
        PRESERVE8
        REQUIRE8

        EXTERN hard_fault_handler_c

        SECTION .text:CODE:REORDER(2)
        THUMB

        PUBLIC  HardFault_Handler
        PUBLIC  BusFault_Handler

HardFault_Handler
BusFault_Handler
        TST LR, #4
        ITE EQ
        MRSEQ R0, MSP
        MRSNE R0, PSP
        B hard_fault_handler_c

        END
        