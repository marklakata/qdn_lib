  .syntax unified
  .cpu cortex-m3
  .fpu softvfp
  .thumb

  .section  .text.HardFault_Handler
  .weak  HardFault_Handler
  .type  HardFault_Handler, %function
HardFault_Handler:

/*        AAPCS BASE,INTERWORK
 *        PRESERVE8
 *        REQUIRE8
 */

.global hard_fault_handler_c


HardFault_Handler:
BusFault_Handler:
        TST LR, #4
        ITE EQ
        MRSEQ R0, MSP
        MRSNE R0, PSP
        B hard_fault_handler_c

        
