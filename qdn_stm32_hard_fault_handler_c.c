// From Joseph Yiu, minor edits by FVH
// hard fault handler in C,
// with stack frame location as input parameter
// called from HardFault_Handler in file xxx.s

#include "stm32f10x.h"
#include "qdn_stm32_hard_fault_handler.h"
#include "qdn_cpu.h"

__no_init volatile QDN_StackDump_t qdn_StackDump;

void hard_fault_handler_c (unsigned int * hardfault_args);

void hard_fault_handler_c (unsigned int * hardfault_args) { // ISR
#ifdef PRINTF_ON_HARDFAULT
  unsigned int stacked_r0;
  unsigned int stacked_r1;
  unsigned int stacked_r2;
  unsigned int stacked_r3;
  unsigned int stacked_r12;
  unsigned int stacked_lr;
  unsigned int stacked_pc;
  unsigned int stacked_psr;
 
  stacked_r0 = ((unsigned long) hardfault_args[0]);
  stacked_r1 = ((unsigned long) hardfault_args[1]);
  stacked_r2 = ((unsigned long) hardfault_args[2]);
  stacked_r3 = ((unsigned long) hardfault_args[3]);
 
  stacked_r12 = ((unsigned long) hardfault_args[4]);
  stacked_lr = ((unsigned long) hardfault_args[5]);
  stacked_pc = ((unsigned long) hardfault_args[6]);
  stacked_psr = ((unsigned long) hardfault_args[7]);
 
  printf ("\n\n[Hard fault handler - all numbers in hex]\n");
  printf ("R0 = %x\n", stacked_r0);
  printf ("R1 = %x\n", stacked_r1);
  printf ("R2 = %x\n", stacked_r2);
  printf ("R3 = %x\n", stacked_r3);
  printf ("R12 = %x\n", stacked_r12);
  printf ("LR [R14] = %x  subroutine call return address\n", stacked_lr);
  printf ("PC [R15] = %x  program counter\n", stacked_pc);
  printf ("PSR = %x\n", stacked_psr);
  printf ("BFAR = %x\n", (*((volatile unsigned long *)(0xE000ED38))));
  printf ("CFSR = %x\n", (*((volatile unsigned long *)(0xE000ED28))));
  printf ("HFSR = %x\n", (*((volatile unsigned long *)(0xE000ED2C))));
  printf ("DFSR = %x\n", (*((volatile unsigned long *)(0xE000ED30))));
  printf ("AFSR = %x\n", (*((volatile unsigned long *)(0xE000ED3C))));
  printf ("SCB_SHCSR = %x\n", SCB->SHCSR);
#else
  qdn_StackDump.R0  = ((uint32_t) hardfault_args[0]);
  qdn_StackDump.R1  = ((uint32_t) hardfault_args[1]);
  qdn_StackDump.R2  = ((uint32_t) hardfault_args[2]);
  qdn_StackDump.R3  = ((uint32_t) hardfault_args[3]);
  qdn_StackDump.R12 = ((uint32_t) hardfault_args[4]);
  qdn_StackDump.LR  = ((uint32_t) hardfault_args[5]);
  qdn_StackDump.PC  = ((uint32_t) hardfault_args[6]);
  qdn_StackDump.PSR = ((uint32_t) hardfault_args[7]);
  
  qdn_StackDump.BFAR = (*((volatile unsigned long *)(0xE000ED38)));
  qdn_StackDump.CFSR = (*((volatile unsigned long *)(0xE000ED28)));
  qdn_StackDump.HFSR = (*((volatile unsigned long *)(0xE000ED2C)));
  qdn_StackDump.DFSR = (*((volatile unsigned long *)(0xE000ED30)));
  qdn_StackDump.AFSR = (*((volatile unsigned long *)(0xE000ED3C)));
  qdn_StackDump.SCB_SHCSR = SCB->SHCSR;
  qdn_StackDump.status = 0xDEADBEEF;
  
#endif
  
  // this will let the watchdog do a reset, if it is enabled.
  while (1);
}
