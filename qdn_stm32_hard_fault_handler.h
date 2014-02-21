// qdn_stm32_hard_fault_handler.h

#include <stdint.h>

typedef struct  {
    uint32_t status;
    uint32_t R0;
    uint32_t R1;
    uint32_t R2;
    uint32_t R3;
    uint32_t R12;
    uint32_t LR;
    uint32_t PC;
    uint32_t PSR;
    uint32_t BFAR;
    uint32_t CFSR;
    uint32_t HFSR;
    uint32_t DFSR;
    uint32_t AFSR;
    uint32_t SCB_SHCSR;
}  QDN_StackDump_t;

extern volatile QDN_StackDump_t qdn_StackDump;

