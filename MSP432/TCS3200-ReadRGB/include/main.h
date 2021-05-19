
#ifndef MAIN_H_
#define MAIN_H_


//#define SRAM_START               0x20000000U
//#define SIZE_SRAM                ( (128) * (1024))
//#define SRAM_END                 ((SRAM_START) + (SIZE_SRAM) )


#define INTERRUPT_DISABLE()  do{__asm volatile ("MOV R0,#0x1"); asm volatile("MSR PRIMASK,R0"); } while(0)

#define INTERRUPT_ENABLE()  do{__asm volatile ("MOV R0,#0x0"); asm volatile("MSR PRIMASK,R0"); } while(0)

#endif /* MAIN_H_ */
