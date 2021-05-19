#ifndef INTERRUPTS_H_
#define INTERRUPTS_H_

#include <stdint.h>
#include "tinyprintf.h"

#ifdef __cplusplus
extern "C" {
#endif

    /* TimerA0 */
    void TA0_0_IRQHandler(void);
    void TA0_N_IRQHandler(void);
    
    /* Exception Handlers */
	void SysTick_Handler(void);


    /* This is the code that gets called when the processor receives a fault        */
    /* interrupt.  This simply enters an infinite loop, preserving the system state */
    /* for examination by a debugger.                                               */
    void HardFault_Handler(void)
    {
        printf("Exception : Hardfault\n");
        /* Enter an infinite loop. */
        while(1)
        {
        }
    }

    void MemManage_Handler(void){

        printf("Exception : MemManage\n");
        while(1);
    }

    void BusFault_Handler(void){
        
        printf("Exception : BusFault\n");
        while(1);
    }

    /* This is the code that gets called when the processor receives a NMI.  This  */
    /* simply enters an infinite loop, preserving the system state for examination */
    /* by a debugger.                                                              */
    void NMI_Handler(void){

        printf("Exception : NMI\n");
        /* Enter an infinite loop. */
        while(1)
        {
        }
    }

#ifdef __cplusplus
}
#endif

#endif /* INTERRUPTS_H_ */
