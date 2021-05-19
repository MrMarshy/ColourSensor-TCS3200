#ifndef TIMEIT_H_
#define TIMEIT_H_

// The address of the DWT_CTRL register
volatile unsigned int *DWT_CTRL = (unsigned int *)0xE0001000;

// The address of the DWT_CYCCNT register
volatile unsigned int *DWT_CYCCNT = (unsigned int*)0xE0001004;

// The addres of the (Debug Exception and Monitor Control Register) DEMCR register
volatile unsigned int *DEMCR = (unsigned int*)0xE000EDFC;

void StartTiming(void){
    *DEMCR = *DEMCR | 0x01000000; // Enable the DWT unit
    *DWT_CTRL = *DWT_CTRL | 1; // Enable the cycle counter
    *DWT_CYCCNT = 0; // Reset the cycle counter
}

unsigned int StopTiming(void){
    return *DWT_CYCCNT; // return the total number of clock cycles spent executing the code block
}

float CalculateTime(unsigned int cycle, unsigned int frequency){
    return (float)cycle/frequency; // return time in seconds it took to execute the code block
}

#endif /* TIMEIT_H_ */