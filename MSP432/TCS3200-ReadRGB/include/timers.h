#ifndef MSP432_TIMERS_H
#define MSP432_TIMERS_H

#define INPUT_CAPTURE_DEFAULT_CONFIG() \
/* uses TA0.2 for capture mode of pin P2.5. */ \
P2->SEL1 &= ~BIT5; \
P2->SEL0 |= BIT5; \
P2->DIR &= ~BIT5; \
/* Clock Source SMCLK, No Divider, Reset Count, Timer Counts up to 0xFFFF, */ \
TIMER_A0->CTL = (TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_CLR | TIMER_A_CTL_MC__CONTINUOUS); \
/* Capture Rising Edge, CCI2A, Synchromous Capture, Capture Mode */ \
TIMER_A0->CCTL[2] = (TIMER_A_CCTLN_CM_1 | TIMER_A_CCTLN_CCIS_0 | TIMER_A_CCTLN_SCS | TIMER_A_CCTLN_CAP); \
/* Enable Capture Interrupt */ \
TIMER_A0->CCTL[2] |= TIMER_A_CCTLN_CCIE; \
/* Divide Input Clock By 1 */ \
TIMER_A0->EX0 = TIMER_A_EX0_TAIDEX_0; \
NVIC_EnableIRQ(TA0_N_IRQn)
 
#endif // MSP432_TIMERS_H