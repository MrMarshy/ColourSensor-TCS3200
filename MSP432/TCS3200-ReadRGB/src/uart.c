#include <uart.h>

static volatile uint8_t UART_rXData = 0;

/* For tinyprintf */
void _putchar(char c){

	// While Trasmit Flag is not set (wait until ready)
    while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));   

    EUSCI_A0->TXBUF = c;
}

void UART0_init(){

    /* Set P1.2 and P1.3 as UCA0RXD and UCA0TXD */
    P1->SEL0 |= BIT2 | BIT3;   

/** UART Configuration for 115200 with Clk @ SMCLK of 3MHz.
 *   Value generated using the tool provided on the following page:
 * 
 *  http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 * 
 */ 

    if(SystemCoreClock == 3000000U){
        /* 8-bit, 1 stop bit, no parity */
        EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST;         // Put in reset mode while configuring 
        EUSCI_A0->MCTLW = 0xA0 | EUSCI_A_MCTLW_OS16;    // First Modulation is 10, Oversampling enabled
        EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SSEL__SMCLK;   // SMCLK
        EUSCI_A0->BRW = 0x01;                           // Clock prescalar 1
    }
    else if(SystemCoreClock == 48000000U){
        /* 8-bit, 1 stop bit, no parity */
        EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST;         // Put in reset mode while configuring 
        EUSCI_A0->MCTLW = 0x09 | EUSCI_A_MCTLW_OS16;    // First Modulation is 9, Oversampling enabled
        EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SSEL__SMCLK;   // SMCLK
        EUSCI_A0->BRW = 0x1A;                           // Clock prescalar 26
    }
    /* Clear SWRST to resume */
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;

    /* Enable Rx interrupts */
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE; 

    NVIC_SetPriority(EUSCIA0_IRQn, 4); /* set priority to 4 in NVIC */
    NVIC_EnableIRQ(EUSCIA0_IRQn);      /* enable interrupt in NVIC */

}

void UART_clearScreen(void){
    printf("%s", CLEAR_SCREEN_HOME);
    
}


void EUSCIA0_IRQHandler(void){

    if(EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG){

         // Get Received UART data , rx interrupt flag is cleared by read
        (void)EUSCI_A0->RXBUF;              
    }
}