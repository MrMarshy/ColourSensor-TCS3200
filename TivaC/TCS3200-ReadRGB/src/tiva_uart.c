#include "tiva_uart.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "tinyprintf.h"
#include "TM4C123GH6PM.h"
#include "inc/hw_ints.h"

char rx_buffer[RX_BUFF_LEN] = {0};
volatile int rx_idx = 0;

/* Configures the USB connection to be used as Serial */
void USB_UART_Init(bool with_interrupts){
    
    SYSCTL->RCGCUART |= BIT0;  /* Provide Clock to UART0 */
    SYSCTL->RCGCGPIO |= BIT0; /* Enable clock to PORTA */

    /* UART0 Initialization */
    
    /* See Page 903 of the Datasheet for calculations */
    UART0->CTL &= ~BIT0;                  // disable UART
    UART0->IBRD = 8;                    // IBRD = int(16MHz / 16 * 115200) = int(8.680555556)
    UART0->FBRD = 44;                     // FBRD = int(0.680555556 * 64 + 0.5) = 44
    UART0->CC = 0;                        // Use system clock
    UART0->LCRH = 0x60;                   // 8-bit, no parity, 1-stop bit, no fifo
    UART0->CTL = 0x301;                   // enable UART with RX and TX enabled
    
    GPIOA->AFSEL |= 0x03;           // enable alt funct PA1 and PA0
    GPIOA->DEN |= 0x03;             // enable digital i/o on PA1 and PA0
    GPIOA->PCTL = 0x11;             // configure PA0 and PA1 for UART

    if(with_interrupts){
      ROM_IntEnable(INT_UART0);
      ROM_UARTIntEnable(UART0_BASE, UART_INT_RX);
    }
}

void _putchar(char character){
    while((UART0->FR & BIT5) != 0);
    UART0->DR = character;
}

//------------UART_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none
void UART_OutChar(unsigned char data){
  while((UART0->FR & BIT5) != 0);
  UART0->DR = data;
}

//-----------------------UART_OutUDec-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
void UART_OutUDec(unsigned long n){
// This function uses recursion to convert decimal number
//   of unspecified length as an ASCII string
  if(n >= 10){
    UART_OutUDec(n/10);
    n = n%10;
  }
  UART_OutChar((unsigned char)n+'0'); /* n is between 0 and 9 */
}

//------------UART_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void UART_OutString(char *pt){
  while(*pt){
    UART_OutChar(*pt);
    pt++;
  }
}

