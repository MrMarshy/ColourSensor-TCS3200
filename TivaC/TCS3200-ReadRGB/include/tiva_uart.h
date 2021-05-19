#ifndef TIVA_UART__H_
#define TIVA_UART__H_

#include <stdint.h>
#include <stdbool.h>
#include "TM4C123GH6PM.h"
#include "Utils.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RX_BUFF_LEN     128
extern char rx_buffer[RX_BUFF_LEN];
extern volatile int rx_idx;


/* Configures the USB connection to be used as Serial */
void USB_UART_Init(bool with_interrupts);

//------------UART_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none
void UART_OutChar(unsigned char data);
//-----------------------UART_OutUDec-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
void UART_OutUDec(unsigned long n);

//------------UART_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void UART_OutString(char *pt);

#ifdef __cplusplus
}
#endif

#endif // TIVA_UART__H_