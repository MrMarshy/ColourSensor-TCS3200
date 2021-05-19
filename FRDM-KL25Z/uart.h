#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "tinyprintf.h"

#define CLEAR_SCREEN_HOME   "\033[2j\033[H"
#define CLEAR_SCREEN        "\033[2j"

int uart_write(char *p, int len);

int uart_read(char *p, int len);

// A blocking write, useful for error/crash/debug reporting
int uart_write_err(char *p, int len);

//
// uart_init() -- Initialize debug / OpenSDA UART0
//
//      The OpenSDA UART RX/TX is conntected to pins 27/28, PTA1/PTA2 (ALT2)
//
void uart_init(uint32_t baud_rate);

#endif // UART_H
