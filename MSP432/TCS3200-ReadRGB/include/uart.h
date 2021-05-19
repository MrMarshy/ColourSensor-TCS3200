#ifndef UART_H_
#define UART_H_

#include "msp.h"
#include "tinyprintf.h"

#define CLEAR_SCREEN_HOME   "\033[2j\033[H"
#define CLEAR_SCREEN        "\033[2j"

#ifdef __cplusplus
extern "C" {
#endif

void UART0_init(void);
void UART_clearScreen(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_H_ */
