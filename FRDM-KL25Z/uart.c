#include <MKL04Z4.h>
#include "common.h"
#include "uart.h"
#include "ring.h"



// Circular buffers for transmit and receive
#define BUFLEN 128

static uint8_t _tx_buffer[sizeof(RingBuffer) + BUFLEN] __attribute__ ((aligned(4)));
static uint8_t _rx_buffer[sizeof(RingBuffer) + BUFLEN] __attribute__ ((aligned(4)));

static RingBuffer *const tx_buffer = (RingBuffer *) &_tx_buffer;
static RingBuffer *const rx_buffer = (RingBuffer *) &_rx_buffer;

void UART0_IRQHandler(void);

/* For tinyprintf */
void _putchar(char c){
	
	while(!(UART0->S1 & 0x80)); /* Wait for tx buffer to be empty */
	UART0->D = (uint8_t)c;
}

//
// uart_init() -- Initialize debug / OpenSDA UART0
//
//      The OpenSDA UART RX/TX is conntected to pins 27/28, PTA1/PTA2 (ALT2)
//      If not using OpenSDA Debugging then connect FTDI to pins PTA1 (RX) / PTA2 (TX)
//
void uart_init(uint32_t baud_rate){
		SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
        
    // Turn on clock to UART0 module and select 48Mhz clock (FLL/PLL source)
    SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;
    SIM->SOPT2 &= ~SIM_SOPT2_UART0SRC_MASK;
    SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);                 // FLL/PLL source

    // Select "Alt 2" usage to enable UART0 on pins
    PORTA->PCR[1] = PORT_PCR_MUX(2);
    PORTA->PCR[2] = PORT_PCR_MUX(2);

    UART0->C2 = 0;
    UART0->C1 = 0;
    UART0->C3 = 0;
    UART0->S2 = 0;     

    // Set the baud rate divisor
    #define OVER_SAMPLE 16u
    uint32_t divisor = (SystemCoreClock / OVER_SAMPLE) / baud_rate;
    UART0->C4 = UARTLP_C4_OSR(OVER_SAMPLE - 1);
    UART0->BDH = (divisor >> 8) & UARTLP_BDH_SBR_MASK;
    UART0->BDL = (divisor & UARTLP_BDL_SBR_MASK);

    // Initialize transmit and receive circular buffers
    buf_reset(tx_buffer, BUFLEN);
    buf_reset(rx_buffer, BUFLEN);

    // Enable the receiver interrupts
    UART0->C2 = UARTLP_C2_RE_MASK | UART0_C2_TE_MASK |UART0_C2_RIE_MASK;
    NVIC_EnableIRQ(UART0_IRQn);
}

/* Only to be used when tinyprintf is not used */
int uart_write(char *p, int len){
	
	int i = 0;
	
	for(i = 0; i < len; ++i){
		while(buf_isfull(tx_buffer)); /* Spin wait while full */
		
		buf_put_byte(tx_buffer, *p++);
		UART0->C2 |= UART0_C2_TIE_MASK; /* Turn on Tx Interrupt */
	}
	
	return len;
}

int uart_read(char *p, int len){
	
	int i = len;
	
	while(i > 0){
		while(buf_isempty(rx_buffer)); /* Spin wait */
		
		*p++ = buf_get_byte(rx_buffer);
		UART0->C2 |= UART0_C2_RIE_MASK; /* Turn on the Rx Interrupt */
		i--;
	}
	
	return len - i;
}

// A blocking write, useful for error/crash/debug reporting
int uart_write_err(char *p, int len){
	
	int i = 0;
	
	__disable_irq();
	for(i = 0; i < len; ++i){
		while((UART0->S1 & UART0_S1_TDRE_MASK) == 0); /* Wait until transmit buffer empty */
		
		UART0->D = *p++; /* Send character */
	}
	
	__enable_irq();
	return len;
}


void UART0_IRQHandler()
{
    uint32_t status;
   
    status =  UART0->S1;
    
    // If there is received data, read it into the receive buffer.  If the
    // buffer is full, disable the receive interrupt.
    if ((status & UART0_S1_RDRF_MASK) && !buf_isfull(rx_buffer)) {
        buf_put_byte(rx_buffer, UART0->D);
        if(buf_isfull(rx_buffer))
            UART0->C2 &= ~UART0_C2_RIE_MASK;
    }
}
