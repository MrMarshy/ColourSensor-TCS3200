#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "msp.h"
#include "spi.h"


static volatile uint8_t SPI_rxData = 0;
static volatile uint8_t SPI_txData = 0;

/* Default Settings */
static SPI_settings spi_settings = {
    .bit_order = SPI_MSBFIRST, // MSB First
    .mode = SPI_MODE0
};
static volatile bool spi_xfer_done;

void SPI_chipSelect(void){
    // Active low
    P2->OUT &= ~BIT3;
}

void SPI_chipDeselect(void){
   
    P2->OUT |= BIT3;
}

/* Init SPI1 in master mode to with phase, polarity */
void SPI1_init(SPI_settings const * const settings){
    
    if(settings){
        spi_settings = *settings;
    }

    /** Set 3-SPI Pins as second function
     *  MISO - P1.7
     *  MOSI - P1.6
     *  SCLK - P1.5
      */
    P1->SEL0 |=  BIT5 | BIT6 | BIT7;

    /* SS - P2.3 */
    P2->SEL1 &= ~BIT3;
    P2->SEL0 &= ~BIT3;
    P2->DIR |= BIT3;
    P2->OUT |= BIT3; /* Assert High */

    /* Put into reset */
    EUSCI_B0_SPI->CTLW0 |= EUSCI_B_CTLW0_SWRST;
    

    EUSCI_B0_SPI->CTLW0 = EUSCI_B_CTLW0_SWRST | // remain in reset
        EUSCI_B_CTLW0_MST |          // Master Mode
        EUSCI_B_CTLW0_SYNC |         // Synchronous Mode
        EUSCI_B_CTLW0_SSEL__SMCLK;   // BRCLK is SMCLK

    switch(settings->bit_order){

        case SPI_MSBFIRST:

            EUSCI_B0_SPI->CTLW0 |= EUSCI_B_CTLW0_MSB;
            break;

        case SPI_LSBFIRST:

            EUSCI_B0_SPI->CTLW0 &= ~EUSCI_B_CTLW0_MSB;
            break;

        default:
            ;
    }

    switch(settings->mode){

        case SPI_MODE0:

            // The inactive state is low
            EUSCI_B0_SPI->CTLW0 &= ~EUSCI_B_CTLW0_CKPL;

            // Data is captured on the first UCLK edge and changed on the following edge.
            EUSCI_B0_SPI->CTLW0 |= EUSCI_B_CTLW0_CKPH;

            break;

        case SPI_MODE1:

            // The inactive state is low
            EUSCI_B0_SPI->CTLW0 &= ~EUSCI_B_CTLW0_CKPL;

            // Data is changed on the first UCLK edge and captured on the following edge
            EUSCI_B0_SPI->CTLW0 &= ~EUSCI_B_CTLW0_CKPH;

            break;

        case SPI_MODE2:

            // The inactive state is high
            EUSCI_B0_SPI->CTLW0 |= EUSCI_B_CTLW0_CKPL;

            // Data is changed on the first UCLK edge and captured on the following edge
            EUSCI_B0_SPI->CTLW0 &= ~EUSCI_B_CTLW0_CKPH;

            break;

        case SPI_MODE3:

            // The inactive state is high
            EUSCI_B0_SPI->CTLW0 |= EUSCI_B_CTLW0_CKPL;

            // Data is captured on the first UCLK edge and changed on the following edge.
            EUSCI_B0_SPI->CTLW0 |= EUSCI_B_CTLW0_CKPH;

            break;

        default:
            ;
    }
    

    EUSCI_B0_SPI->BRW = 0x02U;          // Prescaler of 3, BitClock = BRCLK/(BRW)

    EUSCI_B0_SPI->IFG &= ~EUSCI_B_IFG_TXIFG;  // Clear Tx interrupt flag
    EUSCI_B0_SPI->IFG &= ~EUSCI_B_IFG_RXIFG;  // Clear Rx interrupt flag
    EUSCI_B0_SPI->IE &= ~EUSCI_B__TXIE;      // Disable Tx interrupt

    // Enable eUSCIB0 interrupt in NVIC
    NVIC_SetPriority(EUSCIB0_IRQn, 2); /* set priority to 2 in NVIC */
    NVIC_EnableIRQ(EUSCIB0_IRQn);      /* enable interrupt in NVIC */

    EUSCI_B0_SPI->CTLW0 &= ~EUSCI_B_CTLW0_SWRST; // Initialize

}

void SPI1_deInit(void){
     /* Put into reset */
    EUSCI_B0_SPI->CTLW0 |= EUSCI_B_CTLW0_SWRST;
}

void SPI1_enable(void){
    /* Take out of reset */
    EUSCI_B0_SPI->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;
}

void SPI1_disable(void){
    SPI1_deInit();
}

void SPI_configure(SPI_settings const * const settings){

    if(!settings){
        return;
    }
    else{
        spi_settings = *settings;
    }

    /* Put into reset */
    EUSCI_B0_SPI->CTLW0 |= EUSCI_B_CTLW0_SWRST;

    switch(settings->bit_order){

        case SPI_MSBFIRST:

            EUSCI_B0_SPI->CTLW0 |= EUSCI_B_CTLW0_MSB;
            break;

        case SPI_LSBFIRST:

            EUSCI_B0_SPI->CTLW0 &= ~EUSCI_B_CTLW0_MSB;
            break;

        default:
            ;
    }

    switch(settings->mode){

        case SPI_MODE0:

            // The inactive state is low
            EUSCI_B0_SPI->CTLW0 &= ~EUSCI_B_CTLW0_CKPL;

            // Data is captured on the first UCLK edge and changed on the following edge.
            EUSCI_B0_SPI->CTLW0 |= EUSCI_B_CTLW0_CKPH;

            break;

        case SPI_MODE1:

            // The inactive state is low
            EUSCI_B0_SPI->CTLW0 &= ~EUSCI_B_CTLW0_CKPL;

            // Data is changed on the first UCLK edge and captured on the following edge
            EUSCI_B0_SPI->CTLW0 &= ~EUSCI_B_CTLW0_CKPH;

            break;

        case SPI_MODE2:

            // The inactive state is high
            EUSCI_B0_SPI->CTLW0 |= EUSCI_B_CTLW0_CKPL;

            // Data is changed on the first UCLK edge and captured on the following edge
            EUSCI_B0_SPI->CTLW0 &= ~EUSCI_B_CTLW0_CKPH;

            break;

        case SPI_MODE3:

            // The inactive state is high
            EUSCI_B0_SPI->CTLW0 |= EUSCI_B_CTLW0_CKPL;

            // Data is captured on the first UCLK edge and changed on the following edge.
            EUSCI_B0_SPI->CTLW0 |= EUSCI_B_CTLW0_CKPH;

            break;

        default:
            ;
    }

    /* Take out of reset */
    EUSCI_B0_SPI->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;
}


void SPI_beginTransaction(SPI_settings const * const settings){

    SPI1_deInit();
    SPI_configure(settings);
    SPI1_enable();
    
}

void SPI_endTransaction(void){
    SPI1_deInit();
}

uint8_t SPI_transfer(uint8_t data){

    SPI_txData = data;

    EUSCI_B0_SPI->IFG |= EUSCI_B_IFG_TXIFG;// Set TXIFG flag
    EUSCI_B0_SPI->IE |= EUSCI_B__TXIE;    // Enable TX interrupt

    return SPI_rxData;
}

uint16_t SPI_transfer16(uint16_t data){

    union{
        uint16_t val;
        struct{
            uint8_t lsb;
            uint8_t msb;
        };
    }t;

    t.val = data;

    if(spi_settings.bit_order == SPI_LSBFIRST){
        t.lsb = SPI_transfer(t.lsb);
        t.msb = SPI_transfer(t.msb);
    }
    else{
        t.msb = SPI_transfer(t.msb);
        t.lsb = SPI_transfer(t.lsb);
    }

    return t.val;

}

void SPI_setBitOrder(SPI_BitOrder const bit_order){
    spi_settings.bit_order = bit_order;
}

void SPI_setDataMode(SPI_Mode const mode){
    spi_settings.mode = mode;
}


void EUSCIB0_IRQHandler(void){

	if (EUSCI_B0_SPI->STATW & EUSCI_B_STATW_OE){
		/* Overrun Error */
        while(1){;}
	}

    if(EUSCI_B0_SPI->STATW & EUSCI_B_STATW_FE){
        /* Bus Conflict */
       while(1){;}
    }

    /* Tx Interrupt */
    if(EUSCI_B0_SPI->IFG & EUSCI_B_IFG_TXIFG){

        /* Tx chars */
        EUSCI_B0_SPI->TXBUF = SPI_txData;

        /* Clear tx interrupt flag */
        EUSCI_B0_SPI->IFG &= ~EUSCI_B_IFG_TXIFG;

        /* Disable Tx interrupts */
        EUSCI_B0_SPI->IE &= ~EUSCI_B__TXIE;

        // Wait until Bus is not busy
        while ((EUSCI_B0_SPI->STATW & EUSCI_B_STATW_BUSY));

        SPI_rxData = EUSCI_B0_SPI->RXBUF;

        // Clear the receive interrupt flag
        EUSCI_B0_SPI->IFG &= ~EUSCI_B_IFG_RXIFG;
        
    }

    // /* Rx Interrupt */
    // if(EUSCI_B0_SPI->IFG & EUSCI_B_IFG_RXIFG){

    //     SPI_rxData = EUSCI_B0_SPI->RXBUF; // Reading  RXBUF clears rx interrupt flag

    //      /* Disable Rx interrupts */
    //     EUSCI_B0_SPI->IE &= ~EUSCI_B__RXIE;
    // }
    
}