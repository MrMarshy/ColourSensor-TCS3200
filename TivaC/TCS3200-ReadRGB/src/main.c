#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "TM4C123GH6PM.h"
#include "bsp.h"
#include "tiva_uart.h"
#include "Utils.h"
#include "tinyprintf.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"

#include "TCS3200.h"

/**
 * TCS3200 Colour Sensor to Tiva Pins
 * -----------------------------------------------
 * S0 pin connected to Tiva Pin PC4
 * S1 pin connected to Tiva Pin PC5 
 * S2 pin connected to Tiva Pin PC6 
 * S3 pin connected to Tiva Pin PC7
 * Out pin connected to Tiva Pin PB6
 * OE pin connected to Tiva GND
 * VCC pin connected to Tiva 3v3
 * GND pin connected to Tiva GND
 */


typedef struct{
    volatile bool is_uart_string_ready;
}globals_t;

typedef struct{
    uint32_t namesz;
    uint32_t descsz;
    uint32_t type;
    uint8_t data[];

}ElfNoteSection_t;

extern const ElfNoteSection_t g_note_build_id;

static globals_t globals = {0};
static TCS3200_t colourSensor = {0};

static void init_hardware(void);
static void print_build_id(void);


void UART0_isr(void);

int main(void){

    __disable_irq();
    
    init_hardware();

    const PinConfig_t configs[] = {
        {.port_base = GPIOC_AHB, .port = PORTC, .pin = BIT4}, // S0
        {.port_base = GPIOC_AHB, .port = PORTC, .pin = BIT5}, // S1
        {.port_base = GPIOC_AHB, .port = PORTC, .pin = BIT6}, // S2
        {.port_base = GPIOC_AHB, .port = PORTC, .pin = BIT7}, // S3
        {.port_base = GPIOB_AHB, .port = PORTB, .pin = BIT6}, // Freq Out
    };
    const uint8_t do_calib = 0;
    TCS3200_FreqScaler_t scale_factor = TCS3200_SCALE_20_PCT;
    TCS3200_Init(&colourSensor, do_calib, configs, ARRAY_SZ(configs), scale_factor);
    TCS3200_SetFreqScaling(&colourSensor, TCS3200_SCALE_20_PCT);

    __enable_irq();
    
    printf("TCS3200 Colour Sensor Example\r\n");
    printf("Built: %s %s\r\n\r\n", __DATE__, __TIME__);
    print_build_id();
    
    int rw, gw, bw;

    for(;;) {


        rw = TCS3200_GetRedPulseWidth(&colourSensor);
        delay_ms(200);

        gw = TCS3200_GetGreenPulseWidth(&colourSensor);
        delay_ms(200);

        bw = TCS3200_GetBluePulseWidth(&colourSensor);
        delay_ms(200);
        
        red_led_toggle();

        if(colourSensor.doCalibration){
            printf("RW: %d GW: %d BW: %d\r\n", rw, gw, bw);
        }
        printf("%d,%d,%d\r\n", colourSensor.redValue, colourSensor.greenValue, colourSensor.blueValue);
    }

}

static void print_build_id(void){

    const uint8_t *build_id_data = &g_note_build_id.data[g_note_build_id.namesz];

    printf("BUILD ID: ");

    for(int i = 0; i < g_note_build_id.descsz; ++i){
        printf("%02x", build_id_data[i]);
    }

    printf("\r\n");
}


/**
 * Setup UART, GPIO etc
 */
static void init_hardware(void){

    SetupSysTick(TICK_HZ);

    /* Initialise the UART for 115200 baud with interrupt enabled */
    USB_UART_Init(true);

    /* Initialise the on board LEDs as digital outputs */
    leds_init();
}


// UART0_isr
void UART0_isr(void){
  uint32_t status;
  
  // Get interrupt status.
  status = ROM_UARTIntStatus(UART0_BASE, true);

  // Clear the asserted interupts
  ROM_UARTIntClear(UART0_BASE, status);

  // Loop while chars are available in the RX FIFO
  while(ROM_UARTCharsAvail(UART0_BASE)){
      
      if(rx_idx == (RX_BUFF_LEN - 1)){
          rx_idx = 0;
      }
      rx_buffer[rx_idx] = ROM_UARTCharGetNonBlocking(UART0_BASE);

      if((rx_buffer[rx_idx] == '\n') || (rx_buffer[rx_idx] == '\r')){
          rx_buffer[rx_idx] = '\0';
          rx_idx = 0;
          globals.is_uart_string_ready = true;
          break;
      }
      else{
          globals.is_uart_string_ready = false;
          rx_idx++;
      }
  }
}