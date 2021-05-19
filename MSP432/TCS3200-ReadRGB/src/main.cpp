#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdbool.h>

#include "msp.h"

#include "main.h"
#include "bsp.h"
#include "led.h"
#include "spi.h"
#include "uart.h"
#include "led.h"
#include "tinyprintf.h"
#include "tcs3200.h"

/* Functions */
static void getRedColour(TCS3200* const sensor);
static void getGreenColour(TCS3200* const sensor);
static void getBlueColour(TCS3200* const sensor);

int main(void){

	UART0_init();
	
	printf("TCS3200 Colour Sensor Demo\r\n%s\r\n", __TIMESTAMP__);

	/* Initalize the onboard LEDs */
	Led leds;

	/* Initialize the TCS3200 Color Sensor */
	TCS3200 colourSensor;

	/* Start SysTick with interrupt every 1ms */
	init_systick_timer(TICK_HZ);

	for(int i = 0; i < 10; ++i){
		leds.rgb_led_on(RGB_GREEN);
		delay_ms(25);
		leds.rgb_led_off(RGB_GREEN);
		delay_ms(25);
	}

	leds.led1_toggle();

	for(;;){
		
		
		getRedColour(&colourSensor);
		printf("%s Diff: %.2f \r\n", colourSensor.getCurrFilterStr(), colourSensor.getDiff());
		printf("%s: %u \r\n\r\n", colourSensor.getCurrFilterStr(), colourSensor.getRedVal());

		getGreenColour(&colourSensor);
		printf("%s Diff: %.2f \r\n", colourSensor.getCurrFilterStr(), colourSensor.getDiff());
		printf("%s: %u \r\n\r\n", colourSensor.getCurrFilterStr(), colourSensor.getGreenVal());

		getBlueColour(&colourSensor);
		printf("%s Diff: %.2f \r\n", colourSensor.getCurrFilterStr(), colourSensor.getDiff());
		printf("%s: %u \r\n\r\n", colourSensor.getCurrFilterStr(), colourSensor.getBlueVal());
			
		
		
		delay_ms(1000);

	}		
}


static void getRedColour(TCS3200* const sensor){

	sensor->setRedFilter();

	__WFI();
	__NOP();

	while(!sensor->captureCompleted());
	
	//printf("%s Diff: %.2f \r\n", colourSensor.getCurrFilterStr(), colourSensor.getDiff());
		
	sensor->setCaptureComplete(false);
	

}

static void getGreenColour(TCS3200* const sensor){
	
	sensor->setGreenFilter();
	
	__WFI();
	__NOP();

	while(!sensor->captureCompleted());
	
	//printf("%s Diff: %.2f \r\n", colourSensor.getCurrFilterStr(), colourSensor.getDiff());
		
	sensor->setCaptureComplete(false);
}

static void getBlueColour(TCS3200* const sensor){
	
	sensor->setBlueFilter();

	__WFI();
	__NOP();

	while(!sensor->captureCompleted());
	
	//printf("%s Diff: %.2f \r\n", colourSensor.getCurrFilterStr(), colourSensor.getDiff());
		
	sensor->setCaptureComplete(false);

}
