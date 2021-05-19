#include <stdint.h>
#include "msp.h"
#include "led.h"


Led::Led(){

    /* Configure P2.2-P2.0 for tri color RGB LEDs */
	P2->SEL1 &= ~7U; /* P2.2-P2.0 Digital I/O */
	P2->SEL0 &= ~7U;

	P2->DIR |= 7U; /* P2.2-P2.0 GPIO Output */
	P2->OUT &= ~7U;	/* All three LEDs are Off */

	P1->SEL1 &= (1 << LED1); /* P1.0 LED1 Digital I/O */
	P1->SEL0 &= ~(1 << LED1); 
	P1->DIR |= (1 << LED1); /* LED1 P1.0 is GPIO Output */
	P1->OUT &= ~(1 << LED1); /* Initally off */ 
}

void Led::rgb_led_on(uint8_t led_no){
    P2->OUT |= (1 << led_no);
}

void Led::rgb_led_off(uint8_t led_no){
    P2->OUT &= ~(1 << led_no);
}

void Led::rgb_led_toggle(uint8_t led_no){
    P2->OUT ^= (1 << led_no);
}

void Led::led1_on(void){
    P1->OUT |= (1 << LED1);
}

void Led::led1_off(void){
    P1->OUT &= ~(1 << LED1);
}

void Led::led1_toggle(void){
    P1->OUT ^= (1 << LED1);
}
