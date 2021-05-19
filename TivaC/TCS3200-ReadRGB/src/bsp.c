/* Board Support Package */
#include "TM4C123GH6PM.h"
#include "bsp.h"


void leds_init(void){

    SYSCTL->RCGCGPIO  |= (1U << 5); /* enable Run mode for GPIOF */
    while( !(SYSCTL->RCGCGPIO & (1U << 5))){}

    SYSCTL->GPIOHBCTL |= (1U << 5); /* enable AHB for GPIOF */
    while( !(SYSCTL->GPIOHBCTL & (1U << 5))){}

    GPIOF_AHB->DIR |= (LED_RED | LED_BLUE | LED_GREEN);
    GPIOF_AHB->DEN |= (LED_RED | LED_BLUE | LED_GREEN);
}

void red_led_on(void){
    GPIOF_AHB->DATA_Bits[LED_RED] = LED_RED;
}

void blue_led_on(void){
    GPIOF_AHB->DATA_Bits[LED_BLUE] = LED_BLUE;
}

void green_led_on(void){
    GPIOF_AHB->DATA_Bits[LED_GREEN] = LED_GREEN;
}

void red_led_off(void){
    GPIOF_AHB->DATA_Bits[LED_RED] = 0U;
}

void blue_led_off(void){
    GPIOF_AHB->DATA_Bits[LED_BLUE] = 0U;
}
void green_led_off(void){
    GPIOF_AHB->DATA_Bits[LED_GREEN] = 0U;
}

void red_led_toggle(void){
    GPIOF_AHB->DATA_Bits[LED_RED] ^= LED_RED;
}

void blue_led_toggle(void){
    GPIOF_AHB->DATA_Bits[LED_BLUE] ^= LED_BLUE;
}

void green_led_toggle(void){
    GPIOF_AHB->DATA_Bits[LED_GREEN] ^= LED_GREEN;
}
