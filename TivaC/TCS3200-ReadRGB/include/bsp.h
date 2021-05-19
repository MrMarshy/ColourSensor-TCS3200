#ifndef BSP_H
#define BSP_H

#include "TM4C123GH6PM.h"
/* Board Support Package for the EK-TM4C123GXL board */

/* system clock setting [Hz] */
#define SYS_CLOCK_HZ 16000000U

/* on-board LEDs */
#define LED_RED   (1U << 1)
#define LED_BLUE  (1U << 2)
#define LED_GREEN (1U << 3)

void leds_init(void);

void red_led_on(void);
void blue_led_on(void);
void green_led_on(void);

void red_led_off(void);
void blue_led_off(void);
void green_led_off(void);

void red_led_toggle(void);
void blue_led_toggle(void);
void green_led_toggle(void);


typedef struct{
    GPIOA_Type *port_base;  /*<! Example: GPIOB_AHB */
    uint32_t pin;           /*<! Example: BIT5 */
    uint8_t port;           /*<! Example: PORTC */
}PinConfig_t;

#endif // BSP_H
