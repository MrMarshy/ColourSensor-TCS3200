#ifndef MSP432_GPIO_H
#define MSP432_GPIO_H

#include "msp.h"

#define GPIO_OUTPUT_CONFIG(port, pin) \
port->SEL1 &= ~pin; \
port->SEL0 &= ~pin; \
port->DIR |= pin; \
port->OUT &= ~pin	 

#define GPIO_SET(port, pin) \
port->OUT |= pin

#define GPIO_RESET(port, pin) \
port->OUT &= ~pin

#define GPIO_TOGGLE(port, pin) \
port->OUT ^= pin

#define GPIO_SWITCHES_INPUT_CONFIG() \
P1->SEL1 &= ~0x12; \
P1->SEL0 &= ~0x12; \
P1->DIR &= ~0x12; \
P1->REN |= 0x12; \
P1->OUT |= 0x12

#define GPIO_SWITCHES_INTR_ENABLE() \
P1->IES |= 0x12; \
P1->IFG = 0; \
P1->IE |= 0x12; \
NVIC_EnableIRQ(PORT1_IRQn)

#define GPIO_INPUT_CONFIG(port, pin) \
port->SEL1 &= ~pin; \
port->SEL0 &= ~pin; \
port->DIR &= ~pin; \
port->REN |= pin; \
port->OUT |= pin

#define GPIO_INPUT_INTR_ENABLE(port, pin, irqn) \
port->IES |= pin; \
port->IFG = 0; \
port->IE |= pin; \
NVIC_EnableIRQ(irqn) 

#endif // MSP432_GPIO_H