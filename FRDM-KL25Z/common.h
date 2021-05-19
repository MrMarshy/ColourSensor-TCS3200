#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include "core_cm0plus.h"

// Memory locations defined by the linker
//extern uint32_t __heap_start[];
//extern uint32_t __StackTop[];
//extern uint32_t __data_start__[], __data_end__[];
//extern uint32_t __bss_start__[], __bss_end__[];
//extern uint32_t __etext[];                // End of code/flash

void delay(unsigned int ms);

// From accel.c
void accel_init(void);
int16_t accel_x(void);
int16_t accel_y(void);
int16_t accel_z(void);

// From touch.c
//int touch_data(int channel);
//void touch_init(uint32_t channel_mask);

// From _startup.c
void fault(uint32_t pattern);
#define FAULT_FAST_BLINK 	(0b10101010101010101010101010101010)
#define FAULT_MEDIUM_BLINK 	(0b11110000111100001111000011110000)
#define FAULT_SLOW_BLINK 	(0b11111111000000001111111100000000)

// usb.c
//void usb_init(void);
//void usb_dump(void);

// tests.c
void tests(void);

// Standard mapping function
long map(long x, long in_min, long in_max, long out_min, long out_max);


#endif // COMMON_H
