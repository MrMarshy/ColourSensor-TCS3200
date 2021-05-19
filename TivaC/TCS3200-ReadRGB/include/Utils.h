#ifndef AUTILS_H__
#define AUTILS_H__

#include <stdlib.h>
#include <stdint.h>
#include "bsp.h"

#define TICK_HZ     (1000U)

#define SYSTICK_TIM_CLK     (SYS_CLOCK_HZ)

#define INTERRUPT_DISABLE()  do{__asm volatile ("MOV R0,#0x1"); asm volatile("MSR PRIMASK,R0"); } while(0)

#define INTERRUPT_ENABLE()  do{__asm volatile ("MOV R0,#0x0"); asm volatile("MSR PRIMASK,R0"); } while(0)

#define ARRAY_SZ(x) ((sizeof(x)/sizeof(x[0])))

#define HIGH        0x01U
#define LOW         0x00U

#define BIT0        0x0001U
#define BIT1        0x0002U
#define BIT2        0x0004U
#define BIT3        0x0008U
#define BIT4        0x0010U
#define BIT5        0x0020U
#define BIT6        0x0040U
#define BIT7        0x0080U
#define BIT8        0x0100U
#define BIT9        0x0200U
#define BIT10       0x0400U
#define BIT11       0x0800U
#define BIT12       0x1000U
#define BIT13       0x2000U
#define BIT14       0x4000U
#define BIT15       0x8000U

#define BIT30       0x40000000U

#define PORTA       BIT0
#define PORTB       BIT1
#define PORTC       BIT2
#define PORTD       BIT3
#define PORTE       BIT4
#define PORTF       BIT5


//#define LED_RED             BIT1
//#define LED_BLUE            BIT2
//#define LED_GREEN           BIT3

// For SysTick
#define STCTRL              (*((volatile uint32_t*)0xE000E010U))
#define STRELOAD            (*((volatile uint32_t*)0xE000E014U))
#define STCURRENT           (*((volatile uint32_t*)0xE000E018U))
#define STCOUNT             0x10000U


//extern void assert_failed(char const *file, int line);
//extern void xmalloc_failed(size_t nbytes, char const *file, int line);

void Crap_Delay(int32_t const delay);


void SetupSysTick(uint32_t tick_hz);

// 1/SysClock * delay
void SysTickWait(uint32_t const delay);

void SysTickWaitMs(uint32_t const  ms_delay);

void SysTickWaitUs(uint32_t const us_delay);


void delay_ms(uint32_t ms);

long map(long x, long in_min, long in_max, long out_min, long out_max);

#endif // AUTILS_H__
