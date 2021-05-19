#include "msp.h"
#include "bsp.h"
#include "interrupts.h"

/* Private Globals Variables*/
static uint32_t volatile g_tick_count = 0;

/* Private Functions Prototypes */
static void update_global_tick_count(void);

/* Private Functions */
static void update_global_tick_count(void){
	g_tick_count++;
}

/* Public Functions */
void delay_ms(uint32_t ms){
	uint32_t volatile start = g_tick_count;
	while((g_tick_count - start) < ms);
}

void init_systick_timer(uint32_t tick_hz){

    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;

	SysTick->LOAD = count_value;
	SysTick->VAL = 0;
	SysTick->CTRL = 7; // enable SysTick interrupt, use system clock

}


/* Interrupt Handlers */
void SysTick_Handler(void){

    update_global_tick_count();

}
