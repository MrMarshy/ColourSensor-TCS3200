#include "Utils.h"
#include "TM4C123GH6PM.h"

void SysTick_Handler(void);

/* Globals Variables*/
static uint32_t volatile g_tick_count = 0;

void delay_ms(uint32_t ms){
	uint32_t volatile start = g_tick_count;
	while((g_tick_count - start) < ms);
}

static void update_global_tick_count(void){
	g_tick_count++;
}


void Crap_Delay(int32_t const delay){
    volatile int32_t i= 0; 
    for(i = delay; i >= 0; i--);
}

// 1/SysClock * delay
void SysTickWait(uint32_t const delay){
    STRELOAD = delay - 1;
    STCURRENT = 0;
    while( (STCTRL & STCOUNT) == 0){} // wait for count flag
}

void SysTickWaitMs(uint32_t const ms_delay){
    uint32_t i;
    for(i = 0; i < ms_delay; i++){
        SysTickWait(16000U); // 62.5ns * 16000 = 1ms
    }
}

// Assuming 16MHz Clock
void SysTickWaitUs(uint32_t const us_delay){
    
    uint32_t i;
    
    for(i = 0; i < us_delay; i++){
        SysTickWait(16U);       // 62.5ns * 16 = 1us
    }
}

void SetupSysTick(uint32_t tick_hz){

    uint32_t count_value = (SYSTICK_TIM_CLK / tick_hz) - 1;

    SysTick->LOAD = count_value;
    SysTick->VAL = 0;
    SysTick->CTRL = 7; /* Enable SysTick Interrupt, use Sys Clock */

}



/* Error Handlers
void assert_failed(char const *file, int line){
    // TBD: Damage Control
    printf("%s:line %d\r\n", file, line);
    NVIC_SystemReset();
}

void xmalloc_failed(size_t nbytes, char const *file, int line){
    printf("\r\n%s:line %d: malloc() of %zu bytes failed\r\n", file, line, nbytes);
    //NVIC_SystemReset(); // This reset was driving me mad so I commented it out. Should be in when not debugging though...
}
*/

void SysTick_Handler(void) {
    update_global_tick_count();
}


long map(long x, long in_min, long in_max, long out_min, long out_max){

	in_max += 1;
	out_max += 1;

	long ret = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

	if((ret < out_min) && (out_min > out_max)){
		return ret;
	}
	else if((ret < out_min) && (out_min < out_max)){
		return out_min;
	}
	else if((ret > out_max) && (out_min > out_max)){
		return ret;
	}
	else if((ret > out_max) && (out_min < out_max)){
		return out_max;
	}
	else{
		return ret;
	}

}