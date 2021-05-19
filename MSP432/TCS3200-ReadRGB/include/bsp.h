#ifndef BSP_H_
#define BSP_H_

#include <stdint.h>

#define TICK_HZ 1000U

#define SYSTICK_TIM_CLK   		SystemCoreClock

#ifdef __cplusplus
extern "C" {
#endif

void delay_ms(uint32_t ms);
void init_systick_timer(uint32_t tick_hz);

#ifdef __cplusplus
}
#endif

#endif /* BSP_H_ */
