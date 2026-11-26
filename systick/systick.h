#ifndef _systick_def_
#define _systick_def_

#include "stm32f4xx.h"
#include <stdint.h>

void     delay_ms(uint32_t ms);
void     delay_us(uint32_t us);

/* 非阻塞调度：SysTick 全局毫秒计数器 */
void     SysTickTick_Init(void);          /* 启动 1ms SysTick 中断并清零计数 */
uint32_t GetSysTick_ms(void);             /* 读取当前毫秒计数（自启动以来） */

#endif
