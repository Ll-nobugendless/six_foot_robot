#ifndef _MY_CHUANKOU_H_
#define _MY_CHUANKOU_H_

#include "stm32f4xx.h"
#include <stdio.h>
#include <stdint.h>

/*
 * USART1 串口驱动（接 AI 语音外设）—— 仅接收（RX only）
 *   引脚: PA10 = RX (<- AI 模块 TXD)，PA9(TX) 不使用
 *   规格: 8N1，无硬件流控，波特率由初始化参数指定
 *         （须与 AI 语音模块一致，常见 9600 / 115200）
 *
 * 用法:
 *   USART1_Init(115200);
 *   主循环中:
 *     USART1_TimeoutCheck();                 // 500ms 空闲强制成行
 *     if (USART1_GetLine(buf, sizeof(buf)))  // 取一条完整命令
 *     { ... }
 */

/* 初始化 USART1 接收（RXNE 中断），发送器关闭 */
void USART1_Init(uint32_t baud);

/* 取出一条完整命令行。返回 1=收到一行，0=暂无 */
uint8_t USART1_GetLine(char *out, uint8_t maxlen);

/* 空闲超时检查：缓冲区有数据但超过 500ms 没收到新字节，
 * 强制视为一行结束（AI 模块可能不发 \r\n）。主循环里每次调用一次。 */
void USART1_TimeoutCheck(void);

/* 诊断：累计收到字节总数（主循环定时上报可判断 RX 是否工作） */
uint32_t USART1_GetRxCount(void);

#endif /* _MY_CHUANKOU_H_ */
