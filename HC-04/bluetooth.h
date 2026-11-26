#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "stm32f4xx.h"
#include <stdint.h>

/*
 * JDY-31 蓝牙模块驱动（SPP 透传 + 行缓冲）
 *   串口: USART2  PA2=TX  PA3=RX
 *   规格: 115200 8N1
 *   用法: Bluetooth_Init() 初始化后，主循环里调用
 *         Bluetooth_GetLine(buf, sizeof(buf)) 取出一条完整命令
 *         （以 \r 或 \n 结尾的一行）。收到什么发什么由调用方决定。
 */

/* 初始化 USART2 + RXNE 中断（中断里把字节缓冲进行缓冲区） */
void Bluetooth_Init(void);

/* 取出一条完整命令行。返回 1=收到一行，0=暂无 */
uint8_t Bluetooth_GetLine(char *out, uint8_t maxlen);

/* 空闲超时检查：缓冲区有数据但超过 500ms 没收到新字节，
 * 强制视为一行结束（用于手机端不发 \r\n 的场景）。
 * 主循环里每次调用一次。 */
void Bluetooth_TimeoutCheck(void);

/* 诊断：累计收到字节总数（主循环每秒上报可判断 RX 是否工作） */
uint32_t Bluetooth_GetRxCount(void);

/* 主动发送 */
void Bluetooth_SendByte(uint8_t ch);
void Bluetooth_SendString(const char *str);

#endif /* __BLUETOOTH_H */
