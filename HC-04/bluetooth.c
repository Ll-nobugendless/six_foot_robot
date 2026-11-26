#include "bluetooth.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "misc.h"
#include "systick.h"  /* GetSysTick_ms */

/* JDY-31 蓝牙模块配置：USART2 / PA2(TX) / PA3(RX) / 115200 8N1 */

#define BT_RX_BUF_SIZE   32   /* 一条命令最多 31 字符 + \0 */
#define BT_LINE_TIMEOUT  500u /* 缓冲区空闲超过 500ms 视为行结束（ms） */

/* 行缓冲：中断里把字节往里塞，遇到 \r / \n 结束一行。
 * - rx_line_ready = 1 时表示已收到完整一行，主循环可以取走；
 * - 主循环取走前，中断里继续收到的字符会被丢弃（避免覆盖）。 */
static volatile uint8_t rx_buf[BT_RX_BUF_SIZE];
static volatile uint8_t rx_len       = 0;
static volatile uint8_t rx_line_ready = 0;
static volatile uint32_t rx_last_byte_tick = 0;  /* 上次收到字节的 SysTick */

/* 诊断计数器：累计收到的字节总数（主循环每秒上报一次） */
static volatile uint32_t rx_byte_count = 0;

/* 取出累计收到字节总数（用于主循环诊断上报） */
uint32_t Bluetooth_GetRxCount(void)
{
    return rx_byte_count;
}

/* 空闲超时检查：缓冲区有数据且超过 BT_LINE_TIMEOUT ms 没收到新字节
 * 就强制视为一行结束。这样即使手机端不发 \r\n 也能拿到命令。 */
void Bluetooth_TimeoutCheck(void)
{
    uint32_t now;

    /* 已经有就绪行 或 缓冲区空：不需要处理 */
    if (rx_line_ready) return;
    if (rx_len == 0)   return;

    now = GetSysTick_ms();
    if ((now - rx_last_byte_tick) >= BT_LINE_TIMEOUT)
    {
        /* 超时且没有 \r\n，强制结束当前行 */
        rx_buf[rx_len] = 0;
        rx_line_ready = 1;
    }
}

void Bluetooth_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef  NVIC_InitStruct;

    /* 打开 GPIOA / USART2 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* PA2 / PA3 设为复用功能 */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA2 -> USART2_TX,  PA3 -> USART2_RX */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    /* USART2: 115200 8N1, 收发使能 */
    USART_InitStruct.USART_BaudRate            = 115200;
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;
    USART_InitStruct.USART_Parity              = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStruct);

    /* 使能 RXNE 中断：收到一个字节就进一次中断 */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel                   = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    USART_Cmd(USART2, ENABLE);

    /* 清掉开机可能产生的错误标志 */
    (void)USART2->SR;
    (void)USART2->DR;
}

/* 取出一条完整命令行。返回 1=收到一行，0=暂无 */
uint8_t Bluetooth_GetLine(char *out, uint8_t maxlen)
{
    uint8_t i;

    if (!rx_line_ready) return 0;

    /* 拷贝到 out（留一个字节放 \0） */
    for (i = 0; i < rx_len && i < (uint8_t)(maxlen - 1); i++)
    {
        out[i] = (char)rx_buf[i];
    }
    out[i] = 0;

    /* 释放缓冲，准备接收下一行 */
    rx_len       = 0;
    rx_line_ready = 0;
    return 1;
}

/* 只等 TXE（不等 TC），避免和中断相互死锁 */
void Bluetooth_SendByte(uint8_t ch)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) { }
    USART_SendData(USART2, ch);
}

void Bluetooth_SendString(const char *str)
{
    while (*str)
    {
        Bluetooth_SendByte((uint8_t)*str++);
    }
}

/*
 * USART2 中断服务函数：按字节收字符串入行缓冲
 * 收到 \r 或 \n 视为一行结束；上一行还没被取走时丢弃新字符
 *
 * 调试增强：每收到一个字节就立刻从 TX 回发，手机端打字能立即看到。
 * 这样可以独立判断 RX 和 TX 通路是否正常工作。
 */
void USART2_IRQHandler(void)
{
    uint8_t ch;

    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        ch = (uint8_t)USART_ReceiveData(USART2);

        /* 计数器累加，供主循环诊断 */
        rx_byte_count++;
        rx_last_byte_tick = GetSysTick_ms();

        /* 立即回发该字节（手机端能实时看到自己输入的字符） */
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) { }
        USART_SendData(USART2, ch);

        if (ch == '\r' || ch == '\n')
        {
            /* 行结束：若缓冲非空且上一行已取走，则标记为就绪 */
            if (rx_len > 0 && !rx_line_ready)
            {
                rx_buf[rx_len] = 0;
                rx_line_ready = 1;
            }
            else
            {
                rx_len = 0;   /* 空行或上一行未取，重置 */
            }
        }
        else if (ch >= ' ' && ch < 127)
        {
            /* 可见 ASCII 字符入缓冲 */
            if (!rx_line_ready && rx_len < (BT_RX_BUF_SIZE - 1))
            {
                rx_buf[rx_len++] = ch;
            }
        }
        /* 其他控制字符忽略 */
    }

    /* 清除 ORE / NE / FE / PE：读 SR 再读 DR */
    if (USART2->SR & (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE))
    {
        (void)USART2->SR;
        (void)USART2->DR;
    }
}
