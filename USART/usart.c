#include "usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "misc.h"
#include "systick.h"   /* GetSysTick_ms */

/* ============================================================
 * USART1 纯接收驱动（接 AI 语音外设）
 *   PA10 = RX <- AI 模块 TXD（PA9/TX 不配置、不使用）
 * 发送器关闭：USART_Mode_Rx，不提供任何发送函数。
 * 中断里按字节收进行缓冲，遇到 \r / \n 或空闲 500ms 视为一行，
 * 主循环用 USART1_GetLine() 取走完整命令（如 "spidergo"）。
 * ============================================================ */

#define USART1_RX_BUF_SIZE  32    /* 一条命令最多 31 字符 + \0 */
#define USART1_LINE_TIMEOUT 500u  /* 缓冲区空闲超过 500ms 视为行结束 */

/* 行缓冲：中断里把字节往里塞，遇到 \r / \n 结束一行。
 * - rx_line_ready = 1 表示已收到完整一行，主循环可以取走；
 * - 主循环取走前，中断里继续收到的字符会被丢弃（避免覆盖）。 */
static volatile uint8_t  rx_buf[USART1_RX_BUF_SIZE];
static volatile uint8_t  rx_len          = 0;
static volatile uint8_t  rx_line_ready   = 0;
static volatile uint32_t rx_last_byte_tick = 0;  /* 上次收到字节的 SysTick */

/* 诊断计数器：累计收到的字节总数 */
static volatile uint32_t rx_byte_count = 0;

/* 取出累计收到字节总数（用于主循环诊断上报） */
uint32_t USART1_GetRxCount(void)
{
    return rx_byte_count;
}

/* 空闲超时检查：缓冲区有数据且超过 USART1_LINE_TIMEOUT ms 没收到新字节，
 * 就强制视为一行结束。AI 模块即使不发 \r\n 也能拿到命令。 */
void USART1_TimeoutCheck(void)
{
    uint32_t now;

    /* 已经有就绪行 或 缓冲区空：不需要处理 */
    if (rx_line_ready) return;
    if (rx_len == 0)   return;

    now = GetSysTick_ms();
    if ((now - rx_last_byte_tick) >= USART1_LINE_TIMEOUT)
    {
        /* 超时且没有 \r\n，强制结束当前行 */
        rx_buf[rx_len] = 0;
        rx_line_ready = 1;
    }
}

void USART1_Init(uint32_t baud)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef  NVIC_InitStruct;

    /* 1.使能串口1 / GPIOA 时钟（USART1 在 APB2） */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* 2.仅 PA10 复用映射为 USART1_RX（PA9/TX 不使用，保持默认） */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    /* 3.配置 PA10：复用功能，上拉保证空闲为高电平 */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 4.串口初始化：8N1，仅接收模式，无硬件流控 */
    USART_InitStruct.USART_BaudRate            = baud;
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;
    USART_InitStruct.USART_Parity              = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode                = USART_Mode_Rx;   /* 仅接收 */
    USART_Init(USART1, &USART_InitStruct);

    /* 5.使能接收中断（RXNE：接收数据寄存器不为空） */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* 6.NVIC：打开 USART1 中断通道
     *    抢占优先级 1（与蓝牙 USART2 相同），中断内不调用 FreeRTOS API、
     *    不做阻塞发送，两个串口中断互不影响。 */
    NVIC_InitStruct.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    /* 7.使能串口1（TE 发送器保持关闭，只有 RE 接收器工作） */
    USART_Cmd(USART1, ENABLE);

    /* 清掉开机可能产生的错误标志：读 SR 再读 DR */
    (void)USART1->SR;
    (void)USART1->DR;
}

/* 取出一条完整命令行。返回 1=收到一行，0=暂无 */
uint8_t USART1_GetLine(char *out, uint8_t maxlen)
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
    rx_len        = 0;
    rx_line_ready = 0;
    return 1;
}

/*
 * USART1 中断服务函数：按字节收字符串入行缓冲
 * 收到 \r 或 \n 视为一行结束；上一行还没被取走时丢弃新字符。
 * 纯接收：中断里不发送任何数据，只收字节 + 拼行 + 清错误标志。
 */
void USART1_IRQHandler(void)
{
    uint8_t ch;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        ch = (uint8_t)USART_ReceiveData(USART1);

        /* 计数器累加，供主循环诊断 */
        rx_byte_count++;
        rx_last_byte_tick = GetSysTick_ms();

        if (ch == '\r' || ch == '\n')
        {
            /* 行结束：缓冲非空且上一行已取走，则标记为就绪 */
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
            if (!rx_line_ready && rx_len < (USART1_RX_BUF_SIZE - 1))
            {
                rx_buf[rx_len++] = ch;
            }
        }
        /* 其他控制字符忽略 */
    }

    /* 清除 ORE / NE / FE / PE：读 SR 再读 DR，否则 RX 会卡死 */
    if (USART1->SR & (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE))
    {
        (void)USART1->SR;
        (void)USART1->DR;
    }
}
