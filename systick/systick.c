#include "systick.h"

/* 全局毫秒计数器（由 SysTick_Handler 自增） */
static volatile uint32_t g_sys_tick_ms = 0;

/*
 * @brief  启动 1ms 周期 SysTick 中断
 * @note   168MHz 内核时钟 / 8 = 21MHz -> 重装值 21000-1 产生 1ms 时基
 *         调用后 g_sys_tick_ms 清零并开始累加
 */
void SysTickTick_Init(void)
{
    g_sys_tick_ms = 0;

    SysTick->CTRL = 0;
    SysTick->LOAD = 21000 - 1;     /* 21MHz / 21000 = 1000Hz -> 1ms */
    SysTick->VAL  = 0;
    /* 使能 + 中断使能 + 时钟源 = HCLK/8 (bit2=0) */
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
}

/*
 * @brief  获取当前毫秒计数
 */
uint32_t GetSysTick_ms(void)
{
    return g_sys_tick_ms;
}

/*
 * @brief  SysTick 中断服务函数：累加全局计数器
 * @note   启动文件里 SysTick_Handler 的 [WEAK] 默认实现是 "B ."（死循环），
 *         必须在这里提供真实实现，否则 SysTick 中断触发后 CPU 会卡死。
 *
 * ============================================================
 *  整合到 FreeRTOS 时必须执行的迁移步骤（现在不用做，记着）：
 *  ------------------------------------------------------------
 *  1) 删除本函数 SysTick_Handler（FreeRTOS 会接管 SysTick 中断）
 *  2) FreeRTOS\include\FreeRTOSConfig.h 末尾取消注释，恢复：
 *         #define xPortSysTickHandler SysTick_Handler
 *     （让 port.c 的 xPortSysTickHandler 经预处理变成 SysTick_Handler，
 *       接管向量表。当前是注释状态，因为本函数已经占用此符号，
 *       否则链接报 L6200E: Symbol SysTick_Handler multiply defined.）
 *  3) 在 FreeRTOSConfig.h 里把 configUSE_TICK_HOOK 设为 1
 *  4) 新增 vApplicationTickHook(void) 函数（任意 .c 文件里都行），
 *     在里面做 g_sys_tick_ms++ 以保留 GetSysTick_ms() 旧接口
 *  5) 删除 main.c 里对 SysTickTick_Init() 的调用
 *     （vTaskStartScheduler() 内部会自己配置 SysTick 寄存器）
 *
 *  备选方案（更简单）：启用 FreeRTOS 后直接把 GetSysTick_ms()
 *  的实现改成 return xTaskGetTickCount(); 不再维护 g_sys_tick_ms
 * ============================================================
 */
void SysTick_Handler(void)
{
    g_sys_tick_ms++;
}

/*
 * @brief  毫秒级阻塞延时（兼容旧接口，不影响 SysTick 计数运行）
 * @note   使用时基计数器实现，无需再开关 SysTick。
 */
void delay_ms(uint32_t ms)
{
    uint32_t start = GetSysTick_ms();
    /* 若 SysTickTick_Init 尚未调用，回退到旧的寄存器轮询方式，保证兼容 */
    if ((SysTick->CTRL & SysTick_CTRL_TICKINT_Msk) == 0)
    {
        while (ms--)
        {
            SysTick->CTRL = 0;
            SysTick->LOAD = 168000 - 1;
            SysTick->VAL  = 0;
            SysTick->CTRL = 5;
            while ((SysTick->CTRL & (1 << 16)) == 0) { }
            SysTick->CTRL = 0;
        }
        return;
    }
    while ((GetSysTick_ms() - start) < ms) { }
}

/*
 * @brief  微秒级阻塞延时（仍使用寄存器轮询，使用完恢复 SysTick）
 */
void delay_us(uint32_t us)
{
    uint32_t ctrl_save = SysTick->CTRL;
    uint32_t load_save = SysTick->LOAD;
    uint32_t val_save  = SysTick->VAL;

    SysTick->CTRL = 0;
    SysTick->LOAD = 168 * us - 1;   /* HCLK/8 = 21MHz -> 168 计数/8us？不对：
                                       21MHz/21 = 1MHz -> 1 计数 = 1/21 us，
                                       此处用 21 计数 = 1us -> 修正公式如下： */
    /* 上面 168*us 是旧代码遗留，实际 21MHz 下 1us = 21 个时钟。
       若 HCLK=168MHz，不做 8 分频（CTRL bit2=1）时 1us = 168 个时钟。
       这里统一用 21MHz（不分频到 8 分频），每 us = 21 tick： */
    SysTick->LOAD = (21 * us) - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = 5;   /* ENABLE=1, TICKINT=0, CLKSOURCE=0 (HCLK/8=21MHz) */
    while ((SysTick->CTRL & (1 << 16)) == 0) { }
    SysTick->CTRL = 0;

    /* 恢复原 SysTick 配置，避免打断 1ms 计数 */
    SysTick->LOAD = load_save;
    SysTick->VAL  = val_save;
    SysTick->CTRL = ctrl_save;
}
