#include "pac.h"
#include "iic.h"
#include "systick.h"
#include <math.h>

/* ================================================
 *            每通道平滑运动状态表
 * ================================================ */
static ServoState_t g_servo[SERVO_CH_NUM];

/*
 * 角度 -> PCA9685 OFF 计数换算
 * -----------------------------------------------------------
 * 公式推导（MG90S 180° 舵机 + PCA9685 @ 50Hz 4096 分辨率）：
 *   周期 T = 20ms = 20000?s
 *   每分辨率计数 = T / 4096 = 4.8828125 ?s
 *   OFF(0°)   = SERVO_PWM_US_MIN / (T / 4096) = US_MIN * 4096 / 20000
 *   OFF(180°) = SERVO_PWM_US_MAX / (T / 4096) = US_MAX * 4096 / 20000
 *
 * 为了 0~180° 之间均匀插值，使用"目标占空比 ?s"直接换算为 OFF 计数：
 *   duty_us(angle_q1) = US_MIN + (US_MAX - US_MIN) * angle_q1 / 1800
 *   OFF = duty_us * 4096 / 20000
 *
 * 合并且整理为单一整数定点公式（每步 1000000 放大避免中间小数）：
 *   OFF = (US_MIN * 4096 * 1000 + (US_MAX - US_MIN) * 4096 * angle_q1 / 1800)
 *         / 20000000
 * 更简单且精度一致的实现：先算 duty_us（整数 ?s，四舍五入），再 *4096/20000
 *   duty_us = US_MIN + ( (US_MAX - US_MIN) * angle_q1 + 900 ) / 1800
 *   OFF     = ( duty_us * 4096 + 10000 ) / 20000
 * -----------------------------------------------------------
 * MG90S 标准值(US_MIN=500, US_MAX=2500)反推：
 *   0°   -> duty_us=500   OFF=102
 *   90°  -> duty_us=1500  OFF=307
 *   180° -> duty_us=2500  OFF=512
 * 即 1° ≈ 1.139 ticks，之前用 1.135/° 偏小 ~0.3%，叠加 Q1 系数时乘了 0.1
 * 导致旧版 90° 仅 OFF=205 (~990?s)，物理约 44°，与你实测 45° 吻合 ?
 * ================================================ */
#define US_MIN   SERVO_PWM_US_MIN
#define US_MAX   SERVO_PWM_US_MAX
#define US_RANGE ((US_MAX) - (US_MIN))

/* Q1 限幅宏 */
#define CLAMP_Q1(x)  do { if ((x) < SERVO_ANGLE_MIN) (x) = SERVO_ANGLE_MIN; \
                          if ((x) > SERVO_ANGLE_MAX) (x) = SERVO_ANGLE_MAX; } while(0)

/* 向指定总线上的 PCA9685 写一个字节 */
void PCA9685_Write(uint8_t bus, uint8_t addr, uint8_t data)
{
    IIC_Start(bus);
    IIC_Send_Byte(bus, PCA9685_ADDR);
    IIC_Wait_Ack(bus);
    IIC_Send_Byte(bus, addr);
    IIC_Wait_Ack(bus);
    IIC_Send_Byte(bus, data);
    IIC_Wait_Ack(bus);
    IIC_Stop(bus);
}

/* 从指定总线上的 PCA9685 读一个字节 */
uint8_t PCA9685_Read(uint8_t bus, uint8_t addr)
{
    uint8_t data;
    IIC_Start(bus);
    IIC_Send_Byte(bus, PCA9685_ADDR);
    IIC_Wait_Ack(bus);
    IIC_Send_Byte(bus, addr);
    IIC_Wait_Ack(bus);
    IIC_Stop(bus);
    delay_us(10);
    IIC_Start(bus);
    IIC_Send_Byte(bus, PCA9685_ADDR | 0x01);  /* 读模式 */
    IIC_Wait_Ack(bus);
    data = IIC_Read_Byte(bus, 0);  /* 非应答 */
    IIC_Stop(bus);
    return data;
}

/* 设置某片板上某一通道的 PWM 占空比（ON 和 OFF 计数值）
 * bus: 总线号(IIC_BUS0/IIC_BUS1)，num: 片内通道 0~15 */
void PCA9685_setPWM(uint8_t bus, uint8_t num, uint32_t on, uint32_t off)
{
    uint8_t reg = LED0_ON_L + 4 * (num & 0x0F);
    IIC_Start(bus);
    IIC_Send_Byte(bus, PCA9685_ADDR);
    IIC_Wait_Ack(bus);
    IIC_Send_Byte(bus, reg);
    IIC_Wait_Ack(bus);
    IIC_Send_Byte(bus, on & 0xFF);
    IIC_Wait_Ack(bus);
    IIC_Send_Byte(bus, (on >> 8) & 0xFF);
    IIC_Wait_Ack(bus);
    IIC_Send_Byte(bus, off & 0xFF);
    IIC_Wait_Ack(bus);
    IIC_Send_Byte(bus, (off >> 8) & 0xFF);
    IIC_Wait_Ack(bus);
    IIC_Stop(bus);
}

/* 设置指定板的 PWM 频率（单位 Hz），通常舵机用 50Hz */
void PCA9685_setFreq(uint8_t bus, float freq)
{
    uint8_t prescale, oldmode, newmode;
    double prescaleval;

    /* 修正系数，使频率更准确 */
    freq *= 0.98f;
    prescaleval = 25000000.0;   /* 内部振荡器 25MHz */
    prescaleval /= 4096.0;
    prescaleval /= freq;
    prescaleval -= 1.0;
    prescale = (uint8_t)floor(prescaleval + 0.5);

    oldmode = PCA9685_Read(bus, PCA9685_MODE1);
    newmode = (oldmode & 0x7F) | 0x10;  /* 进入 sleep 模式 */
    PCA9685_Write(bus, PCA9685_MODE1, newmode);
    PCA9685_Write(bus, PCA9685_PRESCALE, prescale);
    PCA9685_Write(bus, PCA9685_MODE1, oldmode);
    delay_ms(5);
    PCA9685_Write(bus, PCA9685_MODE1, oldmode | 0xA1);  /* 开启自动递增和输出使能 */
}

/* 内部：Q1 定点角度(0.1°) -> OFF 计数（按 MG90S 物理脉宽线性插值） */
static uint32_t angle_q1_to_off(int16_t angle_q1)
{
    uint32_t duty_us;
    if (angle_q1 < 0) angle_q1 = 0;
    if (angle_q1 > SERVO_ANGLE_MAX) angle_q1 = SERVO_ANGLE_MAX;

    /* 1) duty_us = US_MIN + (US_RANGE) * angle_q1 / 1800   （四舍五入 + 900 / 1800） */
    duty_us = US_MIN + ( (uint32_t)US_RANGE * (uint32_t)angle_q1 + 900UL ) / 1800UL;

    /* 2) OFF = duty_us * 4096 / 20000   （四舍五入 + 10000 / 20000） */
    return ( duty_us * 4096UL + 10000UL ) / 20000UL;
}

/* 初始化指定总线上的 PCA9685，设置频率，并将该片所有通道置为初始角度 */
void PCA9685_Init(uint8_t bus, float hz, uint16_t angle)
{
    uint32_t off;
    uint8_t  i;
    int16_t  q1;

    IIC_Init(bus);                          /* 初始化该路 IIC 引脚 */
    PCA9685_Write(bus, PCA9685_MODE1, 0x00);  /* 复位模式寄存器 */
    PCA9685_setFreq(bus, hz);               /* 设置频率 */

    /* 使用统一换算公式 */
    if (angle > 180) angle = 180;
    q1  = (int16_t)(angle * 10);
    off = angle_q1_to_off(q1);

    /* 将该片所有通道设置为相同初始角度 */
    for (i = 0; i < SERVO_CH_PER_CHIP; i++)
    {
        PCA9685_setPWM(bus, i, 0, off);
    }
    delay_ms(100);
}

/* 设置指定逻辑通道的角度（0~180°，整数，兼容旧接口）
 * num: 0~15 右板，16~31 左板（自动路由到对应 I2C 总线） */
void setAngle(uint8_t num, uint16_t angle)
{
    uint32_t off;
    if (num >= SERVO_CH_NUM) return;
    if (angle > 180) angle = 180;
    off = angle_q1_to_off((int16_t)(angle * 10));
    PCA9685_setPWM(SERVO_BUS(num), SERVO_CHIP_CH(num), 0, off);

    /* 同步软件侧当前角，避免平滑层下次从旧值开始插值产生抖动回跳 */
    g_servo[num].cur          = (int16_t)(angle * 10);
    g_servo[num].last_written = g_servo[num].cur;
}

/* ================================================
 *            平滑运动控制接口实现
 * ================================================ */

/* 初始化所有通道的平滑状态（PCA9685_Init 后调用） */
void ServoSmooth_Init(uint16_t init_angle, uint16_t default_max_step_q1)
{
    uint8_t i;
    int16_t q1;
    if (init_angle > 180) init_angle = 180;
    q1 = (int16_t)(init_angle * 10);

    for (i = 0; i < SERVO_CH_NUM; i++)
    {
        g_servo[i].cur          = q1;
        g_servo[i].tgt          = q1;
        g_servo[i].last_written = q1;
        g_servo[i].max_step = default_max_step_q1 ? default_max_step_q1 : 10;
        g_servo[i].busy     = 0;
    }
}

/* 设置单通道最大步进（Q1 0.1°/每次调度） */
void Servo_SetMaxStep(uint8_t ch, uint16_t max_step_q1)
{
    if (ch >= SERVO_CH_NUM) return;
    if (max_step_q1 == 0) max_step_q1 = 1;   /* 至少 0.1°，避免死锁 */
    g_servo[ch].max_step = max_step_q1;
}

/* 设置单通道目标角度（0~180°整数），随后由 Servo_Step() 收敛过去 */
void Servo_SetTarget(uint8_t ch, uint16_t target_angle)
{
    int16_t q1;
    if (ch >= SERVO_CH_NUM) return;
    if (target_angle > 180) target_angle = 180;
    q1 = (int16_t)(target_angle * 10);
    Servo_SetTargetQ1(ch, q1);
}

/* 设置单通道目标角度（Q1 0.1°精度） */
void Servo_SetTargetQ1(uint8_t ch, int16_t target_q1)
{
    if (ch >= SERVO_CH_NUM) return;
    CLAMP_Q1(target_q1);
    g_servo[ch].tgt  = target_q1;
    g_servo[ch].busy = (g_servo[ch].cur != target_q1) ? 1 : 0;
}

/* 获取当前角度（0~180°整数） */
uint16_t Servo_GetCurrent(uint8_t ch)
{
    if (ch >= SERVO_CH_NUM) return 0;
    return (uint16_t)(g_servo[ch].cur / 10);
}

/* 通道是否仍在移动（未到达目标） */
uint8_t Servo_IsBusy(uint8_t ch)
{
    if (ch >= SERVO_CH_NUM) return 0;
    return g_servo[ch].busy;
}

/* 所有通道是否全部静止（用于序列切换判定） */
uint8_t Servo_AllIdle(void)
{
    uint8_t i;
    for (i = 0; i < SERVO_CH_NUM; i++)
    {
        if (g_servo[i].busy) return 0;
    }
    return 1;
}

/*
 * @brief  执行一次平滑步进：每通道 cur 向 tgt 最多移动 max_step
 *         若实际产生移动，通过 I2C 写 PCA9685 PWM
 * @retval 本轮写入硬件的通道数（0 = 全体已到位）
 */
uint8_t Servo_Step(void)
{
    uint8_t  i;
    uint8_t  written = 0;
    int32_t  diff;
    int16_t  step;
    uint32_t off;
    ServoState_t *s;

    for (i = 0; i < SERVO_CH_NUM; i++)
    {
        s = &g_servo[i];
        if (!s->busy) continue;

        diff = (int32_t)s->tgt - (int32_t)s->cur;
        if (diff == 0)
        {
            s->busy = 0;
            continue;
        }

        /* 计算本步移动量 */
        if (diff > 0)
        {
            step = (int16_t)s->max_step;
            if ((int32_t)step > diff) step = (int16_t)diff;
            s->cur += step;
        }
        else
        {
            step = (int16_t)s->max_step;
            if ((int32_t)step > -diff) step = (int16_t)(-diff);
            s->cur -= step;
        }

        /* 死区保护：差值小于 3 (0.3°) 直接到位，避免末端 1LSB 抖动 */
        if ((s->tgt > s->cur) && ((int32_t)(s->tgt - s->cur) <= 3))
            s->cur = s->tgt;
        if ((s->tgt < s->cur) && ((int32_t)(s->cur - s->tgt) <= 3))
            s->cur = s->tgt;

        if (s->cur == s->tgt) s->busy = 0;

        /* 写硬件 PWM 阈值：角度变化超过 5 Q1 (0.5°) 或已到位才写 I2C
         * 避免每步都写 I2C 产生高频抖动，MG90S 物理分辨率约 1~2° */
        if ((s->busy == 0) ||
            (s->cur > s->last_written  && (s->cur - s->last_written) >= 5) ||
            (s->cur < s->last_written  && (s->last_written - s->cur) >= 5))
        {
            off = angle_q1_to_off(s->cur);
            PCA9685_setPWM(SERVO_BUS(i), SERVO_CHIP_CH(i), 0, off);
            s->last_written = s->cur;
            written++;
        }
    }

    return written;
}


