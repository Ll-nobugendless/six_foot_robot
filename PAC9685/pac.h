#ifndef __PCA9685_H
#define __PCA9685_H

#include "stm32f4xx.h"
#include <stdint.h>
#include "iic.h"      /* 提供 IIC_BUS0 / IIC_BUS1 总线号定义 */

/* PCA9685 地址（7位地址 0x40，左移一位为 0x80） */
#define PCA9685_ADDR     0x80

/* 寄存器地址 */
#define PCA9685_MODE1    0x00
#define PCA9685_MODE2    0x01
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L        0x06
#define LED0_ON_H        0x07
#define LED0_OFF_L       0x08
#define LED0_OFF_H       0x09

/* 平滑舵机控制：每通道状态结构（Q1 定点，单位 0.1°，避免浮点）
 * 两片 PCA9685（左右各一片），共 2 × 16 = 32 个逻辑通道：
 *   逻辑通道 0~15  -> 右侧板 (I2C总线0, PB8/PB9) 片内通道 0~15
 *   逻辑通道 16~31 -> 左侧板 (I2C总线1, PB12/PB13) 片内通道 0~15
 */
#define SERVO_CH_NUM     32          /* 2片 × 16通道 */
#define SERVO_CH_PER_CHIP 16
#define SERVO_ANGLE_MIN  0           /* 最小角度 0°   */
#define SERVO_ANGLE_MAX  1800        /* 最大角度 180.0° (Q1: 180*10) */
#define SERVO_ANGLE_STEP 1           /* 最小步进 0.1°  */

/* 逻辑通道路由宏：高4位=总线号，低4位=片内通道号 */
#define SERVO_BUS(ch)        ((uint8_t)((ch) >> 4))
#define SERVO_CHIP_CH(ch)    ((uint8_t)((ch) & 0x0F))
/* 由"总线号 + 片内通道"拼出逻辑通道，如左腿片内第3路 = SERVO_MAKE_CH(1,3)=19 */
#define SERVO_MAKE_CH(bus, ch) ((uint8_t)(((bus) << 4) | ((ch) & 0x0F)))

/* ==========================================================
 *  舵机 PWM 脉宽标定（MG90S 等 180° 模拟舵机通用）
 *  --------------------------------------------------------
 *  MG90S 数据手册标称：0° = 500?s ，90° = 1500?s ，180° = 2500?s
 *  但各厂家个体有差异，建议按你手里的舵机实测后微调：
 *
 *  常见问题：
 *   - 角度偏小（设 90° 实际 ~45° ）  → 增大 SERVO_PWM_US_MAX 或减小 SERVO_PWM_US_MIN
 *   - 角度偏大（设 180° 爆冲顶死）   → 减小 SERVO_PWM_US_MAX
 *   - 0° 时还没到真正的极限          → 增大 SERVO_PWM_US_MIN (+20?s~+50?s 起步)
 *  建议每次 ±20?s 调，观察后再改。
 * ========================================================== */
#define SERVO_PWM_US_MIN   500UL     /* 0°   对应脉宽 (单位 ?s) */
#define SERVO_PWM_US_MAX   2500UL    /* 180° 对应脉宽 (单位 ?s) */

/* PCA9685 50Hz 时周期 = 20000?s，分辨率 4096 份
 *   每份 = 20000 / 4096 = 4.8828125 ?s
 * 换算系数（整数定点避免浮点）：
 *   OFF_MIN = SERVO_PWM_US_MIN * 4096 / 20000
 *   OFF_MAX = SERVO_PWM_US_MAX * 4096 / 20000
 */

typedef struct
{
    int16_t  cur;      /* 当前角度，Q1 定点（单位 0.1°），范围 0~1800 */
    int16_t  tgt;      /* 目标角度，Q1 定点 */
    int16_t  last_written; /* 上次写硬件 PWM 时的角度，用于减少 I2C 写次数 */
    uint16_t max_step; /* 每次调度允许的最大步进量（Q1，单位 0.1°） */
    uint8_t  busy;     /* 1 = 当前仍在向目标收敛中，0 = 已到达目标 */
} ServoState_t;

/* 底层寄存器/硬件函数（bus: IIC_BUS0=右板, IIC_BUS1=左板）
 * 注：片内通道 num 范围 0~15；上层舵机接口使用 0~31 逻辑通道即可 */
void PCA9685_Init(uint8_t bus, float hz, uint16_t angle);
void PCA9685_Write(uint8_t bus, uint8_t addr, uint8_t data);
uint8_t PCA9685_Read(uint8_t bus, uint8_t addr);
void PCA9685_setPWM(uint8_t bus, uint8_t num, uint32_t on, uint32_t off);
void PCA9685_setFreq(uint8_t bus, float freq);

/* 直接写角度（无平滑，等价旧接口 setAngle） */
void setAngle(uint8_t num, uint16_t angle);

/* ---------- 平滑运动控制接口 ---------- */
/* 初始化所有通道的平滑状态（PCA9685_Init 后调用） */
void ServoSmooth_Init(uint16_t init_angle, uint16_t default_max_step_q1);

/* 设置单通道最大步进（Q1 0.1°/每次调度） */
void Servo_SetMaxStep(uint8_t ch, uint16_t max_step_q1);

/* 设置单通道目标角度（0~180°整数），随后由 Servo_Step() 收敛过去 */
void Servo_SetTarget(uint8_t ch, uint16_t target_angle);

/* 设置单通道目标角度（Q1 0.1°精度） */
void Servo_SetTargetQ1(uint8_t ch, int16_t target_q1);

/* 获取当前角度（0~180°整数） */
uint16_t Servo_GetCurrent(uint8_t ch);

/* 通道是否仍在移动（未到达目标） */
uint8_t  Servo_IsBusy(uint8_t ch);

/* 所有通道是否全部静止（用于序列切换判定） */
uint8_t  Servo_AllIdle(void);

/*
 * @brief  执行一次平滑步进：把 cur 朝 tgt 移动 max_step
 *         若实际产生移动，再调用底层 setAngle 写入 PWM
 * @retval 本轮产生移动的通道数（0 = 全体已到位）
 * @note   调度周期必须固定，推荐 10ms；否则 max_step 对应的 °/s 含义会变。
 */
uint8_t Servo_Step(void);

#endif

