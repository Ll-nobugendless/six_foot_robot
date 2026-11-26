#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f4xx.h"
#include <stdint.h>
#include "iic.h"   /* IIC_BUS0 / IIC_BUS1 总线号 */

/* ============================================================
 *  GY-521 (MPU-6050) 六轴传感器驱动 —— 基础调用
 * ------------------------------------------------------------
 *  接线（独立 I2C 总线2，不与舵机板并联）：
 *    SCL -> PB0    SDA -> PB1    VCC -> 3.3V    GND -> GND
 *  注意：AD0 悬空/接地时器件地址为 0x68（GY-521 默认）。
 *
 *  用途：
 *    - balance.c 读取加速度做机身水平闭环（spiderthree）
 *    - imucal 命令可重新校准陀螺仪零偏（静止时执行）
 *  启动时会打印一次探测/自检结果（PB0/PB1 电平、芯片型号），
 *  之后不再周期性输出（回显已取消）。
 * ============================================================ */

/* ---- 器件地址（7 位 0x68，左移一位后写入总线） ---- */
#define MPU6050_ADDR_W    0xD0u   /* 写地址 (0x68<<1) */
#define MPU6050_ADDR_R    0xD1u   /* 读地址 */

/* ---- 使用哪条软件 I2C 总线（与 PCA9685 共用） ---- */
#define MPU6050_IIC_BUS   IIC_BUS2

/* ---- 校准/查询 ---- */
#define MPU6050_CALIB_SAMPLES 50u    /* 上电校准采样次数（机身需静止） */

/* ---- 量程配置（默认 ±2000dps / ±2g） ---- */
#define MPU6050_SMPLRT_DIV    9u     /* 采样率 = 1kHz/(1+9) = 100Hz */
#define MPU6050_DLPF_CFG      0x03u  /* CONFIG: 44Hz 低通 */
#define MPU6050_GYRO_FS_SEL   0x18u  /* GYRO_CONFIG: ±2000dps, 16.4 LSB/(°/s) */
#define MPU6050_ACCEL_AFS_SEL 0x00u  /* ACCEL_CONFIG: ±2g, 16384 LSB/g */

/* 一组完整传感器数据（原始值） */
typedef struct
{
    int16_t ax, ay, az;   /* 加速度：±2g 档 16384 = 1g */
    int16_t gx, gy, gz;   /* 陀螺仪：±2000dps 档 16.4 = 1°/s */
    int16_t temp;         /* 温度原始值：T(°C) = raw/340 + 36.53 */
} MPU6050_Data_t;

/* ---- 基础调用 ---- */
uint8_t MPU6050_Init(void);                    /* 初始化，返回 1=成功 0=失败 */
uint8_t MPU6050_Ready(void);                   /* 1=初始化成功可用 0=未就绪 */
void    MPU6050_Read_All(MPU6050_Data_t *d);   /* 读原始值（14 字节一次突发读） */
void    MPU6050_Calibrate(uint16_t samples);   /* 静止时采样求陀螺仪零偏 */

#endif /* __MPU6050_H */
