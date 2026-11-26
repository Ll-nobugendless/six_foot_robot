#ifndef __IIC_H
#define __IIC_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include <stdint.h>

/* ============================================================
 *  双路软件 I2C 总线（两片 PCA9685 各用一路，互不干扰）
 * ------------------------------------------------------------
 *  总线0 (IIC_BUS0)：右侧 16 路舵机驱动板（右腿3条）
 *      SCL -> PB8 ， SDA -> PB9     （原接线，勿动）
 *
 *  总线1 (IIC_BUS1)：左侧 16 路舵机驱动板（左腿3条）
 *      SCL -> PB12， SDA -> PB13
 *      （PB12/PB13 位于多数 F407VE 核心板左侧排针，方便走线；
 *        若与你的板卡排针布局不符，直接改下面 4 个宏即可，
 *        任意空闲 GPIO 都能做软件 I2C，备选 PB0/PB1、
 *        PB10/PB11、PC0/PC1；注意两根线都要在同一个 GPIO 端口组）
 *
 *  接线（每片驱动板都一样）：
 *      板上 SCL -> 对应 SCL 引脚
 *      板上 SDA -> 对应 SDA 引脚
 *      板上 VCC -> 3.3V（逻辑供电）
 *      板上 GND -> STM32 GND（必须共地）
 *      板上 V+  -> 外部 5~6V 舵机电源（勿接STM32 5V）
 *      板上 V+旁GND -> 外部电源 GND（与STM32共地）
 * ============================================================ */
#define IIC_BUS0            0u
#define IIC_BUS1            1u
#define IIC_BUS2            2u
#define IIC_BUS_NUM         3u

/* ---- 总线0引脚（右侧板，保持原接线） ---- */
#define IIC0_SCL_PORT       GPIOB
#define IIC0_SCL_PIN        GPIO_Pin_8
#define IIC0_SDA_PORT       GPIOB
#define IIC0_SDA_PIN        GPIO_Pin_9

/* ---- 总线1引脚（左侧板，新接线） ---- */
#define IIC1_SCL_PORT       GPIOB
#define IIC1_SCL_PIN        GPIO_Pin_12
#define IIC1_SDA_PORT       GPIOB
#define IIC1_SDA_PIN        GPIO_Pin_13

/* ---- 总线2引脚（MPU6050 专用，独立总线不并联） ---- */
#define IIC2_SCL_PORT       GPIOB
#define IIC2_SCL_PIN        GPIO_Pin_0
#define IIC2_SDA_PORT       GPIOB
#define IIC2_SDA_PIN        GPIO_Pin_1

/* 对外接口函数（参数 bus 传 IIC_BUS0 或 IIC_BUS1） */
void    IIC_Init(uint8_t bus);
void    IIC_Start(uint8_t bus);
void    IIC_Stop(uint8_t bus);
void    IIC_Send_Byte(uint8_t bus, uint8_t txd);
uint8_t IIC_Read_Byte(uint8_t bus, uint8_t ack);
uint8_t IIC_Wait_Ack(uint8_t bus);
void    IIC_Ack(uint8_t bus);
void    IIC_NAck(uint8_t bus);

#endif
