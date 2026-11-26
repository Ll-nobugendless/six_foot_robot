#include "mpu6050.h"
#include "iic.h"
#include "systick.h"    /* delay_ms / delay_us */
#include "bluetooth.h"  /* USART2 -> JDY-31 蓝牙透传，用于回显 */

/* ============================================================
 *  MPU-6050 (GY-521) 基础调用
 *  寄存器地址对照 RM-MPU-6000A 数据手册：
 *    SMPLRT_DIV=0x19  CONFIG=0x1A  GYRO_CONFIG=0x1B  ACCEL_CONFIG=0x1C
 *    ACCEL_XOUT_H=0x3B  TEMP_OUT_H=0x41  GYRO_XOUT_H=0x43
 *    PWR_MGMT_1=0x6B  WHO_AM_I=0x75（应读回 0x68）
 * ============================================================ */

/* ---- 寄存器地址 ---- */
#define MPU6050_REG_SMPLRT_DIV   0x19u
#define MPU6050_REG_CONFIG       0x1Au
#define MPU6050_REG_GYRO_CONFIG  0x1Bu
#define MPU6050_REG_ACCEL_CONFIG 0x1Cu
#define MPU6050_REG_ACCEL_XOUT_H 0x3Bu
#define MPU6050_REG_TEMP_OUT_H   0x41u
#define MPU6050_REG_GYRO_XOUT_H  0x43u
#define MPU6050_REG_PWR_MGMT_1   0x6Bu
#define MPU6050_REG_WHO_AM_I     0x75u

#define MPU6050_WHO_AM_I_VAL     0x68u   /* GY-521 默认值（AD0=0） */

/* 陀螺仪零偏（上电/手动校准时计算，读数据时扣除） */
static int16_t g_gyro_off[3] = {0, 0, 0};

/* 初始化结果：1=成功 0=失败（Init 里赋值，供 MPU6050_Ready 查询） */
static uint8_t g_init_ok = 0;

/* ---------- 底层 I2C 读写 ---------- */

/* 写一个寄存器，返回 0=成功 1=失败 */
static uint8_t MPU_WriteByte(uint8_t bus, uint8_t reg, uint8_t data)
{
    IIC_Start(bus);
    IIC_Send_Byte(bus, MPU6050_ADDR_W);
    if (IIC_Wait_Ack(bus)) { IIC_Stop(bus); return 1; }
    IIC_Send_Byte(bus, reg);
    if (IIC_Wait_Ack(bus)) { IIC_Stop(bus); return 1; }
    IIC_Send_Byte(bus, data);
    if (IIC_Wait_Ack(bus)) { IIC_Stop(bus); return 1; }
    IIC_Stop(bus);
    return 0;
}

/* 从 reg 起连续读 len 个字节，返回 0=成功 1=失败 */
static uint8_t MPU_ReadBytes(uint8_t bus, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i = 0;

    IIC_Start(bus);
    IIC_Send_Byte(bus, MPU6050_ADDR_W);
    if (IIC_Wait_Ack(bus)) { IIC_Stop(bus); return 1; }
    IIC_Send_Byte(bus, reg);
    if (IIC_Wait_Ack(bus)) { IIC_Stop(bus); return 1; }
    IIC_Start(bus);   /* 重复起始，转为读模式 */
    IIC_Send_Byte(bus, MPU6050_ADDR_R);
    if (IIC_Wait_Ack(bus)) { IIC_Stop(bus); return 1; }
    while (len > 1)
    {
        buf[i++] = IIC_Read_Byte(bus, 1);   /* 中间字节应答 */
        len--;
    }
    buf[i] = IIC_Read_Byte(bus, 0);         /* 最后一字节不应答 */
    IIC_Stop(bus);
    return 0;
}

/* ---------- 初始化 ---------- */
/* 诊断：把错误码通过蓝牙输出，帮助定位 I2C 通信哪一步失败 */
static void init_diag(const char *msg)
{
    char buf[48];
    uint8_t i;
    for (i = 0; i < sizeof(buf)-3 && msg[i]; i++) buf[i] = msg[i];  /* 留 3 字节给 \r\n\0 */
    buf[i] = '\r'; buf[i+1] = '\n'; buf[i+2] = 0;
    Bluetooth_SendString(buf);
}

uint8_t MPU6050_Init(void)
{
    uint8_t probe_regs[] = { 0x00, 0x75 };  /* ICM-20601 WHO在0x00, MPU6050在0x75 */
    uint8_t probe_who[]  = { 0xAC, 0x68 };  /* 分别对应 ICM-20601 和 MPU6050 */
    uint8_t id = 0;
    uint8_t i;

    IIC_Init(MPU6050_IIC_BUS);

    /* ----- 引脚电平自检 ----- */
    {
        GPIO_InitTypeDef g;
        uint8_t pb0_val, pb1_val;
        g.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
        g.GPIO_Mode  = GPIO_Mode_IN;
        g.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_Init(GPIOB, &g);
        pb0_val = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0);
        pb1_val = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1);
        init_diag(pb0_val ? "PB0(SCL)=HIGH OK" : "WARN PB0(SCL)=LOW no pullup/short");
        init_diag(pb1_val ? "PB1(SDA)=HIGH OK" : "WARN PB1(SDA)=LOW no pullup/short");
        IIC_Init(MPU6050_IIC_BUS);
    }

    /* ----- 自动探测芯片类型 ----- */
    {
        uint8_t reg_found = 0;
        Bluetooth_SendString("Probe chip: ");
        for (i = 0; i < sizeof(probe_regs); i++)
        {
            uint8_t ack1, ack2, ack3, val;
            IIC_Start(MPU6050_IIC_BUS);
            IIC_Send_Byte(MPU6050_IIC_BUS, MPU6050_ADDR_W);
            ack1 = IIC_Wait_Ack(MPU6050_IIC_BUS);
            if (ack1) { IIC_Stop(MPU6050_IIC_BUS); continue; }
            IIC_Send_Byte(MPU6050_IIC_BUS, probe_regs[i]);
            ack2 = IIC_Wait_Ack(MPU6050_IIC_BUS);
            if (ack2) { IIC_Stop(MPU6050_IIC_BUS); continue; }
            IIC_Start(MPU6050_IIC_BUS);
            IIC_Send_Byte(MPU6050_IIC_BUS, MPU6050_ADDR_R);
            ack3 = IIC_Wait_Ack(MPU6050_IIC_BUS);
            if (ack3) { IIC_Stop(MPU6050_IIC_BUS); continue; }
            val = IIC_Read_Byte(MPU6050_IIC_BUS, 0);
            IIC_Stop(MPU6050_IIC_BUS);

            /* 输出探测结果 */
            {
                char tb[24]; uint8_t p = 0;
                tb[p++]='r'; tb[p++]='e'; tb[p++]='g'; tb[p++]='0'; tb[p++]='x';
                tb[p++]=(char)((probe_regs[i]>>4)<10?'0'+(probe_regs[i]>>4):'A'+(probe_regs[i]>>4)-10);
                tb[p++]=(char)((probe_regs[i]&0xf)<10?'0'+(probe_regs[i]&0xf):'A'+(probe_regs[i]&0xf)-10);
                tb[p++]='='; tb[p++]='0'; tb[p++]='x';
                tb[p++]=(char)((val>>4)<10?'0'+(val>>4):'A'+(val>>4)-10);
                tb[p++]=(char)((val&0xf)<10?'0'+(val&0xf):'A'+(val&0xf)-10);
                tb[p++]=' '; tb[p]=0;
                Bluetooth_SendString(tb);
            }

            if (val == probe_who[i])
            {
                id = val;
                reg_found = 1;
                break;
            }
        }

        if (!reg_found)
        {
            init_diag("(no known IMU WHO_AM_I match)");
            init_diag("CHECK: SCL->PB0 SDA->PB1 VCC=3.3V GND shared");
            return 0;
        }
    }

    /* 识别芯片型号（仅用于打印，寄存器映射 MPU6050 与 ICM-20601 一致） */
    Bluetooth_SendString(id == 0x68 ? "= MPU-6050\r\n" : "= ICM-20601\r\n");

    /* ----- 初始化寄存器（MPU6050 与 ICM-20601 寄存器地址相同） ----- */
    /* 1. 全寄存器复位 */
    if (MPU_WriteByte(MPU6050_IIC_BUS, MPU6050_REG_PWR_MGMT_1, 0x80))
    {
        init_diag("MPU: write reset failed");
        return 0;
    }
    delay_ms(100);

    /* 2. 退出睡眠，时钟源选 PLL(X 轴陀螺参考) */
    if (MPU_WriteByte(MPU6050_IIC_BUS, MPU6050_REG_PWR_MGMT_1, 0x01))
    {
        init_diag("MPU: write wake failed");
        return 0;
    }
    delay_ms(50);

    /* 3. 采样率 100Hz、44Hz 低通、±2000dps、±2g */
    MPU_WriteByte(MPU6050_IIC_BUS, MPU6050_REG_SMPLRT_DIV,   MPU6050_SMPLRT_DIV);
    MPU_WriteByte(MPU6050_IIC_BUS, MPU6050_REG_CONFIG,       MPU6050_DLPF_CFG);
    MPU_WriteByte(MPU6050_IIC_BUS, MPU6050_REG_GYRO_CONFIG,  MPU6050_GYRO_FS_SEL);
    MPU_WriteByte(MPU6050_IIC_BUS, MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_AFS_SEL);
    delay_ms(10);

    g_init_ok = 1;
    return 1;
}

/* ---------- 读取 ---------- */
void MPU6050_Read_All(MPU6050_Data_t *d)
{
    uint8_t buf[14];

    if (!d) return;
    /* 从 0x3B 一次突发读 14 字节：加速度(6)+温度(2)+陀螺仪(6)，
     * 保证是同一采样时刻的数据 */
    if (MPU_ReadBytes(MPU6050_IIC_BUS, MPU6050_REG_ACCEL_XOUT_H, buf, 14))
    {
        /* 读失败：清零，避免上层拿到旧数据误判 */
        d->ax = d->ay = d->az = 0;
        d->temp = 0;
        d->gx = d->gy = d->gz = 0;
        return;
    }

    d->ax   = (int16_t)((buf[0]  << 8) | buf[1]);
    d->ay   = (int16_t)((buf[2]  << 8) | buf[3]);
    d->az   = (int16_t)((buf[4]  << 8) | buf[5]);
    d->temp = (int16_t)((buf[6]  << 8) | buf[7]);
    d->gx   = (int16_t)((buf[8]  << 8) | buf[9]);
    d->gy   = (int16_t)((buf[10] << 8) | buf[11]);
    d->gz   = (int16_t)((buf[12] << 8) | buf[13]);
}

/* ---------- 校准 ---------- */
/* 静止时连续采样 samples 次，取陀螺仪平均值作为零偏存入内部变量 */
void MPU6050_Calibrate(uint16_t samples)
{
    int32_t sx = 0, sy = 0, sz = 0;
    uint16_t i;
    MPU6050_Data_t d;

    if (samples == 0) samples = 1;
    for (i = 0; i < samples; i++)
    {
        MPU6050_Read_All(&d);
        sx += d.gx; sy += d.gy; sz += d.gz;
        delay_ms(2);   /* 等下一个采样点 */
    }
    g_gyro_off[0] = (int16_t)(sx / (int32_t)samples);
    g_gyro_off[1] = (int16_t)(sy / (int32_t)samples);
    g_gyro_off[2] = (int16_t)(sz / (int32_t)samples);
}

/* ---------- 状态查询 ---------- */
uint8_t MPU6050_Ready(void)
{
    return g_init_ok;
}
