#include "iic.h"
#include "systick.h"   /* 提供 delay_us / delay_ms */

/* ============================================================
 *  双路软件 I2C 实现
 *  时序与单路版本完全一致，只是 SCL/SDA 引脚按 bus 查表选择。
 * ============================================================ */

typedef struct
{
    GPIO_TypeDef *scl_port;
    uint16_t      scl_pin;
    GPIO_TypeDef *sda_port;
    uint16_t      sda_pin;
} iic_bus_t;

static const iic_bus_t s_bus[IIC_BUS_NUM] =
{
    { IIC0_SCL_PORT, IIC0_SCL_PIN, IIC0_SDA_PORT, IIC0_SDA_PIN },  /* bus0 右板 */
    { IIC1_SCL_PORT, IIC1_SCL_PIN, IIC1_SDA_PORT, IIC1_SDA_PIN },  /* bus1 左板 */
    { IIC2_SCL_PORT, IIC2_SCL_PIN, IIC2_SDA_PORT, IIC2_SDA_PIN },  /* bus2 MPU6050 */
};

static const iic_bus_t *B(uint8_t bus)
{
    return (bus < IIC_BUS_NUM) ? &s_bus[bus] : &s_bus[0];
}

/* 宏：按总线设置 SCL/SDA 高低电平 */
#define SCL_H(bus)  GPIO_SetBits(B(bus)->scl_port,   B(bus)->scl_pin)
#define SCL_L(bus)  GPIO_ResetBits(B(bus)->scl_port, B(bus)->scl_pin)
#define SDA_H(bus)  GPIO_SetBits(B(bus)->sda_port,   B(bus)->sda_pin)
#define SDA_L(bus)  GPIO_ResetBits(B(bus)->sda_port, B(bus)->sda_pin)
#define SDA_READ(bus) GPIO_ReadInputDataBit(B(bus)->sda_port, B(bus)->sda_pin)

/* 初始化指定总线的 IIC 引脚 —— 开漏输出，无需切换方向 */
void IIC_Init(uint8_t bus)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    const iic_bus_t *p = B(bus);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* SCL 和 SDA 配置为开漏输出：输出高=释放总线(上拉拉高)，输出低=拉低
     * 这样读 SDA 时只需输出高再读输入寄存器，无需切换 GPIO 方向 */
    GPIO_InitStruct.GPIO_Pin   = p->scl_pin | p->sda_pin;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(p->scl_port, &GPIO_InitStruct);

    SCL_H(bus);
    SDA_H(bus);
}

/* 起始信号：SCL高时 SDA 下降沿 */
void IIC_Start(uint8_t bus)
{
    SDA_H(bus);
    SCL_H(bus);
    delay_us(5);
    SDA_L(bus);
    delay_us(5);
    SCL_L(bus);
}

/* 停止信号：SCL高时 SDA 上升沿 */
void IIC_Stop(uint8_t bus)
{
    SCL_L(bus);
    SDA_L(bus);
    delay_us(5);
    SCL_H(bus);
    delay_us(5);
    SDA_H(bus);
    delay_us(5);
}

/* 等待应答，返回 0=应答，1=无应答 */
uint8_t IIC_Wait_Ack(uint8_t bus)
{
    uint8_t ucErrTime = 0;
    SDA_H(bus);          /* 释放 SDA，让从机拉低应答 */
    delay_us(3);
    SCL_H(bus);          /* 第9个时钟脉冲 */
    delay_us(3);
    while (SDA_READ(bus))
    {
        ucErrTime++;
        if (ucErrTime > 250)
        {
            SCL_L(bus);
            IIC_Stop(bus);
            return 1;
        }
    }
    SCL_L(bus);
    return 0;
}

/* 产生应答 */
void IIC_Ack(uint8_t bus)
{
    SCL_L(bus);
    SDA_L(bus);          /* 主机拉低 SDA = ACK */
    delay_us(5);
    SCL_H(bus);
    delay_us(5);
    SCL_L(bus);
    SDA_H(bus);          /* 释放 SDA */
}

/* 产生非应答 */
void IIC_NAck(uint8_t bus)
{
    SCL_L(bus);
    SDA_H(bus);          /* 主机释放 SDA = NACK */
    delay_us(5);
    SCL_H(bus);
    delay_us(5);
    SCL_L(bus);
}

/* 发送一个字节 */
void IIC_Send_Byte(uint8_t bus, uint8_t txd)
{
    uint8_t t;
    SCL_L(bus);
    for (t = 0; t < 8; t++)
    {
        if (txd & 0x80)
            SDA_H(bus);
        else
            SDA_L(bus);
        txd <<= 1;
        delay_us(5);
        SCL_H(bus);
        delay_us(5);
        SCL_L(bus);
        delay_us(5);
    }
}

/* 读取一个字节，ack=1 发送应答，ack=0 发送非应答 */
uint8_t IIC_Read_Byte(uint8_t bus, uint8_t ack)
{
    uint8_t i, receive = 0;
    SDA_H(bus);          /* 释放 SDA，让从机驱动 */
    for (i = 0; i < 8; i++)
    {
        SCL_L(bus);
        delay_us(5);
        SCL_H(bus);
        receive <<= 1;
        if (SDA_READ(bus))
            receive++;
        delay_us(3);
    }
    if (!ack)
        IIC_NAck(bus);
    else
        IIC_Ack(bus);
    return receive;
}
