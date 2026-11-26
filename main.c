/* ====================================================
   文件名 : main.c
   功能   : 六足机器人主程序 - 蓝牙 / AI 串口指令解析 + 步态调度 + IMU 平衡
   硬件   : STM32F407VE + 2× PCA9685 + JDY-31 蓝牙模块 + AI 语音外设
   接线   : 左板 SCL->PB8  SDA->PB9
            右板 SCL->PB12 SDA->PB13
            蓝牙 USART2  PA2=TX  PA3=RX  115200 8N1
            AI   USART1  PA10=RX（仅接收，不发数据给 AI 模块）波特率见 AI_UART_BAUD
            MPU6050 SCL->PB0  SDA->PB1（独立总线2，不与舵机板共线）
   指令   : 蓝牙或 AI 串口收到蜘蛛指令后解析执行（不带 \r\n 则 500ms 超时成行）
            支持的指令：
              spidergo     -> 前进
              spiderback   -> 后退
              spiderleft   -> 原地左转
              spiderright  -> 原地右转
              spiderstop   -> 停止站立
              spiderone    -> 姿态1（校准站立）
              spidertwo    -> 姿态2（全 90°）
              spiderthree  -> 姿态2 + 陀螺仪自稳平衡（自动保持机身水平）
              spiderfour   -> 姿态4（自定义姿态，中间腿 knee 动画浮动）
              spiderfive   -> 姿态5（胜利姿态：左右晃动 + 前后俯仰）
              spidersix    -> 极致状态（加速档：300°/s，再发 spidergo 灵动快走）
              imucal       -> 重新校准陀螺仪零偏（需静止）
              其他字符串   -> 回发 "unknown cmd\r\n"
   ==================================================== */

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stdio.h"
#include "string.h"   /* strcmp */
#include "pac.h"      /* PCA9685 + 舵机平滑层 */
#include "systick.h"  /* SysTick 计时 */
#include "gait/gait.h" /* 步态调度 + 舵机平滑 */
#include "attitude/attitude.h"  /* 姿态切换 */
#include "bluetooth.h"
#include "mpu6050.h"   /* MPU6050 (GY-521) IMU */
#include "usart.h"     /* USART1（AI 语音外设，仅接收） */
#include "balance/balance.h"  /* spiderthree 陀螺仪自稳平衡 */

/* ---------- 参数 ---------- */
#define CALIB_TARGET_DEG     90u     /* 校准时所有舵机归位到 90° */
#define SERVO_TICK_MS       20u      /* 平滑层步进周期 20ms = 50Hz，MG90S 标准 */

/* AI 语音外设串口波特率（有些模块是 9600，按实际改） */
#define AI_UART_BAUD     115200u

/* ---------- 函数声明 ---------- */
void SystemClock_Config(void);
static const char *Spider_Dispatch(const char *cmd);
static uint8_t Append_UInt(char *buf, uint8_t pos, uint32_t v);

/* ---------- 主函数 ---------- */
int main(void)
{
    uint32_t last_step_tick;
    uint32_t now;
    uint32_t last_hb;
    uint32_t last_rx_count = 0;   /* 上次蓝牙 RX 字节计数（诊断用） */
    uint32_t last_ai_count = 0;   /* AI 串口 RX 字节计数（诊断用） */
    uint32_t cur = 0;
    uint8_t  dp;
    char     cmd[32];
    char     diag_buf[48];

    /* 1. 系统时钟 + SysTick 初始化 */
    SystemClock_Config();
    SysTickTick_Init();

    /* 2. 蓝牙初始化（USART2 / PA2 / PA3 / 115200 8N1，中断接收） */
    Bluetooth_Init();
    Bluetooth_SendString("\r\n[Boot] Bluetooth ready\r\n");

    /* 2b. AI 语音外设串口初始化（USART1 / PA10=RX，仅接收，中断接收） */
    USART1_Init(AI_UART_BAUD);
    Bluetooth_SendString("[Boot] AI UART1 ready (RX only)\r\n");

    /* 3. PCA9685 初始化：两块板各 50Hz，上电归位到 90° */
    PCA9685_Init(IIC_BUS0, 50.0f, CALIB_TARGET_DEG);   /* 左板 PB8/PB9  */
    PCA9685_Init(IIC_BUS1, 50.0f, CALIB_TARGET_DEG);   /* 右板 PB12/PB13 */

    /* 3b. MPU6050 (GY-521)：初始化 + 陀螺仪零偏校准
     *     校准期间需保持静止，耗时约 150ms */
    if (MPU6050_Init())
    {
        MPU6050_Calibrate(MPU6050_CALIB_SAMPLES);
        Bluetooth_SendString("[Boot] MPU6050 ok, zero-bias calibrated\r\n");
    }
    else
    {
        Bluetooth_SendString("[Boot] MPU6050 NOT FOUND (check SCL/SDA/VCC/GND)\r\n");
    }

    /* 4. 步态初始化 + 初始姿态设置
     *    Gait_Init 内部 Apply_Stand 会把 tgt 设到状态1 基准角，
     *    Attitude_ApplyState2 把 tgt 覆盖为全 90°（cur==tgt==90），
     *    这样 Servo_Step 不会产生从 90° 到状态1 的过渡。
     *    要用状态1 启动时把 ApplyState2 改成 ApplyState1。 */
    ServoSmooth_Init(CALIB_TARGET_DEG, GAIT_SPEED_DEFAULT);
    Gait_Init();
    Attitude_ApplyState2();

    Bluetooth_SendString("[Boot] Attitude 2 ready (all 90)\r\n");
    Bluetooth_SendString("       cmds: spidergo / spiderback / spiderleft / spiderright / spiderstop\r\n");
    Bluetooth_SendString("              spiderone (calib stand) / spidertwo (all 90)\r\n");
    Bluetooth_SendString("              spiderthree (attitude2 + IMU auto-level)\r\n");
    Bluetooth_SendString("              spiderfour (custom pose + knee animation)\r\n");
    Bluetooth_SendString("              spiderfive (victory sway: left-right + front-back)\r\n");
    Bluetooth_SendString("              spidersix (EXTREME: 2x speed, then spidergo to run)\r\n");
    Bluetooth_SendString("              imucal (recalibrate gyro, keep still)\r\n");

    last_step_tick = GetSysTick_ms();
    last_hb = last_step_tick;

    /* 5. 主循环 */
    while (1)
    {
        /* 蓝牙/AI 串口超时检查：收到的数据不带 \r\n 则 500ms 超时成行 */
        Bluetooth_TimeoutCheck();
        USART1_TimeoutCheck();

        /* 每 20ms 推进一步：Gait_Tick 驱动步态 + Servo_Step 写 PWM */
        now = GetSysTick_ms();
        if ((now - last_step_tick) >= SERVO_TICK_MS)
        {
            last_step_tick += SERVO_TICK_MS;
            Gait_Tick();             /* 推进步态相位 */
            Attitude_TickState4();   /* 状态4动画：中间腿 knee 上下浮动 */
            Attitude_TickState5();   /* 状态5动画：左右晃动 + 前后俯仰 */
        }

        /* 平衡模式（spiderthree）：状态机 + 闭环调腿保持机身水平 */
        Balance_Tick();

        /* 步态后修正 */
        Gait_ImuCorrect();

        /* 蓝牙命令解析：收到完整行后交给 Spider_Dispatch 执行 */
        if (Bluetooth_GetLine(cmd, (uint8_t)sizeof(cmd)))
        {
            Bluetooth_SendString(Spider_Dispatch(cmd));
        }

        /* AI 串口1 命令解析：收到完整行后回显原始字符串再执行
         *    回显通过 Spider_Dispatch 内部统一处理，蓝牙端可看到
         *    AI 串口1 实际收到的命令内容，便于确认语音识别结果 */
        if (USART1_GetLine(cmd, (uint8_t)sizeof(cmd)))
        {
            Bluetooth_SendString("[AI rx] ");
            Bluetooth_SendString(cmd);
            Bluetooth_SendString("\r\n");
            Bluetooth_SendString(Spider_Dispatch(cmd));
        }

        /* 每 1 秒诊断：汇报蓝牙和 AI 串口的 RX 字节计数
         *    - count 不变 = 没收到数据（检查接线/波特率/配置）
         *    - count 变化 = 收到了数据但可能未成行（检查结束符） */
        if ((now - last_hb) >= 1000u)
        {
            uint32_t ai;
            last_hb += 1000u;
            cur = Bluetooth_GetRxCount();
            ai  = USART1_GetRxCount();
            if (cur != last_rx_count || ai != last_ai_count)
            {
                last_rx_count = cur;
                last_ai_count = ai;

                dp = 0;
                diag_buf[dp++] = '['; diag_buf[dp++] = 'D'; diag_buf[dp++] = 'i';
                diag_buf[dp++] = 'a'; diag_buf[dp++] = 'g'; diag_buf[dp++] = ']';
                diag_buf[dp++] = ' '; diag_buf[dp++] = 'b'; diag_buf[dp++] = 't';
                diag_buf[dp++] = ' '; diag_buf[dp++] = 'r'; diag_buf[dp++] = 'x';
                diag_buf[dp++] = '=';
                dp = Append_UInt(diag_buf, dp, cur);
                diag_buf[dp++] = ' '; diag_buf[dp++] = 'a'; diag_buf[dp++] = 'i';
                diag_buf[dp++] = ' '; diag_buf[dp++] = 'r'; diag_buf[dp++] = 'x';
                diag_buf[dp++] = '=';
                dp = Append_UInt(diag_buf, dp, ai);
                diag_buf[dp] = 0;

                Bluetooth_SendString(diag_buf);
                Bluetooth_SendString("\r\n");
            }
        }
    }
}

/* ==========================================================
 *  命令分发函数：蓝牙（USART2）或 AI 串口（USART1）收到
 *  spider xxx 指令后调用，执行步态/姿态切换并返回应答字符串。
 *  发任何 spider 指令（除 spiderthree）会自动停止平衡模式。
 *
 *  状态6 (spidersix) = 极致状态：加速档 toggle 开关
 *    第一次发 spidersix  -> 进入极致（调高舵机速度到 ATT6_SPEED_Q1）
 *    第二次发 spidersix  -> 退出极致（恢复默认速度）
 *    其他 spider 指令（spidergo/back/left/right/stop/one/two...）
 *      不动速度，保持当前档位（极致或默认），可以任意走路。
 *  极致档与 spiderthree/spiderfour/spiderfive 互斥：
 *    进入这三个状态时会强制退出极致档（恢复默认速度），
 *    因为平衡模式自带有自己的慢速 BAL_SPEED_Q1，
 *    状态4/5 是动画姿态不是步态，速度档无意义。
 * ========================================================== */
#define ATT6_SPEED_Q1   60u      /* 极致状态舵机速度 0.1°/20ms = 300°/s
                                   * 默认 28 = 140°/s，60 = 300°/s（约2倍）
                                   * 不计舵机寿命的激进值，可继续往上调 */

/* 极致档状态：0=正常速度，1=极致速度。toggle 开关 */
static uint8_t s_extreme_on = 0;

static const char *Spider_Dispatch(const char *cmd)
{
    /* 收到任何 spider 指令（除 spiderthree）时：
     *   1. 停止平衡模式
     *   2. 停止状态4/5动画
     * spiderthree 自己管 Balance，不进这个分支。
     * spidersix 进这里会停 Balance + State4/5，但不动 s_extreme_on
     * 标志（toggle 由 spidersix 自己处理） */
    if (strncmp(cmd, "spider", 6) == 0 &&
        strcmp(cmd, "spiderthree") != 0)
    {
        if (Balance_Active()) Bluetooth_SendString(Balance_Stop());
        Attitude_StopState4Anim();   /* 切换模式时停止状态4动画 */
        Attitude_StopState5Anim();   /* 切换模式时停止状态5动画 */
        /* spiderone/two/three/four/five 进入会切姿态/平衡，
         * 极致档（加速走路用）无意义，强制退出恢复默认速度 */
        if (s_extreme_on &&
            (strcmp(cmd, "spiderone")   == 0 ||
             strcmp(cmd, "spidertwo")   == 0 ||
             strcmp(cmd, "spiderfour")  == 0 ||
             strcmp(cmd, "spiderfive")  == 0))
        {
            s_extreme_on = 0;
            Gait_SetSpeed(GAIT_SPEED_DEFAULT);
        }
    }

    if (strcmp(cmd, "spidergo") == 0)
    {
        Gait_Forward();
        return "[Cmd] FORWARD\r\n";
    }
    else if (strcmp(cmd, "spiderback") == 0)
    {
        Gait_Backward();
        return "[Cmd] BACKWARD\r\n";
    }
    else if (strcmp(cmd, "spiderleft") == 0)
    {
        Gait_TurnLeft();
        return "[Cmd] TURN LEFT\r\n";
    }
    else if (strcmp(cmd, "spiderright") == 0)
    {
        Gait_TurnRight();
        return "[Cmd] TURN RIGHT\r\n";
    }
    else if (strcmp(cmd, "spiderstop") == 0)
    {
        Gait_Stop();
        return "[Cmd] STOP\r\n";
    }
    else if (strcmp(cmd, "spiderone") == 0)
    {
        Gait_UseSet1();          /* 切步态集到状态1 */
        Gait_Stop();             /* 停步态 + 回站立（用新表） */
        Attitude_ApplyState1();  /* 切姿态到状态1（与 gait 表一致） */
        return "[Cmd] ATTITUDE 1 (gait set 1)\r\n";
    }
    else if (strcmp(cmd, "spidertwo") == 0)
    {
        Gait_UseSet2();          /* 切步态集到状态2 */
        Gait_Stop();             /* 停步态 + 回站立（用新表） */
        Attitude_ApplyState2();  /* 切姿态到状态2 */
        return "[Cmd] ATTITUDE 2 (gait set 2)\r\n";
    }
    else if (strcmp(cmd, "spiderfour") == 0)
    {
        Gait_Stop();             /* 停步态 + 回站立 */
        Attitude_ApplyState4();  /* 切姿态到状态4 + 启动动画 */
        return "[Cmd] ATTITUDE 4 (custom + animation)\r\n";
    }
    else if (strcmp(cmd, "spiderfive") == 0)
    {
        Gait_Stop();             /* 停步态 + 回站立 */
        Attitude_ApplyState5();  /* 切姿态到状态5(胜利姿态) + 启动动画 */
        return "[Cmd] ATTITUDE 5 (victory sway)\r\n";
    }
    else if (strcmp(cmd, "spidersix") == 0)
    {
        /* 极致状态 toggle 开关：不切姿态、不停步态，只切速度档。
         * 进入：调高到 ATT6_SPEED_Q1 (300°/s)
         * 退出：恢复 GAIT_SPEED_DEFAULT (140°/s)
         * 进入极致档后，spidergo/back/left/right 都以快速度执行，
         * 速度保持直到再次发 spidersix 或进入姿态1/2/3/4/5 */
        if (s_extreme_on)
        {
            s_extreme_on = 0;
            Gait_SetSpeed(GAIT_SPEED_DEFAULT);
            return "[Cmd] EXTREME OFF (140deg/s)\r\n";
        }
        else
        {
            s_extreme_on = 1;
            Gait_SetSpeed(ATT6_SPEED_Q1);
            return "[Cmd] EXTREME ON (300deg/s, send spidergo to run)\r\n";
        }
    }
    else if (strcmp(cmd, "spiderthree") == 0)
    {
        return Balance_Start();  /* 姿态2 -> 采基准 -> 闭环平衡 */
    }

    else if (strcmp(cmd, "imucal") == 0)
    {
        MPU6050_Calibrate(MPU6050_CALIB_SAMPLES);
        return "[Cmd] IMU recalibrated (keep still)\r\n";
    }

    return "[Cmd] unknown cmd\r\n";
}

/* 辅助函数：把无符号整数追加到 buf 的 pos 位置，替代 sprintf */
static uint8_t Append_UInt(char *buf, uint8_t pos, uint32_t v)
{
    char tmp[10];
    uint8_t n = 0;

    if (v == 0)
    {
        buf[pos++] = '0';
        return pos;
    }
    while (v)
    {
        tmp[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    }
    while (n)
    {
        buf[pos++] = tmp[--n];
    }
    return pos;
}

/**
  * @brief  系统时钟配置
  */
void SystemClock_Config(void)
{
    SystemInit();
}
