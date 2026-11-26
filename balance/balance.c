/* ====================================================
   文件名 : balance/balance.c
   功能   : spiderthree 陀螺仪自稳平衡（姿态2 站立 + P 控制器）
   ----------------------------------------------------
   原理：
     机器人静站时脚固定在桌面，桌面倾斜 = 机身跟随倾斜。
     要保持机身水平，需改变各条腿的"垂直伸展量"：
       机身哪边低 -> 那边腿伸长顶高，对侧腿缩短放低。
     用加速度计求倾斜角误差，按每条腿的前后(x)/左右(y)
     位置线性分配调整量：
       Δh(i) = Kp * ( x(i)*err_pitch + y(i)*err_roll )
     Δh>0 = 该腿伸长（lift/knee 语义 -Δ），Δh<0 = 缩短。

   语义方向沿用 gait.c：lift +1=抬腿（缩短），knee +1=收腿（缩短），
   伸长即两者同时 -Δ。通道/方向/基准角与 gait.c 的 s_leg2 一致，
   改 s_leg2 时请同步本文件 s_bal_leg。

   调参：
     误差死区 BAL_DEADZONE_RAW 防静止抖动；
     Kp 定点 Q4（16=1.0）；输出限幅 ±6°；
     舵机速度与步态相同（GAIT_SPEED_DEFAULT），便于观察补偿方向对错。

   轴向/方向若与实际安装不符：改 BAL_PITCH_AXIS /
   BAL_ROLL_AXIS / BAL_PITCH_SIGN / BAL_ROLL_SIGN 四个宏即可。
   ==================================================== */

#include "balance.h"
#include "pac.h"        /* Servo_SetTarget / Servo_AllIdle */
#include "mpu6050.h"    /* MPU6050_Read_All */
#include "gait.h"       /* Gait_UseSet2 / Gait_Stop / Gait_SetSpeed / GAIT_SPEED_DEFAULT */
#include "attitude.h"   /* Attitude_ApplyState2 */
#include "bluetooth.h"  /* Bluetooth_SendString（调试行） */
#include "systick.h"    /* delay_ms / GetSysTick_ms */

/* ---------- 可调参数 ---------- */
/* 加速度轴选择：0=ax 1=ay 2=az（IMU 安装朝向不同时改这里） */
#define BAL_PITCH_AXIS   0
#define BAL_ROLL_AXIS    1
/* 方向符号：实测机身前倾/右倾时 err 为负则改成 -1 */
#define BAL_PITCH_SIGN   (+1)
#define BAL_ROLL_SIGN    (+1)

#define BAL_RAW_PER_DEG  286     /* ±2g 档 1° 倾角 ≈ 16384/57.3 raw */
#define BAL_DEADZONE_RAW 2860    /* 误差区间（死区）±10° = 10×286 raw，水平时不动 */
#define BAL_KP_Q4        2       /* P 增益 Q4：4 = 0.25（1° 误差出 0.25° 腿调整） */
#define BAL_OUT_MAX_Q1   60      /* 单腿调整限幅 ±6.0°（Q1 0.1°） */
#define BAL_TICK_MS      200u    /* 闭环周期 200ms（慢一点减少噪声抖动） */
#define BAL_SPEED_Q1     8u      /* 平衡舵机速度 0.5°/20ms = 25°/s（慢速防打齿） */
#define BAL_DBG_MS       1000u   /* 调试行周期 1s */
#define BAL_FILTER_N     4u      /* 滑动平均滤波：连续采 4 次取平均 */

/* ---------- 状态机 ---------- */
enum { BAL_IDLE = 0, BAL_WAIT_SERVO, BAL_RUN };

typedef struct
{
    uint8_t ch_lift;   /* 大腿逻辑通道 */
    uint8_t ch_knee;   /* 膝关节逻辑通道 */
    int8_t  s_lift;    /* 大腿方向系数（同 gait.c s_leg2） */
    int8_t  s_knee;    /* 膝方向系数 */
    uint8_t lift0;     /* 姿态2基准角 */
    uint8_t knee0;
    int8_t  px;        /* 位置 x：+1 前 / 0 中 / -1 后 */
    int8_t  py;        /* 位置 y：+1 右 / -1 左 */
} BalLeg_t;

/* 与 gait.c s_leg2 的 lift/knee 保持一致（姿态2 = spiderthree 初始姿态）。
 * !! 左右按实际机身：idx 0,1,2 = 左腿（py=-1），idx 3,4,5 = 右腿（py=+1） !! */
static const BalLeg_t s_bal_leg[6] =
{
    /* ch_lift ch_knee s_lift s_knee lift0 knee0 px py */
    {   1,      2,      +1,    -1,    90,  100,  +1, -1 },  /* L1 左前 */
    {   5,      6,      -1,    +1,    90,   80,   0, -1 },  /* L2 左中 */
    {   9,     10,      -1,    +1,    90,   70,  -1, -1 },  /* L3 左后 */
    {  17,     18,      +1,    -1,    90,  100,  +1, +1 },  /* R1 右前 */
    {  21,     22,      +1,    -1,    90,  100,   0, +1 },  /* R2 右中 */
    {  25,     26,      -1,    +1,    90,   70,  -1, +1 },  /* R3 右后 */
};

static uint8_t  s_stage     = BAL_IDLE;
static uint32_t s_wait_t0   = 0;    /* 等舵机到位开始时间 */
static uint32_t s_ctrl_t    = 0;    /* 控制节拍 */
static uint32_t s_dbg_t     = 0;    /* 调试行节拍 */
static int32_t  s_base_ax   = 0;    /* 基准（水平时的加速度投影） */
static int32_t  s_base_ay   = 0;
static int16_t  s_last_dh[6] = {0,0,0,0,0,0};  /* 上次输出，变化才写舵机 */

/* ---------- 内部：从原始数据取指定轴 ---------- */
static int16_t Axis_Get(const MPU6050_Data_t *d, uint8_t axis)
{
    switch (axis)
    {
    case 0:  return d->ax;
    case 1:  return d->ay;
    default: return d->az;
    }
}

/* ---------- 内部：死区线性化（死区内输出 0，带外只保留超出部分） ---------- */
static int32_t Deadzone(int32_t err, int32_t db)
{
    if (err >  db) return err - db;
    if (err < -db) return err + db;
    return 0;
}

/* ---------- 内部：无符号/带符号定点追加（调试行拼串用） ---------- */
static uint8_t Append_U(char *buf, uint8_t pos, uint32_t v)
{
    char tmp[10];
    uint8_t n = 0;
    if (v == 0) { buf[pos++] = '0'; return pos; }
    while (v) { tmp[n++] = (char)('0' + (v % 10u)); v /= 10u; }
    while (n) { buf[pos++] = tmp[--n]; }
    return pos;
}

static uint8_t Append_SI(char *buf, uint8_t pos, int32_t v)
{
    uint32_t a;
    if (v < 0) { buf[pos++] = '-'; a = (uint32_t)(-v); }
    else       { buf[pos++] = '+'; a = (uint32_t) v; }
    return Append_U(buf, pos, a);
}

static uint8_t Append_SQ1(char *buf, uint8_t pos, int32_t v_q1)
{
    uint32_t a;
    if (v_q1 < 0) { buf[pos++] = '-'; a = (uint32_t)(-v_q1); }
    else          { buf[pos++] = '+'; a = (uint32_t) v_q1; }
    pos = Append_U(buf, pos, a / 10u);
    buf[pos++] = '.';
    pos = Append_U(buf, pos, a % 10u);
    return pos;
}

/* ---------- 内部：静止采样基准（阻塞约 200ms，仅在等舵机到位后执行） ---------- */
static void CaptureBaseline(void)
{
    MPU6050_Data_t d;
    int32_t sx = 0, sy = 0;
    uint8_t i;

    for (i = 0; i < 20; i++)
    {
        MPU6050_Read_All(&d);
        sx += Axis_Get(&d, BAL_PITCH_AXIS);
        sy += Axis_Get(&d, BAL_ROLL_AXIS);
        delay_ms(10);
    }
    s_base_ax = sx / 20;
    s_base_ay = sy / 20;

    {
        char buf[48];
        uint8_t p = 0;
        const char *s1 = "[Bal] baseline ax=";
        const char *s2 = " ay=";
        while (*s1) buf[p++] = *s1++;
        p = Append_SI(buf, p, s_base_ax);
        while (*s2) buf[p++] = *s2++;
        p = Append_SI(buf, p, s_base_ay);
        buf[p++] = '\r'; buf[p++] = '\n'; buf[p] = 0;
        Bluetooth_SendString(buf);
    }
    Bluetooth_SendString("[Bal] ON - tilt the table, I keep level\r\n");
}

/* ==========================================================
 *                     对 外 接 口
 * ========================================================== */

const char *Balance_Start(void)
{
    /* MPU 未就绪：拒绝进入，避免拿无效数据乱调腿 */
    if (!MPU6050_Ready())
    {
        return "[Bal] ERROR: MPU6050 not ready (check boot log)\r\n";
    }

    /* 已在平衡模式：重新采基准即可 */
    if (s_stage != BAL_IDLE)
    {
        CaptureBaseline();
        return "[Bal] rebaseline done\r\n";
    }

    /* 1. 切姿态2：与 spidertwo 一致（用户约定姿态2为平衡初始姿态） */
    Gait_UseSet2();
    Gait_Stop();              /* 停步态 + 回站立（s_leg2 基准） */
    Attitude_ApplyState2();   /* 覆盖为姿态2 角度 */

    /* 2. 平衡模式舵机速度 = 步态速度（便于观察补偿方向对错） */
    Gait_SetSpeed(BAL_SPEED_Q1);

    /* 3. 等舵机到位后采基准（非阻塞，由 Tick 推进） */
    s_wait_t0 = GetSysTick_ms();
    s_stage   = BAL_WAIT_SERVO;
    return "[Cmd] BALANCE: switch attitude 2, sampling baseline...\r\n";
}

const char *Balance_Stop(void)
{
    uint8_t i;
    if (s_stage == BAL_IDLE) return "[Bal] not active\r\n";

    /* 腿回姿态2基准（清除平衡偏移） */
    for (i = 0; i < 6; i++)
    {
        Servo_SetTarget(s_bal_leg[i].ch_lift, s_bal_leg[i].lift0);
        Servo_SetTarget(s_bal_leg[i].ch_knee, s_bal_leg[i].knee0);
    }
    Gait_SetSpeed(GAIT_SPEED_DEFAULT);   /* 恢复默认舵机速度 */
    s_stage = BAL_IDLE;
    return "[Bal] OFF\r\n";
}

uint8_t Balance_Active(void)
{
    return (s_stage != BAL_IDLE) ? 1u : 0u;
}

void Balance_Tick(void)
{
    MPU6050_Data_t d;
    int32_t ep, er;          /* 死区后的误差（raw） */
    uint32_t now;
    uint8_t i;

    if (s_stage == BAL_IDLE) return;

    now = GetSysTick_ms();

    /* ---- 阶段1：等舵机全部到位（最多 2s 超时） ---- */
    if (s_stage == BAL_WAIT_SERVO)
    {
        if (Servo_AllIdle() || (now - s_wait_t0) >= 2000u)
        {
            CaptureBaseline();            /* 阻塞约 200ms */
            s_ctrl_t = GetSysTick_ms();
            s_dbg_t  = s_ctrl_t;
            s_stage  = BAL_RUN;
        }
        return;
    }

    /* ---- 阶段2：闭环运行 ---- */
    if ((now - s_ctrl_t) < BAL_TICK_MS) return;
    s_ctrl_t += BAL_TICK_MS;

    /* 滑动平均滤波：连续读 BAL_FILTER_N 次取平均，降低单次噪声 */
    {
        int32_t sx = 0, sy = 0;
        uint8_t f;
        for (f = 0; f < BAL_FILTER_N; f++)
        {
            MPU6050_Read_All(&d);
            sx += Axis_Get(&d, BAL_PITCH_AXIS);
            sy += Axis_Get(&d, BAL_ROLL_AXIS);
        }
        ep = Deadzone((sy / (int32_t)BAL_FILTER_N - s_base_ax) * BAL_PITCH_SIGN,
                      BAL_DEADZONE_RAW);
        er = Deadzone((sx / (int32_t)BAL_FILTER_N - s_base_ay) * BAL_ROLL_SIGN,
                      BAL_DEADZONE_RAW);
    }

    for (i = 0; i < 6; i++)
    {
        /* Δh(Q1,0.1°) = (x*ep + y*er) * Kp * 10 / (raw/° * 16) */
        int32_t num = (int32_t)s_bal_leg[i].px * ep + (int32_t)s_bal_leg[i].py * er;
        int32_t dh  = num * (int32_t)BAL_KP_Q4 * 10 / (BAL_RAW_PER_DEG * 16);
        int16_t dhq1;

        if (dh >  BAL_OUT_MAX_Q1) dh =  BAL_OUT_MAX_Q1;
        if (dh < -BAL_OUT_MAX_Q1) dh = -BAL_OUT_MAX_Q1;
        dhq1 = (int16_t)dh;

        if (dhq1 != s_last_dh[i])
        {
            /* 伸长(Δh>0)：lift/knee 语义偏移 -Δh -> 顶高机身该角 */
            Servo_SetTarget(s_bal_leg[i].ch_lift,
                            (uint16_t)(s_bal_leg[i].lift0 + s_bal_leg[i].s_lift * (-dhq1)));
            Servo_SetTarget(s_bal_leg[i].ch_knee,
                            (uint16_t)(s_bal_leg[i].knee0 + s_bal_leg[i].s_knee * (-dhq1)));
            s_last_dh[i] = dhq1;
        }
    }

    /* ---- 调试行：每 1s 上报当前误差（0.1°） ---- */
    if ((now - s_dbg_t) >= BAL_DBG_MS)
    {
        char buf[40];
        uint8_t p = 0;
        const char *h = "[Bal] tilt p=";
        while (*h) buf[p++] = *h++;
        p = Append_SQ1(buf, p, ep * 10 / BAL_RAW_PER_DEG);
        buf[p++] = ' ';
        buf[p++] = 'r';
        buf[p++] = '=';
        p = Append_SQ1(buf, p, er * 10 / BAL_RAW_PER_DEG);
        buf[p++] = '\r'; buf[p++] = '\n'; buf[p] = 0;
        Bluetooth_SendString(buf);
        s_dbg_t += BAL_DBG_MS;
    }
}
