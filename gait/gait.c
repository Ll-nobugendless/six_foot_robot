/* ====================================================
   文件名 : gait/gait.c
   功能   : 六足机器人 - 涟漪步态(Ripple Gait)封装
   ----------------------------------------------------
   步态原理（涟漪步态，速度与稳定性的折中）：
     6 条腿分 3 组，每组 2 条交错腿，轮流腾空：
       组A = L1左前 + R3右后
       组B = L2左中 + R1右前
       组C = L3左后 + R2右中
     6 相位一个周期（腾空组之间插入全支撑拍）：
       相位0: 组A腾空(抬+前摆)，组B/C 支撑后蹬
       相位1: 全部支撑（组A落地踩实，组B/C 继续后蹬）
       相位2: 组B腾空，组A/C 支撑后蹬
       相位3: 全部支撑
       相位4: 组C腾空，组A/B 支撑后蹬
       相位5: 全部支撑
       -> 循环往复实现连续行走
     任何时刻至少 4 条腿支撑，比波浪步态快一倍
     （腾空拍从 6 个减到 3 个），比三角步态稳。

   每腿 3 个关节：
       hip  = 髋关节(前后摆)   -> 决定迈步/蹬地
       lift = 大腿关节(抬放)   -> 决定抬腿离地
       knee = 膝关节(收展)     -> 抬起时收腿，落地时展开

   依赖：pac.h 平滑舵机层（Servo_SetTarget/Servo_Step/
         Servo_AllIdle/Servo_SetMaxStep）
   调度：Gait_Tick() 每 20ms 调用一次（与 SERVO_TICK_MS 一致）
   ==================================================== */

#include "gait.h"
#include "pac.h"
#include "mpu6050.h"  /* 方案四：步态周期末读 IMU 做姿态修正 */

/* ==========================================================
 *  步态姿态参数（MG90S 实测调参）
 *  MG90S: 扭矩 1.8kg·cm@4.8V，速度 0.1s/60°空载
 *  带载后实际速度 200~300°/s，参数要保守避免抖动无力
 * ========================================================== */
#define GAIT_NEUTRAL_DEG    90    /* 中位角（与校准角一致）     */
#define GAIT_HIP_SWING      15    /* 髋关节摆幅(°)：迈步远近    */
#define GAIT_LIFT_UP        20    /* 抬腿高度(°)：越大抬越高    */
#define GAIT_KNEE_TUCK      15    /* 收腿幅度(°)：抬起时收腿    */
#define GAIT_LEG_NUM        6

/* ==========================================================
 *  腿部通道映射 + 方向系数表
 * ----------------------------------------------------------
 * 通道号与校准模式一致（!! 左右按实际机身：0~15=左侧板，16~31=右侧板 !!）：
 *   左板 PB8/PB9  : L1=0,1,2   L2=4,5,6    L3=8,9,10
 *   右板 PB12/PB13: R1=16,17,18  R2=20,21,22  R3=24,25,26
 *
 * s_xxx 方向系数（+1/-1）：把"语义正方向"换算成舵机实际角度。
 * 语义正方向定义：hip +1=向前摆  lift +1=向上抬  knee +1=收腿
 *
 * !! 调试要点 !!：左右两侧舵机镜像安装，默认左腿全部取 -1。
 * 上电跑步态时若某条腿动作方向反了（往前走却在划水/往下砸），
 * 只需把对应腿对应关节的系数改成相反符号即可，其它不用动。
 * ========================================================== */
typedef struct
{
    uint8_t ch_hip;     /* 髋关节逻辑通道 */
    uint8_t ch_lift;    /* 大腿逻辑通道 */
    uint8_t ch_knee;    /* 膝关节逻辑通道 */
    int8_t  s_hip;      /* 髋方向系数 */
    int8_t  s_lift;     /* 大腿方向系数 */
    int8_t  s_knee;     /* 膝方向系数 */
    uint8_t hip0;       /* 站立姿态：髋绝对角度（校准值） */
    uint8_t lift0;      /* 站立姿态：大腿绝对角度（校准值） */
    uint8_t knee0;      /* 站立姿态：膝绝对角度（校准值） */
} GaitLeg_t;

/* 站立基准角为实测校准姿态：Gait_Init / Gait_Stop 都回到这组角度，
 * 步态摆动围绕该基准 ±偏移。hip 全部居中 90，lift/knee 为校准值。
 * s_lift 为逐腿实测标定（与各位置舵机安装朝向有关，无统一侧向规律）。 */
static const GaitLeg_t s_leg[GAIT_LEG_NUM] =
{
    /* ch_hip ch_lift ch_knee  s_hip s_lift s_knee  hip0 lift0 knee0 */
    {   0,      1,      2,      +1,   +1,   -1,     90,  140,  150 },  /* L1 左前 */
    {   4,      5,      6,      +1,   -1,   +1,     90,   40,   30 },  /* L2 左中 */
    {   8,      9,     10,      +1,   -1,   +1,     90,   50,   30 },  /* L3 左后 */
    {  16,     17,     18,      -1,   +1,   -1,     90,  150,  140 },  /* R1 右前 */
    {  20,     21,     22,      -1,   +1,   -1,     90,  140,  150 },  /* R2 右中 */
    {  24,     25,     26,      -1,   -1,   +1,     90,   40,   40 },  /* R3 右后 */
};

/* 状态2基准角表：与 attitude.c 的 s_att2 一致。
 * 用于状态2步态摆动的基准角，可通过 Gait_UseSet2() 切换激活。
 * s_lift/s_knee 方向逐腿标定，与 s_leg 一致。 */
static const GaitLeg_t s_leg2[GAIT_LEG_NUM] =
{
    /* ch_hip ch_lift ch_knee  s_hip s_lift s_knee  hip0 lift0 knee0 */
    {   0,      1,      2,      +1,   +1,   -1,     90,   90,  100 },  /* L1 左前 */
    {   4,      5,      6,      +1,   -1,   +1,     90,   90,   80 },  /* L2 左中 */
    {   8,      9,     10,      +1,   -1,   +1,     90,   90,   70 },  /* L3 左后 */
    {  16,     17,     18,      -1,   +1,   -1,     90,   90,  100 },  /* R1 右前 */
    {  20,     21,     22,      -1,   +1,   -1,     90,   90,  100 },  /* R2 右中 */
    {  24,     25,     26,      -1,   -1,   +1,     90,   90,   70 },  /* R3 右后 */
};

static GaitCmd_t g_cmd   = GAIT_CMD_STOP;   /* 当前动作指令   */
static uint8_t   g_phase = 0;               /* 当前相位 0~3   */
static uint8_t   g_speed = GAIT_SPEED_DEFAULT; /* 0.1°/10ms   */
static uint8_t   g_leg_set = 0;             /* 当前激活步态集：0=状态1, 1=状态2 */

/* ---- 方案四：步态后姿态修正 ----
 * 每个步态周期结束（相位 5→0）时读一次 IMU，
 * 按 pitch/roll 误差微调各腿基准角，下一圈步态围绕新基准走。 */
/* 各腿位置系数：px=+1前/0中/-1后，py=-1左/+1右 */
static const int8_t s_px[6] = {+1,  0, -1, +1,  0, -1};
static const int8_t s_py[6] = {-1, -1, -1, +1, +1, +1};

/* IMU 修正参数（与 balance.c 保持一致的方向和死区） */
#define GIMU_PITCH_AXIS  0       /* 0=ax 1=ay */
#define GIMU_ROLL_AXIS   1
#define GIMU_PITCH_SIGN  (+1)     /* 与 balance.c BAL_PITCH_SIGN 一致 */
#define GIMU_ROLL_SIGN   (+1)     /* 与 balance.c BAL_ROLL_SIGN 一致 */
#define GIMU_RAW_PER_DEG 286
#define GIMU_DEADZONE     2860    /* 死区 ±10°，水平时不修正 */
#define GIMU_KP_Q4        2       /* P 增益 0.125，非常温和 */
#define GIMU_OUT_MAX      30      /* 单腿修正限幅 ±3.0°（Q1 0.1°） */

/* 运行时修正量（raw 角度偏移，已含方向系数换算） */
static int8_t  g_corr_lift[6]  = {0,0,0,0,0,0};
static int8_t  g_corr_knee[6]  = {0,0,0,0,0,0};
/* IMU 基准（水平时的 ax/ay） */
static int32_t g_base_ax = 0;
static int32_t g_base_ay = 0;
static uint8_t g_base_ok = 0;      /* 0=待采集 1=已采集 */
static uint8_t g_imu_pending = 0;  /* 1=周期结束待修正 */

/* ---------- 内部：角度限幅 ---------- */
static uint16_t Clamp180(int v)
{
    if (v < 0)   return 0;
    if (v > 180) return 180;
    return (uint16_t)v;
}

/* ---------- 内部：根据 g_leg_set 选激活的腿表 ----------
 * 0 = s_leg (状态1)，1 = s_leg2 (状态2) */
static const GaitLeg_t * GetLeg(uint8_t idx)
{
    return (g_leg_set == 0) ? &s_leg[idx] : &s_leg2[idx];
}

/* ---------- 内部：单腿下发目标 ----------
 * hip/lift/knee 为"语义偏移量"(°)：+ = 前摆/抬起/收腿
 * 实际角度 = 基准角(xxx0) + IMU修正(g_corr) + 方向系数 * 偏移量 */
static void Leg_Go(uint8_t idx, int16_t hip, int16_t lift, int16_t knee)
{
    const GaitLeg_t *L = GetLeg(idx);
    Servo_SetTarget(L->ch_hip,  Clamp180((int)L->hip0  + L->s_hip  * hip));
    Servo_SetTarget(L->ch_lift, Clamp180((int)L->lift0 + g_corr_lift[idx] + L->s_lift * lift));
    Servo_SetTarget(L->ch_knee, Clamp180((int)L->knee0 + g_corr_knee[idx] + L->s_knee * knee));
}

/* ---------- 内部：站立姿态（全部回中位） ---------- */
static void Apply_Stand(void)
{
    uint8_t i;
    for (i = 0; i < GAIT_LEG_NUM; i++)
    {
        Leg_Go(i, 0, 0, 0);
    }
}

/* ---------- 内部：当前指令下每条腿的"行走方向" ----------
 * +1 = 该腿向前迈步/向后蹬地（身体相对前进）
 * -1 = 反向
 * 原地转向：左转 = 左腿净向后 + 右腿净向前（身体逆时针转）
 * !! idx 0,1,2 = 左腿（左侧板），idx 3,4,5 = 右腿（右侧板） !! */
static int8_t Leg_WalkDir(uint8_t idx)
{
    uint8_t is_right = (idx >= 3);
    switch (g_cmd)
    {
    case GAIT_CMD_FORWARD:    return  1;
    case GAIT_CMD_BACKWARD:   return -1;
    case GAIT_CMD_TURN_LEFT:  return is_right ?  1 : -1;
    case GAIT_CMD_TURN_RIGHT: return is_right ? -1 :  1;
    default:                  return  0;
    }
}

/* ---------- 内部：每条腿的腾空相位（涟漪步态分组） ----------
 * 组A = L1(0) + R3(5)，相位0腾空
 * 组B = L2(1) + R1(3)，相位2腾空
 * 组C = L3(2) + R2(4)，相位4腾空
 * 同组两条腿一左一右交错，腾空时对侧仍有支撑，稳定。 */
static const uint8_t s_lift_phase[GAIT_LEG_NUM] = {0, 2, 4, 2, 4, 0};

/* ---------- 内部：下发一个步态相位（对 6 条腿同时设目标） ----------
 * 涟漪步态（Ripple Gait）：6 相位一周期，3 组轮流腾空，
 * 腾空拍之间有全支撑拍，任何时刻至少 4 腿支撑。
 *
 * 每条腿按"相对腾空拍"rp = (ph - 本腿腾空相位 + 6) % 6 决定动作：
 *   rp=0 : 腾空相（抬腿 + 空中前摆 + 收膝），此时腿已卸载
 *   rp=1 : 落地相（腿放下踩实，髋保持前摆位 +SWING）
 *   rp=2 : 过渡相（髋 +SWING → +SWING/3，开始后蹬渐变）
 *   rp=3 : 支撑相（髋 +SWING/3 → -SWING/3）
 *   rp=4,5: 支撑相（髋在后摆位 -SWING，向后蹬地）
 *
 * 关键：承重腿的髋走 +SWING → +SWING/3 → -SWING/3 → -SWING
 * 三段渐变，单拍最大变化 2/3·SWING(10°)，落地冲击被分摊，
 * MG90S 扛得住不抖。唯一的 30° 大幅摆髋发生在腾空拍（空载）。
 * 转向时通过 Leg_WalkDir 改变各腿方向，分组与抬腿时序不变。 */
static void Apply_Phase(uint8_t ph)
{
    uint8_t i;

    for (i = 0; i < GAIT_LEG_NUM; i++)
    {
        int8_t d    = Leg_WalkDir(i);
        uint8_t rp  = (uint8_t)((ph + 6u - s_lift_phase[i]) % 6u);

        switch (rp)
        {
        case 0:
            /* 腾空相：抬腿 + 空中前摆 + 收膝（腿已卸载，可快速摆髋） */
            Leg_Go(i, d * GAIT_HIP_SWING, GAIT_LIFT_UP, GAIT_KNEE_TUCK);
            break;

        case 1:
            /* 落地相：腿放下、膝展开，髋保持前摆位（承重踩实） */
            Leg_Go(i, d * GAIT_HIP_SWING, 0, 0);
            break;

        case 2:
            /* 过渡相：髋开始后蹬第一段渐变（+SWING → +SWING/3） */
            Leg_Go(i, d * (GAIT_HIP_SWING / 3), 0, 0);
            break;

        case 3:
            /* 支撑相：髋后蹬第二段渐变（+SWING/3 → -SWING/3） */
            Leg_Go(i, -d * (GAIT_HIP_SWING / 3), 0, 0);
            break;

        default:
            /* 支撑相：髋在后摆位，在地面向后蹬（推身体前进） */
            Leg_Go(i, -d * GAIT_HIP_SWING, 0, 0);
            break;
        }
    }
}

/* ==========================================================
 *                     对 外 接 口
 * ========================================================== */

void Gait_Init(void)
{
    uint8_t i;
    const GaitLeg_t *L;
    g_cmd     = GAIT_CMD_STOP;
    g_phase   = 0;
    g_leg_set = 1;   /* 上电默认用状态2步态集 */
    for (i = 0; i < GAIT_LEG_NUM; i++)
    {
        L = GetLeg(i);
        Servo_SetMaxStep(L->ch_hip,  g_speed);
        Servo_SetMaxStep(L->ch_lift, g_speed);
        Servo_SetMaxStep(L->ch_knee, g_speed);
    }
    Apply_Stand();      /* 从校准 90° 上电时无动作；偏了则平滑回正 */
}

void Gait_SetSpeed(uint8_t speed_q1)
{
    uint8_t i;
    const GaitLeg_t *L;
    if (speed_q1 == 0) speed_q1 = 1;
    g_speed = speed_q1;
    for (i = 0; i < GAIT_LEG_NUM; i++)
    {
        L = GetLeg(i);
        Servo_SetMaxStep(L->ch_hip,  g_speed);
        Servo_SetMaxStep(L->ch_lift, g_speed);
        Servo_SetMaxStep(L->ch_knee, g_speed);
    }
}

/* 切换步态集到状态1：之后 Gait_Tick/Apply_Stand 围绕 s_leg 摆动 */
void Gait_UseSet1(void)
{
    g_leg_set = 0;
    Gait_SetSpeed(g_speed);   /* 重设 MaxStep（通道号相同，可省，但保持对称） */
}

/* 切换步态集到状态2：之后 Gait_Tick/Apply_Stand 围绕 s_leg2 摆动 */
void Gait_UseSet2(void)
{
    g_leg_set = 1;
    Gait_SetSpeed(g_speed);
}

void Gait_Tick(void)
{
    Servo_Step();                    /* 平滑层步进（写 PWM）     */
    if (!Servo_AllIdle()) return;    /* 本相位未走完，等待       */

    if (g_cmd == GAIT_CMD_STOP)
    {
        g_phase = 0;                 /* 站立保持，不再下发目标   */
        return;
    }

    Apply_Phase(g_phase);            /* 全部到位 -> 下发下一相位 */
    g_phase = (uint8_t)((g_phase + 1u) % 6u);  /* 涟漪步态 6 相位循环 */
    if (g_phase == 0) g_imu_pending = 1;  /* 周期结束，标记待 IMU 修正 */
}

void Gait_Forward(void)  { g_cmd = GAIT_CMD_FORWARD;    g_phase = 0; }
void Gait_Backward(void) { g_cmd = GAIT_CMD_BACKWARD;   g_phase = 0; }
void Gait_TurnLeft(void) { g_cmd = GAIT_CMD_TURN_LEFT;  g_phase = 0; }
void Gait_TurnRight(void){ g_cmd = GAIT_CMD_TURN_RIGHT; g_phase = 0; }

void Gait_Stop(void)
{
    uint8_t i;
    g_cmd   = GAIT_CMD_STOP;
    g_phase = 0;
    g_imu_pending = 0;
    g_base_ok = 0;            /* 下次走路时重新采基准 */
    for (i = 0; i < 6; i++)   /* 清零修正，回原始基准角 */
    {
        g_corr_lift[i] = 0;
        g_corr_knee[i] = 0;
    }
    Apply_Stand();                   /* 平滑回到站立姿态 */
}

/* ==========================================================
 *  方案四：步态后姿态修正
 * ----------------------------------------------------------
 *  每个步态周期结束（相位 5→0）时，Gait_Tick 设 g_imu_pending=1。
 *  主循环调用本函数，内部检查 pending 标志：
 *    - 未到周期边界 → 立即返回
 *    - 到了 → 读一次 IMU，算 pitch/roll 误差，
 *      按各腿前后/左右位置分配修正量，叠加到下一圈基准角。
 *
 *  特点：
 *    - 不碰步态摆动时序，只在周期间隙做修正
 *    - 修正量很小（±3°），不会干扰步态
 *    - 水平（死区内）时修正量自然衰减到 0
 *    - 第一次走路时自动采集水平基准（读一次 IMU）
 * ========================================================== */

/* 内部：死区处理 */
static int32_t Gimu_Deadzone(int32_t err, int32_t db)
{
    if (err >  db) return err - db;
    if (err < -db) return err + db;
    return 0;
}

/* 内部：从 MPU6050_Data_t 取指定轴 */
static int16_t Gimu_Axis(const MPU6050_Data_t *d, uint8_t axis)
{
    switch (axis)
    {
    case 0:  return d->ax;
    case 1:  return d->ay;
    default: return d->az;
    }
}

void Gait_ImuCorrect(void)
{
    MPU6050_Data_t d;
    int32_t ep, er;       /* 死区后的误差 */
    uint8_t i;

    if (!g_imu_pending) return;       /* 未到周期边界 */
    g_imu_pending = 0;

    if (!MPU6050_Ready()) return;     /* IMU 未就绪，跳过 */

    MPU6050_Read_All(&d);

    /* 第一次走路：采集水平基准（读一次，不阻塞） */
    if (!g_base_ok)
    {
        g_base_ax = Gimu_Axis(&d, GIMU_PITCH_AXIS);
        g_base_ay = Gimu_Axis(&d, GIMU_ROLL_AXIS);
        g_base_ok = 1;
        return;   /* 这一圈不修正，等下一圈 */
    }

    ep = Gimu_Deadzone(((int32_t)Gimu_Axis(&d, GIMU_PITCH_AXIS) - g_base_ax) * GIMU_PITCH_SIGN,
                       GIMU_DEADZONE);
    er = Gimu_Deadzone(((int32_t)Gimu_Axis(&d, GIMU_ROLL_AXIS)  - g_base_ay) * GIMU_ROLL_SIGN,
                       GIMU_DEADZONE);

    /* 误差为 0 时，修正量直接归零（不累积） */
    for (i = 0; i < 6; i++)
    {
        const GaitLeg_t *L = GetLeg(i);
        int32_t num = (int32_t)s_px[i] * ep + (int32_t)s_py[i] * er;
        /* dh(Q1) = num * Kp * 10 / (raw_per_deg * 16) */
        int32_t dh = num * (int32_t)GIMU_KP_Q4 * 10 / (GIMU_RAW_PER_DEG * 16);
        int8_t corr;

        if (dh >  GIMU_OUT_MAX) dh =  GIMU_OUT_MAX;
        if (dh < -GIMU_OUT_MAX) dh = -GIMU_OUT_MAX;
        corr = (int8_t)dh;

        /* 伸长(dh>0)：lift/knee 基准偏移 = s * (-dh) */
        g_corr_lift[i] = (int8_t)((int32_t)L->s_lift * (-corr) / 10);
        g_corr_knee[i] = (int8_t)((int32_t)L->s_knee * (-corr) / 10);
    }
}
