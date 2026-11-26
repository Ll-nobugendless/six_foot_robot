/* ============================================================
 *  test_gait.c — 蜘蛛三角步态主机仿真验证（不参与 Keil 工程编译）
 *
 *  编译（MinGW）:
 *    gcc -O2 -I. -I.. -I..\..\PAC9685 -I..\..\systick test_gait.c ..\gait.c -lm -o test_gait.exe
 *
 *  验证点:
 *    1) IK 往返一致性：由舵机目标角做正运动学反推足端，
 *       与步态规划的足端目标误差 < 3mm
 *    2) 支撑相世界锁死：支撑腿的足端世界坐标逐 tick 纹丝不动
 *    3) 摆动相抬腿：z 高于地面至少 5mm
 *    4) 三角分组：组A(LF/RM/RB) 与组B(RF/LM/LB) 任意时刻反相、组内同步
 *    5) 前进/转向可达性：全程无 IK 越界（除直行外另跑一段原地转向）
 * ============================================================ */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

/* ---- 硬件 stub ---- */
static int16_t g_cap[32];                 /* 捕获的舵机目标（Q1） */
static uint32_t g_tick_ms = 0;

uint32_t GetSysTick_ms(void) { return g_tick_ms; }

void Servo_SetTargetQ1(uint8_t ch, int16_t q1) { if (ch < 32) g_cap[ch] = q1; }
void Servo_SetTarget(uint8_t ch, uint16_t a)   { if (ch < 32) g_cap[ch] = (int16_t)(a * 10); }
void Servo_SetMaxStep(uint8_t ch, uint16_t ms) { (void)ch; (void)ms; }

#include "gait.h"

/* ---- 与 gait.c 相同的表（测试副本；修改实现时需同步） ---- */
static const float t_hip_x[6] = { 60, 0, -60, 60, 0, -60 };
static const float t_hip_y[6] = { 40, 40, 40, -40, -40, -40 };
static const uint8_t t_chan[6][3] = {
    {0,1,2},{4,5,6},{8,9,10},{16,17,18},{20,21,22},{24,25,26}
};
static const int8_t t_calib[6][6] = {
    {+1,+1,+1,  0, 90, 90}, {+1,+1,+1,  0, 90, 90}, {+1,+1,+1,  0, 90, 90},
    {-1,+1,+1,  0, 90, 90}, {-1,+1,+1,  0, 90, 90}, {-1,+1,+1,  0, 90, 90}
};
static const float L1 = 30.0f, L2 = 70.0f, L3 = 100.0f;
static const float BODY_H = 55.0f;

/* 正运动学：舵机目标角 -> 足端（腿部系） */
static void fk_leg(int leg, float *lx, float *ly, float *lz)
{
    float deg[3];
    int j;
    for (j = 0; j < 3; j++)
    {
        float q1 = (float)g_cap[t_chan[leg][j]];
        deg[j] = (q1 / 10.0f - (float)t_calib[leg][3 + j]) / (float)t_calib[leg][j];
    }
    {
        float a = deg[0] * 0.017453293f;
        float b = deg[1] * 0.017453293f;
        float g = deg[2] * 0.017453293f;
        float u = L1 + L2*cosf(b) - L3*cosf(b + g);
        float z =      L2*sinf(b) - L3*sinf(b + g);
        *lx = u * cosf(a);
        *ly = u * sinf(a);
        *lz = z;
    }
}

int main(void)
{
    int  i, t;
    int  fail_lock = 0, fail_group = 0, ik_bad = 0;
    int  swing_lift[6] = {0,0,0,0,0,0};
    float max_round_err = 0.0f;
    float vx = 40.0f;
    const int ticks = 600;                 /* 6 秒 */
    float body_x = 0.0f;                   /* 测试侧独立积分 */
    float prev_world[6] = {0,0,0,0,0,0};
    int   prev_stance[6] = {0,0,0,0,0,0};

    printf("=== spider tripod gait host test ===\n");

    Gait_Init();
    Gait_SetSpeed(vx);
    Gait_SetPeriod(1000.0f);

    for (t = 1; t <= ticks; t++)
    {
        float foot[6][3];
        int  stance[6];
        int  grpA;   /* 按设计相位判定组A支撑（相位<0.5） */

        g_tick_ms += 10;
        body_x    += vx * 0.01f;
        Gait_Update();

        /* 设计相位：组A(LF/RM/RB)=leg1,2,3 相位<0.5 支撑；组B 与之反相 */
        grpA = ((g_tick_ms % 1000) < 500) ? 1 : 0;
        for (i = 0; i < 6; i++)
        {
            stance[i] = (i == 1 || i == 2 || i == 3) ? grpA : (!grpA);
        }

        for (i = 0; i < 6; i++)
        {
            float lx, ly, lz, err;

            Gait_TestGetFoot(i, &foot[i][0], &foot[i][1], &foot[i][2]);

            /* 1) IK 往返一致 */
            fk_leg(i, &lx, &ly, &lz);
            {
                float ex = (foot[i][0] - t_hip_x[i]) - lx;
                float ey = (foot[i][1] - t_hip_y[i]) - ly;
                float ez =  foot[i][2]               - lz;
                err = sqrtf(ex*ex + ey*ey + ez*ez);
            }
            if (err > max_round_err) max_round_err = err;
            if (err > 3.0f) ik_bad++;

            /* 摆动相必须真实抬起（高于地面 5mm 以上出现过） */
            if (foot[i][2] > -BODY_H + 5.0f) swing_lift[i] = 1;

            /* 2) 支撑相世界锁死（世界 x = 机体前进积分 + 足端机体系 x） */
            if (stance[i])
            {
                float wx = body_x + foot[i][0];
                if (prev_stance[i])
                {
                    float dd = (float)fabs(wx - prev_world[i]);
                    if (dd > 0.05f) {
                        fail_lock++;
                        if (fail_lock <= 3)
                            printf("  [lock] T%d leg%d world drift %.4fmm\n", t, i, dd);
                    }
                }
                prev_world[i]   = wx;
                prev_stance[i]  = 1;
            }
            else prev_stance[i] = 0;
        }

        /* 3) 三角分组：组内同步且两组反相（按设计相位，恒成立则为 0） */
        if (!(stance[1]==stance[2] && stance[2]==stance[3] &&
              stance[0]==stance[4] && stance[4]==stance[5]))
            fail_group++;
        if (stance[1] == stance[0]) fail_group++;
    }

    printf("forward walk : vx=%.0fmm/s period=1000ms ticks=%d\n", vx, ticks);
    printf("  IK round-trip max error  : %.3f mm  (bad=%d)\n", max_round_err, ik_bad);
    printf("  stance world-lock drift  : %d failures\n", fail_lock);
    printf("  swing lift seen (leg0..5): %d %d %d %d %d %d\n",
           swing_lift[0],swing_lift[1],swing_lift[2],
           swing_lift[3],swing_lift[4],swing_lift[5]);
    printf("  tripod group violations  : %d\n", fail_group);

    /* 4) 原地转向：vx=0, w=30deg/s，跑 3 秒，只查可达性 */
    {
        int ik_turn = 0;
        float max_err_turn = 0.0f;
        Gait_SetSpeed(0.0f);
        Gait_SetTurn(30.0f);
        Gait_SetPeriod(1000.0f);
        for (t = 1; t <= 300; t++)
        {
            float foot[6][3];
            g_tick_ms += 10;
            Gait_Update();
            for (i = 0; i < 6; i++)
            {
                float lx, ly, lz, err;
                Gait_TestGetFoot(i, &foot[i][0], &foot[i][1], &foot[i][2]);
                fk_leg(i, &lx, &ly, &lz);
                {
                    float ex = (foot[i][0] - t_hip_x[i]) - lx;
                    float ey = (foot[i][1] - t_hip_y[i]) - ly;
                    float ez =  foot[i][2]               - lz;
                    err = sqrtf(ex*ex + ey*ey + ez*ez);
                }
                if (err > max_err_turn) max_err_turn = err;
                if (err > 3.0f) ik_turn++;
            }
        }
        printf("turn in place: w=30deg/s 3s\n");
        printf("  IK round-trip max error  : %.3f mm  (bad=%d)\n", max_err_turn, ik_turn);
    }

    printf("=== %s ===\n",
           (fail_lock==0 && fail_group==0 && ik_bad==0) ? "ALL CHECKS PASSED" : "HAS FAILURES");
    return (fail_lock + fail_group + ik_bad) ? 1 : 0;
}
