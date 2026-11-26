#ifndef __GAIT_H
#define __GAIT_H

#include <stdint.h>

/* ============================================================
 *  六足机器人 - 涟漪步态(Ripple Gait) 对外接口
 * ------------------------------------------------------------
 *  涟漪步态：6 腿分 3 组（每组 2 条交错腿）轮流腾空，
 *  腾空拍之间有全支撑拍，任何时刻至少 4 腿支撑。
 *  比波浪步态快一倍，比三角步态稳。
 *
 *  使用前提：
 *    1. PCA9685_Init(两片板) + ServoSmooth_Init(90) 已完成
 *    2. 主循环每 20ms 调用一次 Gait_Tick()（内含 Servo_Step）
 *
 *  典型用法：
 *      Gait_Init();        // 站立姿态 + 速度初始化
 *      Gait_Forward();     // 持续前进
 *      while(1){ 每20ms: Gait_Tick(); }
 *      ...
 *      Gait_Stop();        // 停止并回站立
 * ============================================================ */

/* 默认步态速度（单位与平滑层一致：0.1°/每次调度）
 * 调度周期 20ms 时：max_step = 速度(°/s) × 0.02 × 10
 *  12 = 60°/s  18 = 90°/s  24 = 120°/s  30 = 150°/s（上限附近）
 *
 * !! 实测基准 !!：24 (120°/s) = 当年三角步态实测不抖的速度
 * （三角步态时代 10ms 调度周期 + max_step=12，换算到 20ms 周期 = 24）。
 * 涟漪步态承重渐变比三角步态更细（单拍最大 10°），该速度下应同样稳定。
 * MG90S 带载约 200~300°/s，超过 30 (150°/s) 相位内走不到位，步态拖沓。
 * 可用 Gait_SetSpeed() 运行时调。 */
#define GAIT_SPEED_DEFAULT   28u

typedef enum {
    GAIT_CMD_STOP = 0,      /* 停止：回站立姿态 */
    GAIT_CMD_FORWARD,       /* 前进 */
    GAIT_CMD_BACKWARD,      /* 后退 */
    GAIT_CMD_TURN_LEFT,     /* 原地左转 */
    GAIT_CMD_TURN_RIGHT     /* 原地右转 */
} GaitCmd_t;

/* 初始化：设置全部 18 路速度并进入站立姿态（上电即 90° 时无动作） */
void Gait_Init(void);

/* 步态调度：每 10ms 调用一次（内含 Servo_Step，勿再重复调用） */
void Gait_Tick(void);

/* 运行中修改步态速度（0.1°/10ms，如 8=80°/s 慢，20=200°/s 快） */
void Gait_SetSpeed(uint8_t speed_q1);

/* 切换步态基准集：状态1=s_leg（校准姿态1），状态2=s_leg2（校准姿态2）
 * 切换后 Apply_Stand/Apply_Phase 都围绕新表基准角摆动。
 * 通常配合 Attitude_ApplyState1/2 一起调用：先切步态集，再切姿态。 */
void Gait_UseSet1(void);
void Gait_UseSet2(void);

/* ---- 动作指令（下发后由 Gait_Tick 自动循环执行） ---- */
void Gait_Forward(void);    /* 持续前进 */
void Gait_Backward(void);   /* 持续后退 */
void Gait_TurnLeft(void);   /* 原地左转 */
void Gait_TurnRight(void);  /* 原地右转 */
void Gait_Stop(void);       /* 停止并平滑回站立 */

/* ---- 方案四：步态后姿态修正 ----
 * 每个步态周期结束（6 相位走完一圈）时，读一次 IMU，
 * 计算倾斜误差并微调 6 条腿的站立基准角（±3°），
 * 下一圈步态摆动围绕新基准走，实现"走歪了拉回来"。
 *
 * 调用方式：主循环每次迭代调用 Gait_ImuCorrect()，
 * 函数内部自行判断是否到了周期边界，不到就立即返回。 */
void Gait_ImuCorrect(void);

#endif
