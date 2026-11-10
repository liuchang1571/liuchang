/**
  ******************************************************************************
  * @file    task_chassis.c
  * @brief   底盘主任务：1kHz 控制循环（齿条式阿卡曼，2 个后轮 + 1 个转向）
  *
  *   ★ 云台(yaw/pitch) 和 发射(摩擦轮/拨弹盘) 已经拆到 task_gimbal.c 的
  *     独立 1kHz 任务里，本文件只管底盘，和 CAN2。
  *
  * ============================================================================
  *  一个周期（1ms）里的 6 步，顺序不能乱：
  *      ① 处理遥控器（取新帧 / 判断关控 / 摇杆回中）
  *      ② 摇杆 -> 目标速度 + 目标齿条位置
  *      ③ 运动解算 -> 左右后轮转速 + 齿条位置
  *      ④ 把目标写进电机对象
  *      ⑤ 后轮速度环 PID（2个）+ 转向位置环 PID（1个）
  *      ⑥ 通过 CAN2 发出去（0x200 后轮 / 0x1FF 转向）
  *      ⑦ 等下一个 1ms
  *
  * ============================================================================
  *  摇杆分工（右摇杆 ch0/ch1 和 S1/S2 归云台任务，见 task_gimbal.c）
  * ============================================================================
  *      左摇杆上下（ch3） -> 前进后退，推到底 = CHASSIS_MAX_RPM
  *      左摇杆左右（ch2） -> 齿条位置，推到底 = ±22mm（打满方向）
  ******************************************************************************
  */
#include "task_chassis.h"
#include "config.h"        /* 参数、rc_data、rc_lost_flag */
#include "drv_dbus.h"      /* DBUS_GetChannel */
#include "drv_motor.h"     /* 后轮速度环 */
#include "drv_steer.h"     /* 前轮转向位置环 */
#include "driver_gimbal.h" /* 只为了 g_vehicle_fault（全车失能标志） */
#include "bsp_can.h"       /* 发 CAN2 */
#include "task_rc.h"       /* Task_RC_Loop */
#include "ackermann.h"     /* 运动解算 */
#include "drv_power.h"     /* 功率限制：Power_Init / Power_CalcScale */

/* FreeRTOS 的头文件。注意顺序：FreeRTOS.h 必须在 task.h 前面。 */
#include "FreeRTOS.h"
#include "task.h"

#include <math.h>

/* ============================================================================
 *  两个小工具函数（只在本文件用，所以加 static）
 * ==========================================================================*/

/**
  * @brief  把摇杆原始值（364~1684）变成 -1.0 ~ +1.0
  * 算法：(raw - 1024) / 660
  */
static float RC_Norm(uint16_t raw)
{
    float v;

    v = (raw - RC_CH_MID) / RC_CH_RANGE;

    if (v > 1.0f)
    {
        v = 1.0f;
    }
    if (v < -1.0f)
    {
        v = -1.0f;
    }
    return v;
}

/**
  * @brief  死区：推杆量小于 dz 就当成 0
  * @note   摇杆松手后不会精确回到 1024，不加死区车会一直慢慢爬
  */
static float RC_DeadZone(float v, float dz)
{
    if (fabsf(v) < dz)
    {
        return 0.0f;
    }
    return v;
}

/* ============================================================================
 *  Task_Chassis_Init —— 初始化底盘
 *  调用者：freertos.c 的 Init_Task（StartTask04），开机一次
 * ==========================================================================*/
void Task_Chassis_Init(void)
{
    /* ---- 后轮：速度环（2 个电机）----
     * 编号 = 电调 ID 减 1（因为 0x200+ID 的反馈索引是 ID-1）。
     *   左后轮 ID2 -> 1 号电机（反馈 0x202）
     *   右后轮 ID1 -> 0 号电机（反馈 0x201） */
    Motor_Init(DRIVE_ID_L - 1, DRIVE_PID_KP, DRIVE_PID_KI, DRIVE_PID_KD);
    Motor_Init(DRIVE_ID_R - 1, DRIVE_PID_KP, DRIVE_PID_KI, DRIVE_PID_KD);

    /* ---- 前轮：位置环（1 个电机推齿条）----
     * 后两个参数是标定值：
     *     STEER_CENTER_RAW  齿条居中时的编码器值
     *     STEER_RAW_PER_MM  每毫米多少编码器计数
     * ★ 这两个必须你自己标定，详见 config.h 的说明。 */
    Steer_Init(STEER_PID_KP, STEER_PID_KI, STEER_PID_KD,
               STEER_CENTER_RAW, STEER_RAW_PER_MM);

    /* ---- 功率限制：初始化 ----
     * 参数全在 config.h 第 5.5 部分，这里只是把状态清零。 */
    Power_Init();
}

/* ============================================================================
 *  Task_Chassis_Loop —— 1kHz 主循环
 *  调用者：freertos.c 的 Chassis_Task（StartTask02），调用后不返回
 * ==========================================================================*/
void Task_Chassis_Loop(void)
{
    TickType_t last_wake;      /* 上次醒来的时刻，vTaskDelayUntil 要用 */
    float dt;                  /* 周期，单位秒 */
    float base_rpm;            /* 摇杆给的速度，单位转子 rpm */
    float rack_mm;             /* 摇杆给的齿条位置，单位毫米 */
    Ackermann_Out_t out;       /* 运动解算结果 */
    Motor_t *motor_l;          /* 左后轮电机对象 */
    Motor_t *motor_r;          /* 右后轮电机对象 */
    Steer_t *steer;            /* 转向电机对象 */

    /* 先把电机对象的地址取出来，后面就不用反复调 Motor_Get / Steer_Get */
    motor_l = Motor_Get(DRIVE_ID_L - 1);
    motor_r = Motor_Get(DRIVE_ID_R - 1);
    steer = Steer_Get();

    /* vTaskDelayUntil 需要一个起始时刻，先取一次 */
    last_wake = xTaskGetTickCount();

    /* dt = 0.001 秒，因为循环周期是 1ms。这个值传给 PID 算积分。 */
    dt = 0.001f;

    /* ==================== 死循环开始，每 1ms 转一圈 ==================== */
    for (;;)
    {
        /* ---- 心跳：证明这个 1kHz 循环真的在跑（只给调试用，不参与控制）----
         * 恒为 0 就说明任务压根没跑起来，问题在初始化而不是 CAN。 */
        g_alive_cnt = g_alive_cnt + 1U;

        /* ============================================================
         *  ① 处理遥控器
         * ------------------------------------------------------------
         *     有新帧         -> 拷进 rc_data + 解除关控
         *     超过 21ms 没帧 -> 摇杆回中 + 置关控标志
         *  ★ 云台任务也读 rc_data，最多比这里晚 1ms（对 14ms 一帧没影响）
         * ========================================================== */
        Task_RC_Loop();

        /* ============================================================
         *  ①.5 全车失能（翻车保护）
         * ------------------------------------------------------------
         *  g_vehicle_fault 现在恒为 0。
         *  以后接上陀螺仪：pitch 或 roll > 80° 置 1，回到 50° 以下清 0。
         *  置 1 时后轮/转向输出 0（云台那边自己有同样的检查）。
         * ========================================================== */
        if (g_vehicle_fault)
        {
            motor_l->target_rpm   = 0.0f;
            motor_l->give_current = 0;
            motor_l->integral     = 0.0f;
            motor_r->target_rpm   = 0.0f;
            motor_r->give_current = 0;
            motor_r->integral     = 0.0f;
            steer->give_out       = 0;

            BSP_CAN_Send_Drive_Current(0, 0);   /* CAN2 0x200 */
            BSP_CAN_Send_Steer_Voltage(0);      /* CAN2 0x1FF */

            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
            continue;
        }

        /* ============================================================
         *  ② 摇杆 -> 目标值
         * ========================================================== */

        /* 先给安全初值 0（关控时就是靠这个让车停下、齿条不动） */
        base_rpm = 0.0f;
        rack_mm = 0.0f;

        /* 只有遥控器没关控时才听摇杆。
         * 关控时 base_rpm / rack_mm 保持 0：
         *   后轮目标速度 0（速度环会把车刹停）
         *   转向目标不更新（见第 ④ 步），齿条保持原位不回中 */
        if (rc_lost_flag == 0)
        {
            /* ch3 = 左摇杆上下 -> 前进速度（转子 rpm） */
            base_rpm = RC_Norm(DBUS_GetChannel(&rc_data, 3));
            base_rpm = RC_DeadZone(base_rpm, RC_DEAD_ZONE);
            base_rpm = base_rpm * CHASSIS_MAX_RPM;

            /* ch2 = 左摇杆左右 -> 齿条位置（毫米，正 = 向左）
             * ★ 最后乘 STEER_DIR：用来适配"摇杆方向和齿条方向相反"。
             *   翻方向只能翻这里，不能翻 STEER_RAW_PER_MM（原因见 config.h）。 */
            rack_mm = RC_Norm(DBUS_GetChannel(&rc_data, 2));
            rack_mm = RC_DeadZone(rack_mm, RC_DEAD_ZONE);
            rack_mm = rack_mm * STEER_RACK_MM_MAX * STEER_DIR;
        }

        /* ============================================================
         *  ③ 运动解算（阿卡曼）
         * ------------------------------------------------------------
         *  输入：base_rpm（直行速度）+ rack_mm（齿条位置）
         *  输出：左右后轮各自的目标转速 + 限幅后的齿条位置
         * ========================================================== */
        Ackermann_Solve(base_rpm, rack_mm, &out);

        /* ============================================================
         *  ④ 把目标写进电机对象
         * ========================================================== */

        /* ---- 后轮：目标是转速（转子 rpm）----
         * 乘 DIR 是为了适配"电机左右反装"。 */
        motor_l->target_rpm = out.rear_rpm[0] * DRIVE_DIR_L;
        motor_r->target_rpm = out.rear_rpm[1] * DRIVE_DIR_R;

        /* ---- 前轮：目标是齿条位置（毫米）----
         * ★ 关控时【故意不更新】转向目标，齿条保持原位不回正。 */
        if (rc_lost_flag == 0)
        {
            Steer_SetTargetMM(out.rack_mm);
        }

        /* ============================================================
         *  ⑤ 三个闭环各算一次
         * ========================================================== */

        /* ---- 后轮速度环（2 个）---- */
        Motor_Speed_PID_Calc(motor_l, dt);
        Motor_Speed_PID_Calc(motor_r, dt);

        /* ============================================================
         *  ⑤.5 功率限制（裁判系统规则）
         * ------------------------------------------------------------
         *  规则：底盘功率超限会扣"缓冲能量"，扣到 0 以下直接断电 5 秒。
         *
         *  做法：把两路 PID 电流当成 I，用功率模型
         *          F(k) = A·k² + B·k + L
         *        解出满足 F(k) ≤ P 的最大 k，再把电流乘上 k。
         *
         *  ★ 必须在 PID 之后、发 CAN 之前做。
         *  ★ 公式和参数见 config.h 第 5.5 部分 / drv_power.h。 */
        {
            int16_t i_buf[2];       /* 两路电流 */
            int16_t n_buf[2];       /* 两路转速 */
            float   k;              /* 缩放系数 0~1 */

            i_buf[0] = motor_l->give_current;
            i_buf[1] = motor_r->give_current;
            n_buf[0] = (int16_t)motor_l->actual_rpm;
            n_buf[1] = (int16_t)motor_r->actual_rpm;

            k = Power_CalcScale(i_buf, n_buf, 2U);

            if (k < 0.999f)
            {
                /* 两路按同一个 k 缩放（"统一缩放系数"的含义） */
                motor_l->give_current = (int16_t)((float)motor_l->give_current * k);
                motor_r->give_current = (int16_t)((float)motor_r->give_current * k);

                /* 抗积分饱和：限功率期间把积分往回压。
                 * 不压的话，限功率时误差一直消不掉，积分会攒满，
                 * 一旦功率放开就会猛冲一下。
                 * （ki = 0 时积分本来就不起作用，这里是为以后留的。） */
                if (motor_l->ki > 0.0001f)
                {
                    motor_l->integral = motor_l->integral * k;
                }
                if (motor_r->ki > 0.0001f)
                {
                    motor_r->integral = motor_r->integral * k;
                }
            }
        }

        /* ---- 前轮转向位置环（1 个）---- */
        Steer_Pos_PID_Calc(dt);



        /* ============================================================
         *  ⑥ 发 CAN2
         * ------------------------------------------------------------
         *  ★ 不管什么情况都必须发！
         *    电调没有超时保护：你不发，它就保持上一次的电流一直出力。
         * ========================================================== */

        /* 后轮：0x200 电流帧 */
        BSP_CAN_Send_Drive_Current(motor_l->give_current,
                                   motor_r->give_current);

        /* 前轮：0x1FF 电压帧 */
        BSP_CAN_Send_Steer_Voltage(steer->give_out);

        /* ============================================================
         *  ⑦ 严格等 1ms
         * ------------------------------------------------------------
         *  vTaskDelayUntil 会补偿循环体本身的耗时，周期永远是精确的 1ms。
         * ========================================================== */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}
