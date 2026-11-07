/**
  ******************************************************************************
  * @file    driver_gimbal.c
  * @brief   云台：yaw = GM6020（关节伺服），小 pitch = M3508（关节闭环）
  *
  *   yaw  ：关节角误差 -> 角速度参考 -> 电压        （反馈=6020 绝对编码器）
  *   pitch：关节角误差 -> 转子 rpm  -> 电流          （反馈=3508 累加角度）
  *
  *   ★ 现在两套 PID 的 kp/ki 全是 0：环输出恒 0，电机不出力。
  *     这样可以直接推摇杆验证"目标累加 / 限位 / 编码器读数"，见 README。
  ******************************************************************************
  */
#include "driver_gimbal.h"
#include "config_gimbal.h"
#include "math.h"

Gimbal_t g_gimbal;

volatile uint8_t g_vehicle_fault = 0;

/* 关节角速度(deg/s) -> 转子 rpm：rotor_rpm = 19.0/(6*K) * joint_dps */
#define PITCH_ROTOR_PER_JOINT_DPS  (MOTOR3508_GEAR_RATIO / (6.0f * PITCH_GEAR_JOINT_PER_OUT))

/* ------------------------------------------------------------------ 工具 */
float Gimbal_Wrap180(float deg)
{
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

static float Gimbal_ClampYaw(float deg)
{
    if (deg > YAW_JOINT_LIMIT_MAX) return YAW_JOINT_LIMIT_MAX;
    if (deg < YAW_JOINT_LIMIT_MIN) return YAW_JOINT_LIMIT_MIN;
    return deg;
}

static float Gimbal_ClampPitch(float deg)
{
    if (deg > PITCH_LIMIT_MAX) return PITCH_LIMIT_MAX;
    if (deg < PITCH_LIMIT_MIN) return PITCH_LIMIT_MIN;
    return deg;
}

/* 小 pitch 实测关节角（定零之前不可信，返回 0） */
float Gimbal_PitchJointDeg(void)
{
    if (!g_gimbal.pitch_zero_ready) return 0.0f;

    return (g_gimbal.pitch_motor.angle_deg - g_gimbal.pitch_zero_out_deg)
           * PITCH_GEAR_JOINT_PER_OUT;
}

/* ------------------------------------------------------------------ 初始化 */
void Gimbal_Init(void)
{
    GM6020_Init(&g_gimbal.yaw_motor, YAW_6020_ID);
    MOTOR3508_Init(PITCH_3508_ID, &g_gimbal.pitch_motor);

    g_gimbal.yaw_target            = 0.0f;
    g_gimbal.pitch_target          = 0.0f;
    g_gimbal.yaw_rel               = 0.0f;
    g_gimbal.yaw_target_ready      = 0;

    g_gimbal.pitch_zero_out_deg    = 0.0f;
    g_gimbal.pitch_zero_ready      = 0;

    g_gimbal.pitch_joint_deg       = 0.0f;
    g_gimbal.yaw_volt              = 0;
    g_gimbal.pitch_cur             = 0;
    g_gimbal.yaw_limit_hit         = 0;
    g_gimbal.pitch_limit_hit       = 0;

    /* 四个环：现在全部 0，先验证角度 */
    PID_Init(&g_gimbal.yaw_pid,       YAW_POS_KP, YAW_POS_KI, 0.0f,
             YAW_POS_OUT_MAX, YAW_POS_I_MAX);
    PID_Init(&g_gimbal.yaw_vel_pid,   YAW_VEL_KP, YAW_VEL_KI, 0.0f,
             YAW_VEL_OUT_MAX, YAW_VEL_I_MAX);

    /* pitch 位置环的 kp 里已经含了 "关节度 -> 转子 rpm" 的换算和方向 */
    PID_Init(&g_gimbal.pitch_pid,     PITCH_POS_KP * PITCH_ROTOR_PER_JOINT_DPS, 0.0f, 0.0f,
             PITCH_POS_OUT_MAX, 0.0f);
    PID_Init(&g_gimbal.pitch_vel_pid, PITCH_VEL_KP, PITCH_VEL_KI, 0.0f,
             PITCH_VEL_OUT_MAX, PITCH_VEL_I_MAX);
}

/* ------------------------------------------------------- 关节角（每 1ms） */
void Gimbal_UpdateJointAngle(void)
{
    if (!g_gimbal.yaw_motor.updated) return;

    /* 6020 是单圈绝对编码器：直接算，不累加、不漂、不受上电位置影响 */
    g_gimbal.yaw_rel = Gimbal_Wrap180(
        (g_gimbal.yaw_motor.angle_deg - YAW_ZERO_ENC_DEG) * YAW_JOINT_PER_ENC);

    /* ★ 第一帧反馈到达后，把目标对齐到当前角度。
       否则 PID 一使能，云台会从"上电位置"猛甩到 0°。 */
    if (!g_gimbal.yaw_target_ready)
    {
        g_gimbal.yaw_target       = Gimbal_ClampYaw(g_gimbal.yaw_rel);
        g_gimbal.yaw_target_ready = 1;
    }
}

/* --------------------------------------------------- 摇杆 -> 目标（增量式） */
void Gimbal_StickUpdate(int16_t ch0, int16_t ch1)
{
    float d0 = (float)((int32_t)ch0 - GIMBAL_CH_MID);
    float d1 = (float)((int32_t)ch1 - GIMBAL_CH_MID);

    /* 死区：摇杆松手回不到精确的 1024，不加会自己慢慢转 */
    if (fabsf(d0) <= (float)GIMBAL_DEAD_ZONE_RAW) d0 = 0.0f;
    else                                           d0 /= GIMBAL_CH_RANGE;

    if (fabsf(d1) <= (float)GIMBAL_DEAD_ZONE_RAW) d1 = 0.0f;
    else                                           d1 /= GIMBAL_CH_RANGE;

    /* 推着转、松手停：每 1ms 往目标上加一点 */
    g_gimbal.yaw_target   += -d0 * YAW_RATE_S   * GIMBAL_PERIOD_S;
    g_gimbal.pitch_target +=  d1 * PITCH_RATE_S * GIMBAL_PERIOD_S;

    /* 软限位 */
    g_gimbal.yaw_target   = Gimbal_ClampYaw(g_gimbal.yaw_target);
    g_gimbal.pitch_target = Gimbal_ClampPitch(g_gimbal.pitch_target);

    /* 调试标志：目标贴在限位上 */
    g_gimbal.yaw_limit_hit = (uint8_t)(
        (g_gimbal.yaw_target >= YAW_JOINT_LIMIT_MAX - 0.05f) ||
        (g_gimbal.yaw_target <= YAW_JOINT_LIMIT_MIN + 0.05f));

    g_gimbal.pitch_limit_hit = (uint8_t)(
        (g_gimbal.pitch_target >= PITCH_LIMIT_MAX - 0.05f) ||
        (g_gimbal.pitch_target <= PITCH_LIMIT_MIN + 0.05f));
}

/* ------------------------------------------------------- 保护态：目标跟随 */
void Gimbal_TrackCurrent(void)
{
    if (g_gimbal.yaw_motor.updated)
    {
        g_gimbal.yaw_target = Gimbal_ClampYaw(g_gimbal.yaw_rel);
    }
    if (g_gimbal.pitch_zero_ready)
    {
        g_gimbal.pitch_target = Gimbal_ClampPitch(Gimbal_PitchJointDeg());
    }
}

/* ------------------------------------------------------------- 四个环 */
void Gimbal_Control(void)
{
    /* ==================== yaw ==================== */
    if ((!g_gimbal.yaw_motor.updated) ||
        ((HAL_GetTick() - g_gimbal.yaw_motor.last_rx_tick) > GIMBAL_FB_TIMEOUT_MS))
    {
        /* 没反馈 / 反馈超时：不发电压（绝不能用假的关节角去闭环） */
        GM6020_SetVoltage(&g_gimbal.yaw_motor, 0.0f);
        g_gimbal.yaw_volt = 0;
    }
    else
    {
        float rate_ref;
        float rate_fb;
        float volt;

        /* 位置环：关节角误差 -> 关节角速度参考 */
        rate_ref = PID_Calculate(&g_gimbal.yaw_pid,
                                 g_gimbal.yaw_target, g_gimbal.yaw_rel);

        /* 速度环的反馈是"编码器 deg/s"，参考值先换算过去 */
        rate_ref = rate_ref / YAW_JOINT_PER_ENC;
        rate_fb  = g_gimbal.yaw_motor.vel_dps;

        volt = PID_Calculate(&g_gimbal.yaw_vel_pid, rate_ref, rate_fb);
        volt *= YAW_ESC_DIR;

        GM6020_SetVoltage(&g_gimbal.yaw_motor, volt);   /* 只写 volt_cmd，不发帧 */
        g_gimbal.yaw_volt = g_gimbal.yaw_motor.volt_cmd;
    }

    /* ==================== 小 pitch ==================== */
    if (!g_gimbal.pitch_zero_ready)
    {
        /* 3508 没有绝对编码器：收到第一帧反馈后定零。
           上电姿态就是 PITCH_BOOT_JOINT_DEG（= -15°，靠重力落在软限位最低点）。 */
        if (!g_gimbal.pitch_motor.updated)
        {
            g_gimbal.pitch_cur = 0;
            return;
        }

        g_gimbal.pitch_zero_out_deg = g_gimbal.pitch_motor.angle_deg
                                    - PITCH_BOOT_JOINT_DEG / PITCH_GEAR_JOINT_PER_OUT;
        g_gimbal.pitch_zero_ready = 1;

        /* 定零完成：目标就直接放在"当前姿态"，PID 使能瞬间不会动 */
        g_gimbal.pitch_target     = Gimbal_ClampPitch(PITCH_BOOT_JOINT_DEG);
    }

    /* 反馈超时：编码器/总线挂了，绝不能再按假数据算 */
    if ((HAL_GetTick() - g_gimbal.pitch_motor.last_rx_tick) > GIMBAL_FB_TIMEOUT_MS)
    {
        g_gimbal.pitch_cur = 0;
        return;
    }

    {
        float fb = Gimbal_PitchJointDeg();
        float rpm_ref;
        float cur;

        g_gimbal.pitch_joint_deg = fb;

        /* 位置环：误差(deg) -> 转子 rpm 目标（kp 里已含换算和方向） */
        rpm_ref = PID_Calculate(&g_gimbal.pitch_pid, g_gimbal.pitch_target, fb);

        /* 速度环：转子 rpm -> 电流 */
        cur = PID_Calculate(&g_gimbal.pitch_vel_pid, rpm_ref,
                            (float)g_gimbal.pitch_motor.speed_rpm);

        cur += PITCH_GRAV_FF_CUR;               /* 重力前馈，先 0 */

        if (cur >  PITCH_CUR_MAX) cur =  PITCH_CUR_MAX;
        if (cur < -PITCH_CUR_MAX) cur = -PITCH_CUR_MAX;

        g_gimbal.pitch_cur = (int16_t)cur;
    }
}

/* ------------------------------------------------------------- 失能 */
void Gimbal_SafeOutput(void)
{
    g_gimbal.yaw_volt  = 0;
    g_gimbal.pitch_cur = 0;

    GM6020_SetVoltage(&g_gimbal.yaw_motor, 0.0f);   /* volt_cmd = 0 */
}
