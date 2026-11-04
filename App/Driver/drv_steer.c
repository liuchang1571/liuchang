/**
  ******************************************************************************
  * @file    drv_steer.c
  * @brief   GM6020 推齿条 的位置环实现
  *
  *  单位说明（★ 很重要 ★）：
  *     对外（任务层看到的）：齿条位置，单位【毫米】
  *     对内（发给电调的）  ：电压，-30000 ~ +30000
  *     中间桥梁：编码器计数 0~8191
  *
  *  三个单位之间的换算，全部靠 config.h 里的两个标定值：
  *      STEER_CENTER_RAW   齿条居中时的编码器值
  *      STEER_RAW_PER_MM   每毫米多少计数
  ******************************************************************************
  */
#include "drv_steer.h"
#include "config.h"
#include <string.h>     /* memset */

/* 只有一台转向电机，所以直接一个对象，不用数组 */
static Steer_t s_steer;

/* ============================================================================
 *  wrap_enc —— 把任意整数折回 0 ~ 8191
 * ----------------------------------------------------------------------------
 *  什么时候用：算"目标编码器值"的时候。
 *
 *  举例：齿条居中时编码器是 8100，
 *        往左走 22mm（假设每毫米 130 个计数，就是 2860 个计数）
 *        8100 + 2860 = 10960，超过 8191 了
 *        应该折回来：10960 - 8192 = 2768
 *
 *  @note 调用者：Steer_SetTargetMM()
 * ==========================================================================*/
static int16_t wrap_enc(int32_t v)
{
    /* % 是取余数。对 8192 取余，结果一定落在 -8191 ~ 8191 之间 */
    v = v % 8192;

    /* 如果余数是负的，加上一圈就变成正的了 */
    if (v < 0)
    {
        v = v + 8192;
    }

    return v;
}

/* ============================================================================
 *  norm_cnt —— 把编码器差值折算到 ±4096（也就是 ±180 度）
 * ----------------------------------------------------------------------------
 *  这是【位置环最关键的一步】，不写这段电机会疯转。
 *
 *  举例：目标 = 100，实际 = 8100
 *        直接减：(100 - 8100) = -8000
 *                  -> 电机以为"要倒转一大圈"
 *        折算后：-8000 小于 -4096，所以 +8192 -> +192
 *                  -> 正确理解成"只要正转 192 个计数就到"
 *
 *  @note 调用者：Steer_UpdateFeedback()、Steer_Pos_PID_Calc()
 * ==========================================================================*/
static float norm_cnt(int32_t d)
{
    if (d > 4096)
    {
        d = d - 8192;
    }
    if (d < -4096)
    {
        d = d + 8192;
    }
    return d;
}

/* ============================================================================
 *  Steer_Get —— 拿到转向电机对象
 *  调用者：task_chassis.c
 * ==========================================================================*/
Steer_t *Steer_Get(void)
{
    return &s_steer;
}

/* ============================================================================
 *  Steer_Init —— 初始化
 *  调用者：task_chassis.c 的 Task_Chassis_Init()，开机一次
 * ==========================================================================*/
void Steer_Init(float kp, float ki, float kd, int16_t center_raw, float raw_per_mm)
{
    memset(&s_steer, 0, sizeof(Steer_t));

    s_steer.kp = kp;
    s_steer.ki = ki;
    s_steer.kd = kd;

    /* 记下标定参数 */
    s_steer.center_raw = center_raw;
    s_steer.raw_per_mm = raw_per_mm;

    /* 目标先设成"中间"，只是占位。
     * 真正收到第一帧反馈后会被改成"当前位置"，见 UpdateFeedback */
    s_steer.target_raw = center_raw;
    s_steer.actual_raw = center_raw;

    s_steer.last_rx_tick = HAL_GetTick();
}

/* ============================================================================
 *  Steer_UpdateFeedback —— 中断里写反馈
 *  调用者：bsp_can.c 的 HAL_CAN_RxFifo0MsgPendingCallback()，每秒约 1000 次
 * ==========================================================================*/
void Steer_UpdateFeedback(uint16_t angle_raw, int16_t rpm,
                          int16_t torque, uint8_t temp)
{
    float mm;

    /* ------------------------------------------------------------------
     * 【上电保护】★ 这是本文件最重要的几行 ★
     * ------------------------------------------------------------------
     * 第一帧反馈到达的时候，我们还不知道齿条在哪。
     * 如果目标还是"中间"，位置环会立刻输出很大电压，
     * 把齿条猛地推到中间 —— 可能打到手、撞坏齿轮齿条。
     *
     * 所以：第一帧到来时，把目标设成"当前位置"，先原地不动。
     * 等遥控器数据来了，再跟着摇杆走。
     * ------------------------------------------------------------------ */
    if (s_steer.have_fb == 0)
    {
        s_steer.have_fb = 1;
        s_steer.target_raw = angle_raw;      /* 目标 = 当前位置 */
        s_steer.target_mm = 0.0f;            /* 显示用，先给 0 */
        s_steer.integral = 0.0f;             /* 积分清零 */
    }

    /* 保存反馈 */
    s_steer.actual_raw = angle_raw;

    /* 编码器值 -> 毫米
     *   (当前值 - 中位值) / 每毫米计数 = 毫米
     * 注意 raw_per_mm 可能是负数（方向反了），
     * 这里不需要特殊处理，除以负数结果自动是反的，逻辑一致。 */
    mm = (float)(angle_raw - s_steer.center_raw) / s_steer.raw_per_mm;
    s_steer.actual_mm = mm;

    s_steer.actual_rpm = rpm;
    s_steer.torque_cur = torque;
    s_steer.temperature = temp;
    s_steer.last_rx_tick = HAL_GetTick();
    s_steer.online = 1;
}

/* ============================================================================
 *  Steer_SetTargetMM —— 设置目标齿条位置（毫米）
 *  调用者：task_chassis.c 的 Task_Chassis_Loop()，1kHz
 * ==========================================================================*/
void Steer_SetTargetMM(float mm)
{
    int32_t cnt;

    /* ------------------------------------------------------------------
     * 【第一道防线：软件限位】
     * ------------------------------------------------------------------
     * 不管上层传进来什么值（哪怕解算写错，传了 999mm），
     * 这里都强行压到 ±STEER_RACK_MM_MAX（22mm）之内。
     *
     * 为什么必须比物理限位（23mm）小 1mm？
     * 因为程序是"到了就停止出力"，
     * 如果和物理限位一样大，齿条会一直顶在机械块上，
     * 电机持续出力、发热，最后烧电机。 */
    if (mm > STEER_RACK_MM_MAX)
    {
        mm = STEER_RACK_MM_MAX;
    }
    if (mm < -STEER_RACK_MM_MAX)
    {
        mm = -STEER_RACK_MM_MAX;
    }

    /* 记下目标（调试时看这个最直观） */
    s_steer.target_mm = mm;

    /* ------------------------------------------------------------------
     * 毫米 -> 编码器值
     * ------------------------------------------------------------------
     *      目标编码器值 = 中位值 + 毫米数 × 每毫米计数
     * 最后用 wrap_enc 折回 0~8191（因为可能超过一圈） */
    cnt = (int32_t)(mm * s_steer.raw_per_mm);
    s_steer.target_raw = wrap_enc(cnt + s_steer.center_raw);
}

/* ============================================================================
 *  Steer_Pos_PID_Calc —— 位置环核心
 *  调用者：task_chassis.c 的 Task_Chassis_Loop()，1kHz，dt = 0.001f
 * ==========================================================================*/
void Steer_Pos_PID_Calc(float dt)
{
    float err_cnt;     /* 编码器误差（已折算） */
    float err_mm;      /* 毫米误差 */
    float out;         /* 输出 */

    /* ---- 第 0 步：还没收到过反馈 -> 绝对不许输出 ----
     * 因为此时 actual_raw 是假的（初始化时填的中位），
     * 拿它去算会得出一个假误差，把齿条甩出去。 */
    if (s_steer.have_fb == 0)
    {
        s_steer.give_out = 0;
        return;
    }

    /* dt 防呆：防止传进来 0，或者任务卡了很久才跑一次 */
    if (dt <= 0.0f || dt > 0.1f)
    {
        dt = 0.001f;
    }

    /* ------------------------------------------------------------------
     * 【第二道防线：实测超出限位 -> 立刻切断输出】
     * ------------------------------------------------------------------
     * 如果实测位置已经超过 软件限位+2mm，
     * 说明齿条已经顶到机械块上了，再出力只会把东西顶坏。
     * 所以直接输出 0 并清积分。
     * 等外力把齿条推回来，就自动恢复正常。 */
    if (s_steer.actual_mm > STEER_RACK_MM_MAX + 2.0f ||
        s_steer.actual_mm < -STEER_RACK_MM_MAX - 2.0f)
    {
        s_steer.integral = 0.0f;
        s_steer.give_out = 0;
        s_steer.limit_hit = 1;      /* 打上标记，方便调试时看出来 */
        return;
    }
    s_steer.limit_hit = 0;

    /* ---- 第 1 步：算编码器误差 ----
     * 先算"目标 - 实际"，再用 norm_cnt 处理回绕。 */
    err_cnt = norm_cnt(s_steer.target_raw - s_steer.actual_raw);

    /* ---- 第 2 步：编码器误差 -> 毫米误差 ----
     * 除以"每毫米多少计数"就得到毫米数。
     * 例：误差 2860 个计数，每毫米 130 个计数 -> 22mm */
    err_mm = err_cnt / s_steer.raw_per_mm;

    /* ---- 第 3 步：算积分 ----
     * 单位是"毫米·秒"。限幅 ±20 表示：
     * 就算误差一直是 10mm、攒了 2 秒，也只攒到 20。 */
    s_steer.integral = s_steer.integral + err_mm * dt;
    if (s_steer.integral > 20.0f)
    {
        s_steer.integral = 20.0f;
    }
    if (s_steer.integral < -20.0f)
    {
        s_steer.integral = -20.0f;
    }

    /* ---- 第 4 步：算输出 ----
     * kp = 500 的意思是"差 1mm，给 500 的电压"。
     * 差 22mm -> 11000 电压，接近限幅 15000，力度合适。 */
    out = s_steer.kp * err_mm + s_steer.ki * s_steer.integral;

    /* ---- 第 5 步：输出限幅 ---- */
    if (out > STEER_OUT_MAX)
    {
        out = STEER_OUT_MAX;
    }
    if (out < -STEER_OUT_MAX)
    {
        out = -STEER_OUT_MAX;
    }

    /* ---- 第 6 步：存结果 ---- */
    s_steer.give_out = out;
}

/* ============================================================================
 *  Steer_CheckOnline —— 掉线检测
 *  调用者：freertos.c 的 StartDefaultTask()，每 10ms 一次
 * ==========================================================================*/
void Steer_CheckOnline(uint32_t now_tick)
{
    uint32_t elapsed;

    elapsed = now_tick - s_steer.last_rx_tick;

    if (elapsed < 50)
    {
        s_steer.online = 1;
    }
    else
    {
        s_steer.online = 0;
    }
}
