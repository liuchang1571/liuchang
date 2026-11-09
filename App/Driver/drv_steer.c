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
 *  标定用的三个变量（实测后填，给以后的软件限位当边界）
 *  定义在这里、声明在 drv_steer.h，所以全工程和调试器都能看到。
 *  详细用法见 drv_steer.h 里的说明。
 *
 *  ★ __attribute__((used)) 是必须加的，不是装饰：
 *    本工程开了 -fdata-sections + --gc-sections（见 CMakeLists / 链接选项），
 *    链接器会把"没有任何代码引用"的变量当垃圾删掉。
 *    这三个变量就是典型的"只给调试器用、没有代码引用"的变量，
 *    不加 used 的话它们在 ELF 里根本不存在，你在调试器里也就找不到。
 * ==========================================================================*/
volatile int16_t g_steer_raw_center __attribute__((used)) = 6552;   /* 正中间（实测） */
volatile int16_t g_steer_raw_left   __attribute__((used)) = 7742;   /* 最左端，左转（实测） */
volatile int16_t g_steer_raw_right  __attribute__((used)) = 5358;   /* 最右端，右转（实测） */

/* 当前编码值镜像：只为了调试时能一眼看到读数，不参与任何控制 */
volatile int16_t g_steer_raw_now __attribute__((used)) = 0;

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
 *  steer_limit_init —— 把实测的三个编码值变成限位区间（上电跑一次）
 * ----------------------------------------------------------------------------
 *  调用者：Steer_Init()
 *
 *  实测值就写在文件开头那三个全局变量里：
 *        中间 6552 / 最左（左转）7742 / 最右（右转）5358
 *
 *  做两件事：
 *    1. 把两端折算成"相对中位的计数"，得到区间 [lo, hi]
 *    2. 检查这几个值是不是自相矛盾，矛盾就整体关闭限位
 *
 *  ★ 为什么要检查？
 *    参考点用的是 config.h 里的 STEER_CENTER_RAW（= 6552）。
 *    如果它和实测中位对不上，限位就会在错误的位置生效 ——
 *    这时宁可关闭限位、退回 ±22mm，也不要在错的地方限。
 * ==========================================================================*/
static void steer_limit_init(void)
{
    int32_t dl;
    int32_t dr;
    int32_t span;
    float   dc;

    /* 先全部关掉，检查通过再打开 */
    s_steer.limit_enabled = 0;
    s_steer.limit_cfg_bad = 0;
    s_steer.lim_delta_lo  = 0;
    s_steer.lim_delta_hi  = 0;

    if (STEER_RAW_LIMIT_ENABLE == 0)
    {
        return;
    }

    /* 三个值都得是有效编码值 */
    if ((g_steer_raw_center < 0) || (g_steer_raw_left < 0) || (g_steer_raw_right < 0))
    {
        return;
    }

    /* ---- 检查 0：实测中位必须和 config 里的 STEER_CENTER_RAW 一致 ----
     * 这两者分别是"限位的参考点"和"毫米换算的参考点"，
     * 不一致的话两套坐标会打架。差 100 计数（约 2mm）以上就判为填错。 */
    dc = norm_cnt((int32_t)g_steer_raw_center - (int32_t)s_steer.center_raw);
    if ((dc > 100.0f) || (dc < -100.0f))
    {
        s_steer.limit_cfg_bad = 1;
        return;
    }

    if (g_steer_raw_left == g_steer_raw_right)
    {
        s_steer.limit_cfg_bad = 1;
        return;
    }

    /* ---- 两端折算成"相对中位的计数"----
     * 你的数：left 7742 - 6552 = +1190
     *         right 5358 - 6552 = -1194 */
    dl = (int32_t)norm_cnt((int32_t)g_steer_raw_left  - (int32_t)s_steer.center_raw);
    dr = (int32_t)norm_cnt((int32_t)g_steer_raw_right - (int32_t)s_steer.center_raw);

    /* ---- 检查 1：中位必须夹在两端之间 -> 折算后一正一负 ---- */
    if (!(((dl < 0) && (dr > 0)) || ((dl > 0) && (dr < 0))))
    {
        s_steer.limit_cfg_bad = 1;
        return;
    }

    /* ---- 检查 2：行程不能太窄（< 100 计数肯定是填错了）---- */
    span = dr - dl;
    if (span < 0)
    {
        span = -span;
    }
    if (span < 100)
    {
        s_steer.limit_cfg_bad = 1;
        return;
    }

    /* ---- 通过：记下区间 ---- */
    s_steer.lim_delta_lo = (int16_t)((dl < dr) ? dl : dr);
    s_steer.lim_delta_hi = (int16_t)((dl < dr) ? dr : dl);
    s_steer.limit_enabled = 1;
}

/* ============================================================================
 *  steer_clamp_raw —— 把目标编码值钳进实测两端之内
 * ----------------------------------------------------------------------------
 *  调用者：Steer_SetTargetMM()
 *
 *  用你的数字算一遍（中位 6552，两端 5358 / 7742，内缩 55）：
 *  折算区间 = [-1194, +1190]，内缩后 = [-1139, +1135]
 *
 *      目标 6552（中间）-> 落在区间内，原样放行
 *      目标 7742（左端）-> +1190 > 1135，钳到 +1135 -> 7687（离极限约 1mm）
 *      目标 5358（右端）-> -1194 < -1139，钳到 -1139 -> 5413
 *      目标 2000（离谱值）-> 折算后远超下界，同样钳到 5413
 * ==========================================================================*/
static int16_t steer_clamp_raw(int16_t raw)
{
    int32_t d;
    int32_t lo;
    int32_t hi;

    /* 限位没生效（没填 / 填错 / 被关掉）-> 原样放行，退回旧行为 */
    if (s_steer.limit_enabled == 0)
    {
        return raw;
    }

    /* 把"内缩 1mm"算进来：下界往上抬，上界往下压 */
    lo = (int32_t)s_steer.lim_delta_lo + (int32_t)STEER_RAW_MARGIN_CNT;
    hi = (int32_t)s_steer.lim_delta_hi - (int32_t)STEER_RAW_MARGIN_CNT;

    /* 防呆：万一区间特别窄，内缩后反了，就退化成中点 */
    if (lo > hi)
    {
        lo = ((int32_t)s_steer.lim_delta_lo + (int32_t)s_steer.lim_delta_hi) / 2;
        hi = lo;
    }

    d = (int32_t)norm_cnt((int32_t)raw - (int32_t)s_steer.center_raw);

    if (d < lo)
    {
        d = lo;
    }
    if (d > hi)
    {
        d = hi;
    }

    return wrap_enc((int32_t)s_steer.center_raw + d);
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

    /* ---- 编码值软件限位：用实测的三端值算出限位区间 ----
     * ★ 必须在 center_raw 赋值之后调用（它拿 center 当参考点）。
     * ★ 那三个全局变量在这里被真正读取，所以不会被链接器当垃圾删掉。 */
    steer_limit_init();

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

    /* 镜像一份给调试器看（标定的时候就是读这个数）
     * ★ 不参与任何控制，纯粹为了方便观察。 */
    g_steer_raw_now = (int16_t)angle_raw;

    /* 编码器值 -> 毫米
     *   (当前值 - 中位值) / 每毫米计数 = 毫米
     * 注意 raw_per_mm 可能是负数（方向反了），
     * 这里不需要特殊处理，除以负数结果自动是反的，逻辑一致。 */
    mm = (float)(angle_raw - s_steer.center_raw) / s_steer.raw_per_mm;
    s_steer.actual_mm = mm;

    /* 相对中位的计数（编码值限位判断用；norm_cnt 已处理 0/8191 回绕） */
    s_steer.actual_delta = (int16_t)norm_cnt((int32_t)angle_raw - (int32_t)s_steer.center_raw);

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

    /* ------------------------------------------------------------------
     * 【第二道防线：编码值限位】
     * ------------------------------------------------------------------
     * 上面那道是按"毫米"钳的，依赖 STEER_RAW_PER_MM 标定得准。
     * 这一道直接按实测编码值钳，不怕标定误差：
     * 不管前面算出什么目标，都压到实测的两端之内。 */
    s_steer.target_raw = steer_clamp_raw(s_steer.target_raw);

    /* 限位可能把目标拉回来了，这里按最终目标反算一次毫米，
     * 让调试器里 target_mm 和 target_raw 对得上。 */
    if (s_steer.raw_per_mm != 0.0f)
    {
        s_steer.target_mm = norm_cnt((int32_t)s_steer.target_raw - (int32_t)s_steer.center_raw)
                            / s_steer.raw_per_mm;
    }
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
     * 【第三道防线：实测位置已经超出限位 -> 立刻切断输出】
     * ------------------------------------------------------------------
     * 目标虽然钳住了，但齿条可能被外力推出去（撞到东西、有人推车），
     * 或者编码值跳变。这时继续出力只会把齿轮齿条顶坏。
     * 所以直接输出 0 并清积分，等位置回到范围内就自动恢复。
     *
     * 有编码值限位就用它（主防线），没启用就退回旧的 ±22mm 判断。 */
    if (s_steer.limit_enabled)
    {
        int32_t d;

        d = (int32_t)s_steer.actual_delta;

        if ((d > (int32_t)s_steer.lim_delta_hi + (int32_t)STEER_RAW_GUARD_CNT) ||
            (d < (int32_t)s_steer.lim_delta_lo - (int32_t)STEER_RAW_GUARD_CNT))
        {
            s_steer.integral = 0.0f;
            s_steer.give_out = 0;
            s_steer.limit_hit = 1;      /* 打上标记，方便调试时看出来 */
            s_steer.raw_limit_hit = 1;
            return;
        }
        s_steer.raw_limit_hit = 0;
    }
    else
    {
        /* 旧行为：按毫米判断（STEER_RAW_PER_MM 不准时可能误判，
         * 所以只在编码值限位没启用时用它） */
        if (s_steer.actual_mm > STEER_RACK_MM_MAX + 2.0f ||
            s_steer.actual_mm < -STEER_RACK_MM_MAX - 2.0f)
        {
            s_steer.integral = 0.0f;
            s_steer.give_out = 0;
            s_steer.limit_hit = 1;
            return;
        }
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
