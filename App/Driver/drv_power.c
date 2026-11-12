/**
  ******************************************************************************
  * @file    drv_power.c
  * @brief   底盘功率限制的实现
  *
  *  公式和推导过程写在 drv_power.h 的文件头注释里，这里只讲代码。
  *
  *  单位约定（★ 最容易搞错的地方）：
  *      电流 i_raw ：电调单位 -16384 ~ +16384，对应 -20A ~ +20A
  *                    所以 1A = 819.2 个电调单位（POWER_RAW_PER_AMP）
  *      转速 rpm   ：电调反馈的【转子 rpm】，不做任何换算
  *      功率        ：W（瓦）
  *      能量        ：J（焦耳）
  ******************************************************************************
  */
#include "drv_power.h"
#include "config.h"
#include <string.h>     /* memset */
#include <math.h>       /* sqrtf */

/* 全局唯一的对象。调试时展开 g_power 就能看到全部中间量。 */
Power_t g_power;

/* ============================================================================
 *  ★ 实时监测变量（只给调试器 / FreeMASTER 看，不参与控制）
 * ============================================================================
 *  在 CLion 调试器里把这几个加到 Watch 就能看到实时值。
 * ==========================================================================*/
volatile float    g_chassis_power_w   = 0.0f;  /* 底盘功率 W（按单字节 1W 解析） */
volatile float    g_chassis_buffer_j  = 0.0f;  /* 缓冲能量 J（按小端 0.1J 解析）★ */
volatile float    g_chassis_buffer_max_j = 0.0f; /* 开机以来见到的最大缓冲能量 J */
volatile int16_t  g_power_raw_p       = 0;     /* 前两字节按【大端】拼的原始值 */
volatile int16_t  g_power_raw_z       = 0;     /* 后两字节按【大端】拼的原始值 */
volatile uint32_t g_power_rx_cnt      = 0U;    /* 收到 0x31 的累计帧数 */
volatile uint8_t  g_power_level       = 0U;    /* 当前限功率档位 0~3 */
volatile float    g_power_limit_now_w = 0.0f;  /* 当前档位对应的限功率值 W */

/* ============================================================================
 *  Power_Init —— 初始化
 *  调用者：task_chassis.c 的 Task_Chassis_Init()，开机一次
 * ==========================================================================*/
void Power_Init(void)
{
    memset(&g_power, 0, sizeof(Power_t));

    /* 默认"不缩放"，万一算错也不会把车锁死 */
    g_power.scale        = 1.0f;
    g_power.buffer_scale = 1.0f;

    /* 监测变量清零 */
    g_chassis_power_w        = 0.0f;
    g_chassis_buffer_j       = 0.0f;
    g_chassis_buffer_max_j   = 0.0f;
    g_power_raw_p            = 0;
    g_power_raw_z            = 0;
    g_power_rx_cnt           = 0U;
    g_power_level            = 0U;
    g_power_limit_now_w      = POWER_LV0_W;
}

/* ============================================================================
 *  Power_UpdateFeedback —— 收到 0x31 时写反馈
 *  调用者：bsp_can.c 的 HAL_CAN_RxFifo0MsgPendingCallback()
 *
 *  注意：这个函数在 CAN 中断里执行，只做赋值和乘 0.1，
 *        不要在这里做除法、打印、发 CAN。
 * ==========================================================================*/
void Power_UpdateFeedback(int16_t power_0p1w, int16_t buffer_0p1j)
{
    uint16_t p_raw;
    uint16_t z_raw;
    int16_t  z_le;

    /* 记下 bsp_can.c 按【大端】拼出来的原始值，供调试对比 */
    p_raw = (uint16_t)power_0p1w;
    z_raw = (uint16_t)buffer_0p1j;
    g_power_raw_p  = (int16_t)p_raw;
    g_power_raw_z  = (int16_t)z_raw;
    g_power_rx_cnt = g_power_rx_cnt + 1U;

    /* ---- 缓冲能量：实测是【小端】0.1J ----
     * 你的功率计发 frame[2]=160, frame[3]=15：
     *     大端读: (160<<8)|15 = 40975 -> int16 = -24561 -> x0.1 = -2456 J  ❌
     *     小端读: (15<<8)|160 = 4000   ->          x0.1 =   400 J          ✅
     * 所以这里把字节交换一下再算。 */
    z_le = (int16_t)(((z_raw & 0x00FFU) << 8) | ((z_raw & 0xFF00U) >> 8));
    g_power.buffer_j = (float)z_le * 0.1f;

    /* ---- 底盘功率：实测是【单字节 frame[0]】，单位 1W ----
     * frame[0] 就是大端值的高字节。
     * 例：不转时几十、空转 50~100、下地更高 —— 符合物理。 */
    g_power.power_w = (float)(p_raw >> 8);

    g_power.last_rx_tick = HAL_GetTick();
    g_power.online       = 1U;

    /* 记录见过的最大缓冲能量（判断数据可不可信用） */
    if (g_power.buffer_j > g_power.buffer_max_j)
    {
        g_power.buffer_max_j = g_power.buffer_j;
    }

    /* ---- 同步给调试器看的监测变量 ---- */
    g_chassis_power_w      = g_power.power_w;
    g_chassis_buffer_j     = g_power.buffer_j;
    g_chassis_buffer_max_j = g_power.buffer_max_j;
}

/* ============================================================================
 *  power_level_update —— 按缓冲能量决定限功率档位（带滞环）
 *
 *  思路：缓冲能量就是裁判系统的"余额"。
 *        余额足 -> 正常给功率（跑得快）
 *        余额少 -> 一档档收紧（牺牲速度换不断电）
 *        余额见底 -> 直接停
 *
 *  ★ 为什么要滞环？
 *    如果只用"低于 150 收紧、高于 150 放开"一个阈值，
 *    缓冲就会在 150 附近来回跳，档位一秒钟切好几次。
 *    所以"收紧"和"放开"用不同的阈值，中间留一段不动作的区间。
 *
 *  档位与阈值：
 *      LV0  35W   余额 > 200J 才回到这档
 *      LV1  25W   余额 < 150J 掉到这档，> 200J 回 LV0
 *      LV2  15W   余额 < 100J 掉到这档，> 150J 回 LV1
 *      LV3   0W   余额 <  40J 掉到这档，> 100J 回 LV2
 * ==========================================================================*/
static void power_level_update(void)
{
    float b;

    /* 数据不可信（没在线 / 超出合理范围）-> 退回最高档，纯开环。
     * ★ 不能让坏数据把车锁死在 0W。 */
    if ((g_power.online == 0U) ||
        (g_power.buffer_j < POWER_SANE_J_MIN) ||
        (g_power.buffer_j > POWER_SANE_J_MAX))
    {
        g_power_level       = 0U;
        g_power_limit_now_w = POWER_LV0_W;
        return;
    }

    b = g_power.buffer_j;

    /* ---- 档位迁移（含滞环）---- */
    switch (g_power_level)
    {
    case 0U:
        if (b < POWER_TRIP_LV1)
        {
            g_power_level = 1U;         /* 余额偏低 -> 收紧 */
        }
        break;

    case 1U:
        if (b > POWER_TRIP_LV0)
        {
            g_power_level = 0U;         /* 充回来了 -> 放开 */
        }
        else if (b < POWER_TRIP_LV2)
        {
            g_power_level = 2U;         /* 继续掉 -> 再收紧 */
        }
        break;

    case 2U:
        if (b > POWER_TRIP_LV1)
        {
            g_power_level = 1U;
        }
        else if (b < POWER_TRIP_LV3)
        {
            g_power_level = 3U;         /* 快见底 -> 停 */
        }
        break;

    default:
        if (b > POWER_TRIP_LV2)
        {
            g_power_level = 2U;         /* 回充够了 -> 恢复 */
        }
        break;
    }

    /* ---- 档位 -> 本拍的限功率值 ---- */
    switch (g_power_level)
    {
    case 0U:  g_power_limit_now_w = POWER_LV0_W; break;
    case 1U:  g_power_limit_now_w = POWER_LV1_W; break;
    case 2U:  g_power_limit_now_w = POWER_LV2_W; break;
    default:  g_power_limit_now_w = POWER_LV3_W; break;
    }
}

/* ============================================================================
 *  Power_CalcScale —— 算统一缩放系数 k
 *  调用者：task_chassis.c 的 Task_Chassis_Loop()，1kHz
 *
 *  流程：
 *      1. 把各路电流换算成安培，累加 sum(I^2) 和 sum(I*n)
 *      2. 算出 A、B，以及不缩放时的预测功率 F(1) = A + B + L
 *      3. F(1) <= P  ->  k = 1，直接返回
 *      4. 否则解二次方程，取正根，限制在 0~1
 *      5. （可选）缓冲能量低时再额外压一压
 * ==========================================================================*/
float Power_CalcScale(const int16_t i_raw[], const int16_t rpm[], uint8_t cnt)
{
    float sum_i2;       /* sum(I^2) */
    float sum_in;       /* sum(I*n) */
    float a;            /* 公式里的 A */
    float b;            /* 公式里的 B */
    float c;            /* C = P - L */
    float k;            /* 结果 */
    float p_full;       /* F(1)：不缩放时的预测功率 */
    float p_lim;        /* 本拍实际使用的限功率值（按缓冲能量分档） */
    float disc;         /* 求根公式里的判别式 */
    uint8_t t;

    /* ---- 每拍先复位，保证任何分支返回前状态都是干净的 ---- */
    g_power.limiting     = 0U;
    g_power.scale        = 1.0f;
    g_power.buffer_scale = 1.0f;

    if ((i_raw == 0) || (rpm == 0) || (cnt == 0U))
    {
        return 1.0f;
    }

    if (POWER_LIMIT_ENABLE == 0)
    {
        return 1.0f;        /* 开关关掉，永远不缩 */
    }

    /* ---- 功率模块在线判断（带超时）----
     * online 只置不清，所以这里用 last_rx_tick 判断模块是不是还活着。
     * 掉线后清掉 online，缓冲保护自动跳过，退回"只用功率模型"。 */
    if ((g_power.online != 0U) &&
        ((HAL_GetTick() - g_power.last_rx_tick) > POWER_FB_TIMEOUT_MS))
    {
        g_power.online = 0U;
    }

    /* ---- 第 0.5 步：按缓冲能量决定本拍的限功率档位 ----
     * 缓冲余额足 -> 35W；余额少 -> 逐档收紧到 25 / 15 / 0W。
     * 详见 power_level_update() 的注释。 */
    power_level_update();
    p_lim = g_power_limit_now_w;

    /* ---- 第 1 步：累加 ---- */
    sum_i2 = 0.0f;
    sum_in = 0.0f;

    for (t = 0U; t < cnt; t++)
    {
        /* 电调单位 -> 安培 */
        float amp = (float)i_raw[t] / POWER_RAW_PER_AMP;
        float n   = (float)rpm[t];

        sum_i2 = sum_i2 + (amp * amp);
        sum_in = sum_in + (amp * n);
    }

    /* ---- 第 2 步：算出 A、B 和 F(1) ---- */
    a = POWER_R_OHM  * sum_i2;
    b = POWER_K_COEF * sum_in;
    c = p_lim - POWER_BASE_W;

    g_power.a_coef   = a;
    g_power.b_coef   = b;
    p_full           = a + b + POWER_BASE_W;    /* F(1) = A + B + L */
    g_power.p_pred_w = p_full;

    /* ---- 第 3 步：不超限就不缩 ---- */
    if (p_full <= p_lim)
    {
        k = 1.0f;
    }
    /* ---- 允许功率还不到基线功耗：只能全缩 ---- */
    else if (c <= 0.0f)
    {
        k = 0.0f;
    }
    /* ---- A 约等于 0 时二次方程退化成一阶：B*k <= C ---- */
    else if (a < 0.000001f)
    {
        if (b > 0.000001f)
        {
            k = c / b;
        }
        else
        {
            k = 0.0f;
        }
    }
    /* ---- 第 4 步：正常情况，解 A*k^2 + B*k - C = 0，取正根 ---- */
    else
    {
        disc = (b * b) + (4.0f * a * c);
        if (disc < 0.0f)
        {
            disc = 0.0f;        /* 防止浮点误差导致负数开根号 */
        }
        k = (-b + sqrtf(disc)) / (2.0f * a);
    }

    /* ---- 限制在 0 ~ 1 ---- */
    if (k < 0.0f)
    {
        k = 0.0f;
    }
    if (k > 1.0f)
    {
        k = 1.0f;
    }

#if (POWER_BUFFER_PROTECT == 1)
    /* ---- 第 5 步：缓冲能量保护（可选） ----
     * 规则里缓冲能量低于 0J 会断电 5 秒，所以提前降额留余量。
     *
     * ★ 生效条件（两个都满足才行）：
     *   1. 收到过 0x31（online），且没超时
     *   2. 开机以来见过的最大缓冲能量 > POWER_BUFFER_MIN_J
     *
     *   为什么条件 2 这么重要？
     *     真正的裁判系统开局缓冲能量是满的 60J，所以 buffer_max_j 一定
     *     会远超阈值，保护正常工作。
     *     但如果功率模块【没接裁判系统】或者数据是假的，它可能一直报 0J，
     *     这时若照做就会把功率永久压到 0 —— 表现是
     *     "后轮完全不动，但转向正常"（转向不经过功率限制），非常难查。
     *     加上条件 2 就不会再被这种假数据坑住。 */
    if ((g_power.online != 0U) && (g_power.buffer_max_j > POWER_BUFFER_MIN_J))
    {
        if (g_power.buffer_j <= POWER_BUFFER_MIN_J)
        {
            g_power.buffer_scale = 0.0f;        /* 快耗尽了，完全不给功率 */
        }
        else if (g_power.buffer_j < POWER_BUFFER_LOW_J)
        {
            /* 在两个阈值之间线性过渡：
             *   buffer = MIN_J -> 0.0
             *   buffer = LOW_J -> 1.0 */
            g_power.buffer_scale = (g_power.buffer_j - POWER_BUFFER_MIN_J)
                                 / (POWER_BUFFER_LOW_J - POWER_BUFFER_MIN_J);
        }
        else
        {
            g_power.buffer_scale = 1.0f;
        }

        k = k * g_power.buffer_scale;
    }
#endif

    /* ---- 存结果 ---- */
    g_power.scale    = k;
    g_power.limiting = (k < 0.999f) ? 1U : 0U;

    return k;
}
