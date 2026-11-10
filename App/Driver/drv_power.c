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
 *  Power_Init —— 初始化
 *  调用者：task_chassis.c 的 Task_Chassis_Init()，开机一次
 * ==========================================================================*/
void Power_Init(void)
{
    memset(&g_power, 0, sizeof(Power_t));

    /* 默认"不缩放"，万一算错也不会把车锁死 */
    g_power.scale        = 1.0f;
    g_power.buffer_scale = 1.0f;
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
    /* 协议单位是 0.1W / 0.1J，乘 0.1 换成 W / J */
    g_power.power_w      = (float)power_0p1w  * 0.1f;
    g_power.buffer_j     = (float)buffer_0p1j * 0.1f;
    g_power.last_rx_tick = HAL_GetTick();
    g_power.online       = 1U;
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
    c = POWER_LIMIT_W - POWER_BASE_W;

    g_power.a_coef   = a;
    g_power.b_coef   = b;
    p_full           = a + b + POWER_BASE_W;    /* F(1) = A + B + L */
    g_power.p_pred_w = p_full;

    /* ---- 第 3 步：不超限就不缩 ---- */
    if (p_full <= POWER_LIMIT_W)
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
     * 注意：只有收到过 0x31 才生效。没接功率模块时 buffer_j 是 0，
     *       如果照做会把功率直接压到 0，车根本动不了。 */
    if (g_power.online)
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
