/**
  ******************************************************************************
  * @file    driver_shooter.c
  * @brief   发射机构：2 个摩擦轮（速度环）+ 拨弹盘（位置/速度双环）
  *
  *   拨弹盘的思路来自 gimbal_board 的 driver_shooter.c：
  *     · 位置环输出"速度指令"，再串一级速度环，最后给电流
  *     · 低速区规避：误差小于死区就停；误差大但算出来的速度太小就抬到最小速度
  *     · 速度反馈一阶低通 + 电流变化率限制（抑制嗡嗡声和冲击）
  *     · 卡弹判定：有误差但转速≈0 持续一段时间 -> 回退一格
  ******************************************************************************
  */
#include "driver_shooter.h"
#include "config_shooter.h"
#include "math.h"

Shooter_t g_shooter;

/* ------------------------------------------------------------------ 内部 */
/* 往前走一格 */
static void Feeder_AddStep(int8_t n)
{
    g_shooter.feeder_target_cnt += (float)n * FEEDER_COUNTS_PER_SLOT;
    g_shooter.feeder_step_tick   = HAL_GetTick();
    g_shooter.feeder_block_tick  = 0;
}

/* 清拨弹盘的两个环（模式切换 / 失能时用） */
static void Feeder_ResetLoop(void)
{
    PID_Reset(&g_shooter.feeder_pos_pid);
    PID_Reset(&g_shooter.feeder_spd_pid);

    g_shooter.feeder_spd_filt   = 0.0f;
    g_shooter.feeder_cur_last   = 0;
    g_shooter.feeder_block_tick = 0;
}

/* ------------------------------------------------------------------ 初始化 */
void Shooter_Init(void)
{
    MOTOR3508_Init(FEEDER_3508_ID, &g_shooter.feeder);
    MOTOR3508_Init(FRIC_L_3508_ID, &g_shooter.fric_l);
    MOTOR3508_Init(FRIC_R_3508_ID, &g_shooter.fric_r);

    /* 位置环：输出就是速度指令（rpm） */
    PID_Init(&g_shooter.feeder_pos_pid, FEEDER_POS_KP, 0.0f, 0.0f,
             FEEDER_SPD_LIMIT_RPM, 0.0f);
    /* 速度环：输出电流 */
    PID_Init(&g_shooter.feeder_spd_pid, FEEDER_SPD_KP, FEEDER_SPD_KI, 0.0f,
             FEEDER_CUR_MAX, FEEDER_SPD_I_MAX);
    /* 摩擦轮：速度环 */
    PID_Init(&g_shooter.fric_l_pid, FRIC_VEL_KP, FRIC_VEL_KI, 0.0f,
             FRIC_CUR_MAX, FRIC_VEL_I_MAX);
    PID_Init(&g_shooter.fric_r_pid, FRIC_VEL_KP, FRIC_VEL_KI, 0.0f,
             FRIC_CUR_MAX, FRIC_VEL_I_MAX);

    g_shooter.mode      = 0;
    g_shooter.fire_last = 0;
    g_shooter.mode_tick = 0;

    g_shooter.fric_target = 0.0f;
    g_shooter.fric_l_cur  = 0;
    g_shooter.fric_r_cur  = 0;
    g_shooter.fric_ready  = 0;

    g_shooter.feeder_target_cnt = 0.0f;
    g_shooter.feeder_step_tick  = 0;
    g_shooter.feeder_block_tick = 0;
    g_shooter.feeder_retry_tick = 0;
    g_shooter.feeder_retry_cnt  = 0;
    g_shooter.feeder_cur        = 0;
    g_shooter.feeder_spd_filt   = 0.0f;
    g_shooter.feeder_cur_last   = 0;
    g_shooter.feeder_err        = 0.0f;
    g_shooter.jam_cnt           = 0;
}

/* ------------------------------------------------------------------ 主流程 */
void Shooter_Update(uint8_t s1, uint8_t s2)
{
    uint32_t now = HAL_GetTick();
    uint8_t  mode;
    uint8_t  fire;
    uint8_t  fire_ok;

    /* ==================== 1. 模式 ==================== */
    mode = 0;
    if (s1 == 1)      mode = 1;         /* 单发 */
    else if (s1 == 2) mode = 2;         /* 连发 */

    if (mode != g_shooter.mode)
    {
        /* 进/出"关"：目标对齐当前位置，避免突然转一格 */
        if ((mode == 0) || (g_shooter.mode == 0))
        {
            g_shooter.feeder_target_cnt = (float)g_shooter.feeder.angle_total;
            Feeder_ResetLoop();
        }

        g_shooter.mode              = mode;
        g_shooter.mode_tick         = now;
        g_shooter.feeder_step_tick  = now;
        g_shooter.feeder_retry_tick = 0;
        g_shooter.feeder_retry_cnt  = 0;
        g_shooter.fire_last         = 0;
    }

    /* ==================== 2. 摩擦轮 ==================== */
    {
        float tgt  = (mode != 0) ? FRIC_RPM : 0.0f;
        float step = FRIC_ACCEL_RPM_S * 0.001f;     /* 每 1ms 加多少 */

        /* 目标斜坡：迅速但不阶跃（阶跃会冲电流、抖一下） */
        if (g_shooter.fric_target < tgt)
        {
            g_shooter.fric_target += step;
            if (g_shooter.fric_target > tgt) g_shooter.fric_target = tgt;
        }
        else if (g_shooter.fric_target > tgt)
        {
            g_shooter.fric_target -= step;
            if (g_shooter.fric_target < tgt) g_shooter.fric_target = tgt;
        }

        if (mode == 0)
        {
            /* 关：直接 0 电流（让它自己滑停） */
            g_shooter.fric_l_cur = 0;
            g_shooter.fric_r_cur = 0;
            PID_Reset(&g_shooter.fric_l_pid);
            PID_Reset(&g_shooter.fric_r_pid);
        }
        else
        {
            g_shooter.fric_l_cur = (int16_t)PID_Calculate(
                                        &g_shooter.fric_l_pid,
                                        g_shooter.fric_target * FRIC_DIR_L,
                                        (float)g_shooter.fric_l.speed_rpm);

            g_shooter.fric_r_cur = (int16_t)PID_Calculate(
                                        &g_shooter.fric_r_pid,
                                        g_shooter.fric_target * FRIC_DIR_R,
                                        (float)g_shooter.fric_r.speed_rpm);
        }

        /* 到速判定：两个轮子都要到（用于"没到速不许拨弹"） */
        {
            float need = FRIC_RPM * FRIC_READY_RATIO;

            g_shooter.fric_ready = (uint8_t)(
                (fabsf((float)g_shooter.fric_l.speed_rpm) >= need) &&
                (fabsf((float)g_shooter.fric_r.speed_rpm) >= need));
        }
    }

    /* ==================== 3. 拨弹盘 ==================== */

    /* 3.1 模式关：不出力，目标跟住实际 */
    if (mode == 0)
    {
        g_shooter.feeder_target_cnt = (float)g_shooter.feeder.angle_total;
        g_shooter.feeder_cur        = 0;
        g_shooter.feeder_err        = 0.0f;
        return;
    }

    /* 3.2 反馈超时：不出力（不能拿假位置闭环） */
    if ((!g_shooter.feeder.updated) ||
        ((uint32_t)(now - g_shooter.feeder.last_rx_tick) > 50U))
    {
        g_shooter.feeder_target_cnt = (float)g_shooter.feeder.angle_total;
        g_shooter.feeder_cur        = 0;
        Feeder_ResetLoop();
        return;
    }

    /* 3.3 发射信号：S2 = 1；摩擦轮没到速时先压住 */
    fire = (uint8_t)(s2 == 1);

#if (FRIC_READY_GATE == 1)
    fire_ok = (uint8_t)(fire && g_shooter.fric_ready);
#else
    fire_ok = fire;
#endif

    /* 3.4 单发：上升沿走一格（拨一下打一发，按住不会再打） */
    if (mode == 1)
    {
        if (fire_ok && !g_shooter.fire_last)
        {
            Feeder_AddStep(1);
        }
    }
    /* 3.5 连发：按住就按固定间隔一格一格走 */
    else
    {
        if (fire_ok)
        {
            /* 刚按住：先垫一个间隔，让第一格立刻出 */
            if (!g_shooter.fire_last)
            {
                g_shooter.feeder_step_tick = now - FEEDER_AUTO_INTERVAL_MS;
            }

            if ((uint32_t)(now - g_shooter.feeder_step_tick) >= FEEDER_AUTO_INTERVAL_MS)
            {
                Feeder_AddStep(1);
            }
        }
        else
        {
            /* 松手：重新计时，下次按住第一格立即出 */
            g_shooter.feeder_step_tick = now;
        }
    }

    g_shooter.fire_last = fire_ok;

    /* 3.6 卡弹判定：目标一直追不上（误差大）而且转速≈0 */
    g_shooter.feeder_err = g_shooter.feeder_target_cnt - (float)g_shooter.feeder.angle_total;

    if ((uint32_t)(now - g_shooter.mode_tick) > FEEDER_BLOCK_START_MS)
    {
        if ((fabsf(g_shooter.feeder_err) > FEEDER_POS_DEADBAND) &&
            (fabsf((float)g_shooter.feeder.speed_rpm) < FEEDER_STALL_RPM))
        {
            if (g_shooter.feeder_block_tick == 0)
            {
                g_shooter.feeder_block_tick = now;
            }
            else if ((uint32_t)(now - g_shooter.feeder_block_tick) >= FEEDER_BLOCK_MS)
            {
                /* ★ 卡弹：让机构【真的往后退】FEEDER_BACK_SLOTS 格。
                   做法是"先抵消当前误差，再额外退一格"：
                     target -= err  -> 目标 = 当前位置（误差清零，不会动）
                     target -= 1格  -> 目标 = 当前位置 - 1格，位置环会把它拉回去
                   ★ 如果只写 target -= 1格，在"一格都没走进去"的卡弹情况下
                     误差刚好被抹平，机构一动都不动，卡弹根本解不开。 */
                g_shooter.feeder_target_cnt -= g_shooter.feeder_err
                                             + (float)FEEDER_BACK_SLOTS * FEEDER_COUNTS_PER_SLOT;

                g_shooter.feeder_block_tick  = 0;
                g_shooter.feeder_step_tick   = now;
                g_shooter.jam_cnt++;

                /* 单发：等一会儿自动再试一格（连发靠下一个间隔自然重试） */
                if ((mode == 1) && (g_shooter.feeder_retry_cnt < FEEDER_RETRY_MAX))
                {
                    g_shooter.feeder_retry_tick = now + FEEDER_RETRY_MS;
                    g_shooter.feeder_retry_cnt++;
                }
                else
                {
                    g_shooter.feeder_retry_tick = 0;
                }
            }
        }
        else
        {
            g_shooter.feeder_block_tick = 0;
        }
    }

    /* 3.7 单发卡弹后的自动重试 */
    if ((g_shooter.feeder_retry_tick != 0) &&
        ((int32_t)(now - g_shooter.feeder_retry_tick) >= 0))
    {
        g_shooter.feeder_retry_tick = 0;
        Feeder_AddStep(1);
    }

    /* 走完一格 -> 重试计数清零 */
    if (fabsf(g_shooter.feeder_err) < FEEDER_POS_DEADBAND)
    {
        g_shooter.feeder_retry_cnt = 0;
    }

    /* 3.8 位置环 -> 速度环 -> 电流 */
    {
        float spd_cmd;
        float out;

        /* 速度反馈一阶低通（3508 低速时 rpm 量化噪声大） */
        g_shooter.feeder_spd_filt += FEEDER_SPD_FILTER_ALPHA *
            ((float)g_shooter.feeder.speed_rpm - g_shooter.feeder_spd_filt);

        spd_cmd = PID_Calculate(&g_shooter.feeder_pos_pid,
                                g_shooter.feeder_target_cnt,
                                (float)g_shooter.feeder.angle_total);

        /* 低速区规避：到位就停；误差大但速度太小就抬到最小速度，
           跳过 3508 "嗡嗡抖但几乎不转" 的 1~5 rpm 区间 */
        if (fabsf(g_shooter.feeder_err) < FEEDER_POS_DEADBAND)
        {
            spd_cmd = 0.0f;
        }
        else if (fabsf(spd_cmd) < FEEDER_MIN_RPM)
        {
            spd_cmd = (g_shooter.feeder_err > 0.0f) ? FEEDER_MIN_RPM : -FEEDER_MIN_RPM;
        }

        out = PID_Calculate(&g_shooter.feeder_spd_pid, spd_cmd,
                            g_shooter.feeder_spd_filt);

        /* 电流变化率限制：减小"哒哒"的冲击声和机械震动 */
        if (out > (float)g_shooter.feeder_cur_last + (float)FEEDER_CUR_SLEW)
        {
            out = (float)g_shooter.feeder_cur_last + (float)FEEDER_CUR_SLEW;
        }
        if (out < (float)g_shooter.feeder_cur_last - (float)FEEDER_CUR_SLEW)
        {
            out = (float)g_shooter.feeder_cur_last - (float)FEEDER_CUR_SLEW;
        }

        if (out >  FEEDER_CUR_MAX) out =  FEEDER_CUR_MAX;
        if (out < -FEEDER_CUR_MAX) out = -FEEDER_CUR_MAX;

        g_shooter.feeder_cur_last = (int16_t)out;
        g_shooter.feeder_cur      = g_shooter.feeder_cur_last;
    }
}

/* ------------------------------------------------------------------ 失能 */
void Shooter_SafeOutput(void)
{
    g_shooter.fric_l_cur  = 0;
    g_shooter.fric_r_cur  = 0;
    g_shooter.feeder_cur  = 0;

    g_shooter.fric_target = 0.0f;
    g_shooter.fric_ready  = 0;

    g_shooter.mode      = 0;
    g_shooter.fire_last = 0;

    g_shooter.feeder_target_cnt = (float)g_shooter.feeder.angle_total;
    g_shooter.feeder_retry_tick = 0;
    g_shooter.feeder_retry_cnt  = 0;

    PID_Reset(&g_shooter.fric_l_pid);
    PID_Reset(&g_shooter.fric_r_pid);
    Feeder_ResetLoop();
}
