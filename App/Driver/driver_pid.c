/**
  ******************************************************************************
  * @file    driver_pid.c
  * @brief   通用 PID（云台四个环用）
  ******************************************************************************
  */
#include "driver_pid.h"

static float PID_Constrain(float v, float min, float max)
{
    if (v < min) return min;
    if (v > max) return max;
    return v;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_out, float max_iout)
{
    if (pid == 0) return;

    pid->kp       = kp;
    pid->ki       = ki;
    pid->kd       = kd;
    pid->max_out  = max_out;
    pid->max_iout = max_iout;

    pid->target   = 0.0f;
    pid->out      = 0.0f;
    pid->err      = 0.0f;
    pid->err_last = 0.0f;
    pid->err_sum  = 0.0f;
}

/* 清历史：积分、上次误差、输出都归零，但 kp/ki/kd 和限幅保留 */
void PID_Reset(PID_t *pid)
{
    if (pid == 0) return;

    pid->target   = 0.0f;
    pid->out      = 0.0f;
    pid->err      = 0.0f;
    pid->err_last = 0.0f;
    pid->err_sum  = 0.0f;
}

/**
  * @brief 算一拍输出
  * @param pid       对象
  * @param target    目标值
  * @param feedback  实测值
  * @retval 已限幅的输出（-max_out ~ +max_out）
  *
  * @note  没有乘 dt：按"每 1ms 一拍"设计，ki 里已经含了 dt 的量纲。
  */
float PID_Calculate(PID_t *pid, float target, float feedback)
{
    float tmp_sum;
    float out_try;

    if (pid == 0) return 0.0f;

    pid->target = target;
    pid->err    = target - feedback;

    if (pid->max_iout > 0.0f)
    {
        tmp_sum = PID_Constrain(pid->err_sum + pid->err,
                                -pid->max_iout, pid->max_iout);

        out_try = pid->kp * pid->err
                + pid->ki * tmp_sum
                + pid->kd * (pid->err - pid->err_last);

        /* 输出已经顶到限幅、而且误差还在往同方向推 -> 积分不累加（抗饱和） */
        if (!((out_try >= pid->max_out && pid->err > 0.0f) ||
              (out_try <= -pid->max_out && pid->err < 0.0f)))
        {
            pid->err_sum = tmp_sum;
        }
    }

    pid->out      = pid->kp * pid->err
                  + pid->ki * pid->err_sum
                  + pid->kd * (pid->err - pid->err_last);
    pid->err_last = pid->err;

    return PID_Constrain(pid->out, -pid->max_out, pid->max_out);
}
