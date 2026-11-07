/**
  ******************************************************************************
  * @file    driver_pid.h
  * @brief   通用 PID（云台四个环用）
  *
  *   和 drv_motor.c / drv_steer.c 里那两份"内联 PID"不同：
  *     那两份是跟着底盘标定好的，本文件不去动它们；
  *     云台这边用这个通用版本（一份代码，四个环共用）。
  *
  *   特点：
  *     · 输入输出都是 float，单位由调用者定
  *     · 积分限幅（max_iout）+ 输出限幅（max_out）
  *     · 输出饱和且误差同向时，积分不累加（抗饱和）
  *     · 没有乘 dt：误差按"每 1ms 一拍"累加，ki 里已经含了 dt
  ******************************************************************************
  */
#ifndef __DRIVER_PID_H
#define __DRIVER_PID_H

#include "main.h"

typedef struct
{
    float kp, ki, kd;
    float max_out;          /* 输出限幅（对称 ±） */
    float max_iout;         /* 积分累加值限幅（对称 ±），0 = 不要积分 */

    float target;
    float out;

    float err, err_last, err_sum;
}PID_t;

void  PID_Init(PID_t *pid, float kp, float ki, float kd, float max_out, float max_iout);
float PID_Calculate(PID_t *pid, float target, float feedback);

/* 清掉历史（积分/上次误差/输出）：模式切换、失能恢复时用，
   避免带着旧积分突然出力 */
void  PID_Reset(PID_t *pid);

#endif /* __DRIVER_PID_H */
