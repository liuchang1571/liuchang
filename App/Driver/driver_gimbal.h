/**
  ******************************************************************************
  * @file    driver_gimbal.h
  * @brief   云台：yaw = GM6020（关节伺服），小 pitch = M3508（关节闭环）
  *
  *   ★ yaw 是【关节角】闭环：反馈就是 6020 的绝对编码器（云台相对车身），
  *     不是 IMU 的世界角。因为 C 板装在底盘上，IMU 测的是车身，用不了。
  *   ★ 本文件只算输出，不发 CAN：
  *       yaw_volt  -> task_chassis.c 里 CAN_Send4(&hcan1, 0x1FF, ...)
  *       pitch_cur -> task_chassis.c 里 CAN_Send4(&hcan1, 0x200, ...)
  ******************************************************************************
  */
#ifndef __DRIVER_GIMBAL_H
#define __DRIVER_GIMBAL_H

#include "main.h"
#include "bsp_motor_gm6020.h"
#include "bsp_motor_dji3508.h"
#include "driver_pid.h"

typedef struct
{
    /* ---------- 电机对象（都在 CAN1） ---------- */
    GM6020_t  yaw_motor;
    DJI3508_t pitch_motor;

    /* ---------- 四个环 ---------- */
    PID_t yaw_pid;          /* 关节角误差(deg) -> 关节角速度(deg/s) */
    PID_t yaw_vel_pid;      /* 角速度(deg/s)   -> 电压 */
    PID_t pitch_pid;        /* 关节角误差(deg) -> 转子 rpm */
    PID_t pitch_vel_pid;    /* 转子 rpm        -> 电流 */

    /* ---------- 目标（都是关节角，单位度） ---------- */
    float yaw_target;
    float pitch_target;

    /* ---------- 实测 ---------- */
    float yaw_rel;              /* yaw 关节角，±180 折算后 */
    uint8_t yaw_target_ready;   /* 第一帧编码器反馈后，把目标对齐到当前角度 */

    /* ---------- pitch 上电定零（3508 没有绝对编码器） ---------- */
    float    pitch_zero_out_deg;    /* 关节 0° 对应 3508 输出轴的角度 */
    uint8_t  pitch_zero_ready;

    /* ---------- 调试 ---------- */
    float    pitch_joint_deg;
    int16_t  yaw_volt;          /* 本周期要发出去的电压 */
    int16_t  pitch_cur;         /* 本周期要发出去的电流 */
    uint8_t  yaw_limit_hit;     /* 1 = 目标贴在 ±90 限位上 */
    uint8_t  pitch_limit_hit;   /* 1 = 目标贴在 -15/+25 限位上 */
}Gimbal_t;

extern Gimbal_t g_gimbal;

/* 全车失能标志：1 = 所有电机输出 0。
   现在恒为 0；以后接上陀螺仪，用 "pitch/roll > 80° 置 1、< 50° 清 0" 来驱动它。 */
extern volatile uint8_t g_vehicle_fault;

/* 初始化（在 Init_Task 里调一次） */
void Gimbal_Init(void);

/* 每 1ms 调一次：更新 yaw_rel（绝对编码器） */
void Gimbal_UpdateJointAngle(void);

/* 每 1ms 调一次：摇杆 -> 目标（增量式：推着转、松手停） */
void Gimbal_StickUpdate(int16_t ch0, int16_t ch1);

/* 保护态：目标跟住实测，恢复时不会弹回原位置 */
void Gimbal_TrackCurrent(void);

/* 每 1ms 调一次：跑四个环，填 yaw_volt / pitch_cur */
void Gimbal_Control(void);

/* 输出清 0（失能） */
void Gimbal_SafeOutput(void);

/* 小 pitch 实测关节角（定零前返回 0） */
float Gimbal_PitchJointDeg(void);

/* 折算到 ±180 */
float Gimbal_Wrap180(float deg);

#endif /* __DRIVER_GIMBAL_H */
