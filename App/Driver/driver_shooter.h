/**
  ******************************************************************************
  * @file    driver_shooter.h
  * @brief   发射机构：2 个摩擦轮（速度环）+ 拨弹盘（位置/速度双环）
  *
  *   模式：
  *     S1 = 1 单发：摩擦轮起转等待；S2 拨到 1 一次 -> 拨弹盘走一格
  *     S1 = 2 连发：摩擦轮起转等待；S2 按住       -> 每 333ms 走一格（3 发/秒）
  *     S1 其它      ：全部停
  *
  *   ★ 本文件只算电流，不发 CAN：
  *       feeder_cur / fric_l_cur / fric_r_cur 由 task_chassis.c 填进 CAN1 的 0x200。
  ******************************************************************************
  */
#ifndef __DRIVER_SHOOTER_H
#define __DRIVER_SHOOTER_H

#include "main.h"
#include "bsp_motor_dji3508.h"
#include "driver_pid.h"

typedef struct
{
    /* ---------- 电机（都在 CAN1） ---------- */
    DJI3508_t feeder;       /* 拨弹盘 */
    DJI3508_t fric_l;       /* 摩擦轮左 */
    DJI3508_t fric_r;       /* 摩擦轮右 */

    /* ---------- PID ---------- */
    PID_t feeder_pos_pid;   /* 位置(计数) -> 速度(rpm) */
    PID_t feeder_spd_pid;   /* 速度(rpm)   -> 电流 */
    PID_t fric_l_pid;       /* 速度(rpm)   -> 电流 */
    PID_t fric_r_pid;

    /* ---------- 模式 ---------- */
    uint8_t mode;           /* 0=关 1=单发 2=连发 */
    uint8_t fire_last;      /* 上一拍"允许发射"，做边沿检测 */
    uint32_t mode_tick;     /* 进入当前模式的时刻（卡弹判定的宽限起点） */

    /* ---------- 摩擦轮 ---------- */
    float   fric_target;    /* 斜坡后的目标转速（转子 rpm，带方向） */
    int16_t fric_l_cur;
    int16_t fric_r_cur;
    uint8_t fric_ready;     /* 1 = 两个轮子都到速 */

    /* ---------- 拨弹盘 ---------- */
    float    feeder_target_cnt;     /* 目标累计计数 */
    uint32_t feeder_step_tick;      /* 上一格命令的时刻 */
    uint32_t feeder_block_tick;     /* 卡弹确认计时 */
    uint32_t feeder_retry_tick;     /* 单发卡弹后的重试时刻 */
    uint8_t  feeder_retry_cnt;      /* 连续重试次数 */
    int16_t  feeder_cur;
    float    feeder_spd_filt;       /* 速度反馈低通 */
    int16_t  feeder_cur_last;       /* 上一拍电流（做变化率限制） */

    /* ---------- 调试 ---------- */
    float    feeder_err;            /* 目标 - 实际（计数） */
    uint16_t jam_cnt;               /* 卡弹次数 */
}Shooter_t;

extern Shooter_t g_shooter;

void Shooter_Init(void);

/* 每 1ms 调一次：s1/s2 就是 DBUS 的 S1/S2（1/2/3） */
void Shooter_Update(uint8_t s1, uint8_t s2);

/* 失能：全部电流 0，并清掉历史 */
void Shooter_SafeOutput(void);

#endif /* __DRIVER_SHOOTER_H */
