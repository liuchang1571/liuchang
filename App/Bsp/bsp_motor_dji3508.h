/**
  ******************************************************************************
  * @file    bsp_motor_dji3508.h
  * @brief   M3508 反馈解析（pitch，CAN1）
  *
  *   和底盘那两个 3508 的区别：
  *     底盘只需要 rpm（drv_motor.c 的 Motor_t）
  *     pitch 还需要【角度累加】angle_total（因为它没有绝对编码器，靠上电定零）
  *   所以这里单独一份，不去动已经调好的 drv_motor.c。
  *
  *   协议：
  *     电流控制   ID 1~4 -> 0x200，ID 5~8 -> 0x1FF
  *     反馈       ID = 0x200 + 电机ID（ID1 -> 0x201）
  *                字节0~1 角度0~8191 / 2~3 转子rpm / 4~5 转矩 / 6 温度
  ******************************************************************************
  */
#ifndef __BSP_MOTOR_DJI3508_H
#define __BSP_MOTOR_DJI3508_H

#include "main.h"

/* 转子 -> 输出轴 的减速比（和 drv_motor.c 保持一致） */
#define MOTOR3508_GEAR_RATIO   19.0f

typedef struct
{
    uint8_t id;

    int16_t angle_raw;          /* 本帧机械角度 0~8191 */
    int16_t speed_rpm;          /* 转子 rpm */
    int16_t torque;             /* 转矩电流 */

    int16_t last_angle_raw;     /* 上一帧角度（累加用） */
    int32_t angle_total;        /* 上电以来的累计计数 */

    float   angle_deg;          /* 输出轴角度（度） */
    float   speed_rps;          /* 输出轴转速（转/秒） */

    uint8_t  inited;            /* 第一帧只记基准，不累加 */
    uint8_t  updated;
    uint32_t last_rx_tick;
}DJI3508_t;

void MOTOR3508_Init(uint8_t id, DJI3508_t *m);
void MOTOR3508_Feedback(DJI3508_t *m, uint8_t *data);

#endif /* __BSP_MOTOR_DJI3508_H */
