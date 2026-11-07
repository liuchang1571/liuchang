/**
  ******************************************************************************
  * @file    bsp_motor_gm6020.h
  * @brief   GM6020 反馈解析 + 电压限幅（yaw，CAN1）
  *
  *   ★ 本文件【不发 CAN】：发送统一在 task_chassis.c 里用 CAN_Send4(&hcan1, 0x1FF, v)，
  *     这样一帧只在一个地方发，避免"两个地方各发一帧 → 互相把槽清零"。
  *
  *   协议：
  *     电压控制   ID 1~4 -> 0x1FF，ID 5~7 -> 0x2FF，4 个 int16，高字节在前
  *     反馈       ID = 0x204 + 电机ID（ID1 -> 0x205）
  *                字节0~1 角度0~8191 / 2~3 转速rpm / 4~5 电流 / 6 温度
  ******************************************************************************
  */
#ifndef __BSP_MOTOR_GM6020_H
#define __BSP_MOTOR_GM6020_H

#include "main.h"

#define GM6020_VOLT_MAX   30000.0f

typedef struct
{
    uint8_t  id;

    uint16_t angle_raw;         /* 0~8191，单圈【绝对】 */
    float    angle_deg;         /* 0~360 */
    int16_t  vel_rpm;           /* 转子 rpm */
    float    vel_dps;           /* deg/s */
    int16_t  cur;               /* 转矩电流（只显示） */
    uint8_t  temp;

    int16_t  volt_cmd;          /* 本周期要发的电压（已限幅），由任务取出 */

    uint8_t  updated;           /* 1 = 收到过反馈 */
    uint32_t last_rx_tick;
}GM6020_t;

void GM6020_Init(GM6020_t *m, uint8_t id);

/* 只做限幅并写进 m->volt_cmd，不发帧 */
void GM6020_SetVoltage(GM6020_t *m, float voltage);

/* 解析 8 字节反馈（CAN 中断里调） */
void GM6020_Feedback(GM6020_t *m, uint8_t *data);

#endif /* __BSP_MOTOR_GM6020_H */
