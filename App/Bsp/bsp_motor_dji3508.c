/**
  ******************************************************************************
  * @file    bsp_motor_dji3508.c
  * @brief   M3508 反馈解析（pitch，CAN1）
  ******************************************************************************
  */
#include "bsp_motor_dji3508.h"

void MOTOR3508_Init(uint8_t id, DJI3508_t *m)
{
    if (m == 0) return;

    m->id             = id;
    m->angle_raw      = 0;
    m->speed_rpm      = 0;
    m->torque         = 0;
    m->last_angle_raw = 0;
    m->angle_total    = 0;
    m->angle_deg      = 0.0f;
    m->speed_rps      = 0.0f;
    m->inited         = 0;
    m->updated        = 0;
    m->last_rx_tick   = 0;
}

void MOTOR3508_Feedback(DJI3508_t *m, uint8_t *data)
{
    int32_t diff;

    if (m == 0 || data == 0) return;

    m->angle_raw = (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
    m->speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | (uint16_t)data[3]);
    m->torque    = (int16_t)(((uint16_t)data[4] << 8) | (uint16_t)data[5]);

    if (!m->inited)
    {
        /* ★ 第一帧只记基准，不累加。
           原来的写法拿 0 当"上一帧"，上电第一帧会凭空跳 -4096~+4096 个计数
           （= 输出轴 ±9°），pitch 的零点直接被带偏。 */
        m->inited         = 1;
        m->last_angle_raw = m->angle_raw;
        m->angle_total    = 0;
    }
    else
    {
        diff = (int32_t)m->angle_raw - (int32_t)m->last_angle_raw;
        if (diff >  4096) diff -= 8192;         /* 过 0 点回绕 */
        if (diff < -4096) diff += 8192;

        m->angle_total   += diff;
        m->last_angle_raw = m->angle_raw;
    }

    /* 输出轴角度：8192 计数 = 转子 1 圈 = 输出轴 1/19 圈 */
    m->angle_deg = (float)m->angle_total * (360.0f / 8192.0f) / MOTOR3508_GEAR_RATIO;
    m->speed_rps = (float)m->speed_rpm * 0.10472f;

    m->updated      = 1;
    m->last_rx_tick = HAL_GetTick();
}
