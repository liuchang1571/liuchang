/**
  ******************************************************************************
  * @file    bsp_motor_gm6020.c
  * @brief   GM6020 反馈解析 + 电压限幅（yaw，CAN1）
  ******************************************************************************
  */
#include "bsp_motor_gm6020.h"

void GM6020_Init(GM6020_t *m, uint8_t id)
{
    if (m == 0) return;

    m->id           = id;
    m->angle_raw    = 0;
    m->angle_deg    = 0.0f;
    m->vel_rpm      = 0;
    m->vel_dps      = 0.0f;
    m->cur          = 0;
    m->temp         = 0;
    m->volt_cmd     = 0;
    m->updated      = 0;
    m->last_rx_tick = 0;
}

/* 只做限幅，写进 volt_cmd。不发帧（0x1FF 由任务统一发） */
void GM6020_SetVoltage(GM6020_t *m, float voltage)
{
    if (m == 0) return;

    if (voltage >  GM6020_VOLT_MAX) voltage =  GM6020_VOLT_MAX;
    if (voltage < -GM6020_VOLT_MAX) voltage = -GM6020_VOLT_MAX;

    m->volt_cmd = (int16_t)voltage;
}

/* 反馈：8 字节，所有 2 字节数据高字节在前 */
void GM6020_Feedback(GM6020_t *m, uint8_t *data)
{
    if (m == 0 || data == 0) return;

    /* 字节 0~1：机械角度 0~8191（单圈绝对） */
    m->angle_raw = (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
    m->angle_raw &= 0x1FFFU;
    m->angle_deg = (float)m->angle_raw * (360.0f / 8192.0f);

    /* 字节 2~3：转速 rpm（有符号） */
    m->vel_rpm = (int16_t)(((uint16_t)data[2] << 8) | (uint16_t)data[3]);
    m->vel_dps = (float)m->vel_rpm * 6.0f;      /* rpm -> deg/s */

    /* 字节 4~5：转矩电流；字节 6：温度 */
    m->cur  = (int16_t)(((uint16_t)data[4] << 8) | (uint16_t)data[5]);
    m->temp = data[6];

    m->updated      = 1;
    m->last_rx_tick = HAL_GetTick();
}
