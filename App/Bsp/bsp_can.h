/**
  ******************************************************************************
  * @file    bsp_can.h
  * @brief   两条 CAN 的收发（BSP 层）
  *
  * ============================================================================
  *  CAN2（底盘）：0x200 后轮电流 / 0x1FF 转向电压
  *                   收 0x201 0x202 后轮反馈、0x205 转向反馈
  *  CAN1（云台）：0x200 pitch 电流（槽0，槽1~3 留给发射）
  *                0x1FF yaw 电压（槽0）
  *                   收 0x201 pitch 反馈、0x205 yaw 反馈
  *
  *  ★ 两条总线上的 ID 是各自独立的：yaw 和转向都可以是 ID1，
  *    都在各自的 0x1FF 槽0、反馈都在各自的 0x205 —— 靠"哪条总线"区分。
  * ============================================================================
  */
#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "main.h"
#include "can.h"        /* hcan1 / hcan2 的 extern 声明（CAN_Send4 要用） */

/**
  * @brief  CAN1 + CAN2 初始化（配过滤器 -> HAL_CAN_Start -> 开接收中断）
  * @note   调用位置：Core/Src/freertos.c 的 Init_Task（StartTask04），开机一次
  *
  * @note   ★ MX_CANx_Init() 之后 CAN 还停在初始化模式，既不收也不发；
  *           而且这种情况下 HAL_CAN_AddTxMessage() 还会返回"成功"。
  *           必须 HAL_CAN_Start()，并且配了过滤器才收得到反馈。
  */
void BSP_CAN_Init(void);

/**
  * @brief  给后轮两个 3508 发电流（CAN2 的 0x200）
  * @param  left/right  -16384 ~ +16384
  * @note   ★ 电调没有超时保护：必须每 1ms 发一次，哪怕电流是 0。
  */
void BSP_CAN_Send_Drive_Current(int16_t left, int16_t right);

/**
  * @brief  给前轮转向的 6020 发电压（CAN2 的 0x1FF，槽 = STEER_ID-1）
  * @param  voltage  -30000 ~ +30000
  */
void BSP_CAN_Send_Steer_Voltage(int16_t voltage);

/**
  * @brief  通用发送：大疆"一帧控 4 个"的格式（4 个 int16，高字节在前）
  * @param  hcan    哪条总线
  * @param  std_id  帧 ID（0x200 / 0x1FF / 0x2FF ...）
  * @param  val     4 个值，val[0] 对应槽0
  *
  * @note   云台的 pitch（CAN1 0x200）和 yaw（CAN1 0x1FF）都用它发。
  *         发送前会检查空闲邮箱，没有就丢掉这一帧（不会卡住 1ms 循环）。
  */
void CAN_Send4(CAN_HandleTypeDef *hcan, uint16_t std_id, const int16_t val[4]);

#endif
