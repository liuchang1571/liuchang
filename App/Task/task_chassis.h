/**
  ******************************************************************************
  * @file    task_chassis.h
  * @brief   底盘任务（App 层）—— 整个车的大脑，1kHz 循环
  ******************************************************************************
  */
#ifndef __TASK_CHASSIS_H
#define __TASK_CHASSIS_H

#include "main.h"

/**
  * @brief  初始化底盘：后轮速度环 + 前轮位置环的 PID 参数
  * @note   调用位置：Core/Src/freertos.c 的 Init_Task（StartTask04），
  *                   开机一次，在 BSP_CAN_Init() 和 Task_RC_Init() 之后。
  * @note   实现位置：App/Task/task_chassis.c
  * @param   无
  * @retval  无
  */
void Task_Chassis_Init(void);

/**
  * @brief  底盘主循环（内部是死循环，靠 vTaskDelayUntil 每 1ms 走一圈）
  * @note   调用位置：Core/Src/freertos.c 的 Chassis_Task（StartTask02）里，
  *                   等 g_sys_ready 变成 1 之后调用，之后永远不会返回。
  * @note   实现位置：App/Task/task_chassis.c
  * @param   无
  * @retval  无（不会返回）
  */
void Task_Chassis_Loop(void);

#endif
