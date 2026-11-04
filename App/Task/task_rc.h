/**
  ******************************************************************************
  * @file    task_rc.h
  * @brief   遥控器任务（App 层）
  *
  * ============================================================================
  *  这一层夹在中间：
  *     BSP 层（bsp_rc.c）  只管搬字节
  *     App 层（本文件）    判断关控、摇杆回中
  *     底盘任务（task_chassis.c）只管控车
  *
  * ============================================================================
  *  它没有自己的 FreeRTOS 任务
  * ============================================================================
  *  Task_RC_Loop() 是被 Chassis_Task 的 1kHz 循环【调用】的，不是独立任务。
  *  为什么？因为底盘本来每 1ms 就要读一次摇杆，
  *  再单独开一个任务只会多一次任务切换、多一份栈、多一堆同步问题。
  ******************************************************************************
  */
#ifndef __TASK_RC_H
#define __TASK_RC_H

#include "main.h"

/**
  * @brief  初始化遥控器接收
  * @note   调用位置：Core/Src/freertos.c 的 Init_Task（StartTask04），
  *                   开机一次，在 BSP_CAN_Init() 之后
  * @note   实现位置：App/Task/task_rc.c（内部只是调 BSP_RC_Init()）
  * @param   无
  * @retval  无
  */
void Task_RC_Init(void);

/**
  * @brief  处理遥控器：取新帧 / 更新 rc_data / 判断关控并摇杆回中
  * @note   调用位置：App/Task/task_chassis.c 的 Task_Chassis_Loop() 里第一件事，
  *                   每 1 毫秒调用一次
  * @note   实现位置：App/Task/task_rc.c
  * @param   无
  * @retval  无
  */
void Task_RC_Loop(void);

#endif
