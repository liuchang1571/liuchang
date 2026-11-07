/**
  ******************************************************************************
  * @file    task_gimbal.h
  * @brief   云台 + 发射任务（1kHz，独立于底盘任务）
  ******************************************************************************
  */
#ifndef __TASK_GIMBAL_H
#define __TASK_GIMBAL_H

#include "main.h"

/**
  * @brief 初始化云台 + 发射（对象、PID 参数）。在 Init_Task 里调一次
  */
void Task_Gimbal_Init(void);

/**
  * @brief 1kHz 主循环（不返回）。由 freertos.c 的 Gimbal_Task 调用
  */
void Task_Gimbal_Loop(void);

#endif
