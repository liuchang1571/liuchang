/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
#include "bsp_can.h"
#include "bsp_rc.h"
#include "drv_motor.h"
#include "drv_steer.h"
#include "task_rc.h"
#include "task_chassis.h"
#include "task_gimbal.h"    /* ★ 云台 + 发射任务 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* ★ 云台任务：1kHz，优先级比底盘高（Realtime 48 > High 40）
 *
 *   为什么单独一个任务：
 *     · 底盘以后要加陀螺仪（INS 任务 + 姿态解算），云台隔离出来不会被拖慢
 *     · CAN1 上 pitch/拨弹/摩擦L/摩擦R 共用 0x200 一帧，本任务"算完就发"，
 *       天然同一拍，不需要跨任务传电流
 *
 *   栈 384 字（1536 B）：实测最坏调用链约 400 B（含 CAN 中断嵌套），留足余量。
 *   heap 预算：现有 4 个任务 2560 B + 本任务 1536 B ≈ 4.1 KB，远小于 15 KB。 */
osThreadId_t Gimbal_TaskHandle;
const osThreadAttr_t Gimbal_Task_attributes = {
  .name = "Gimbal_Task",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Chassis_Task */
osThreadId_t Chassis_TaskHandle;
const osThreadAttr_t Chassis_Task_attributes = {
  .name = "Chassis_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CAN_Rx_Task */
osThreadId_t CAN_Rx_TaskHandle;
const osThreadAttr_t CAN_Rx_Task_attributes = {
  .name = "CAN_Rx_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Init_Task */
osThreadId_t Init_TaskHandle;
const osThreadAttr_t Init_Task_attributes = {
  .name = "Init_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartGimbalTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Chassis_Task */
  Chassis_TaskHandle = osThreadNew(StartTask02, NULL, &Chassis_Task_attributes);

  /* creation of CAN_Rx_Task */
  CAN_Rx_TaskHandle = osThreadNew(StartTask03, NULL, &CAN_Rx_Task_attributes);

  /* creation of Init_Task */
  Init_TaskHandle = osThreadNew(StartTask04, NULL, &Init_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* ★ 云台 + 发射任务（Init_Task 之后创建，但它优先级和 Init 一样，
       实际由 Init 先跑完删掉自己，再轮到它） */
  Gimbal_TaskHandle = osThreadNew(StartGimbalTask, NULL, &Gimbal_Task_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* 10ms 杂活任务：电机掉线检测；以后可以在这里加喂狗 / 心跳灯 / 调试输出 */
  for(;;)
  {
    uint32_t now = HAL_GetTick();
    Motor_CheckOnline(now);
    Steer_CheckOnline(now);
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Chassis_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
__weak void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* 等 Init_Task 把外设都"点着"了再开始跑 */
  while (g_sys_ready == 0U) {
    osDelay(1);
  }
  Task_Chassis_Loop();          /* 1kHz 死循环，不会返回 */
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the CAN_Rx_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
__weak void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* CAN 接收已经在中断里做完了（见 bsp_can.c 的 HAL_CAN_RxFifo0MsgPendingCallback），
     这个任务没有存在意义。建议在 CubeMX 里直接删掉；
     这里先让自己挂起，不占 CPU、不占时间片。 */
  vTaskSuspend(NULL);
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the Init_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
__weak void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* ============ 这是全工程的"点火"步骤，顺序不能乱 ============ */
  BSP_CAN_Init();        /* 1. CAN1 + CAN2：过滤器 + Start + 接收中断 */
  Task_RC_Init();        /* 2. USART3：DMA 接收 + 空闲中断 */
  Task_Chassis_Init();   /* 3. 后轮速度环 / 前轮转向位置环 的 PID 参数 */
  Task_Gimbal_Init();    /* 4. 云台 yaw/pitch 的四个环 + 发射机构 */

  g_sys_ready = 1U;      /* 5. 放行其他任务 */
  vTaskDelete(NULL);     /* 6. 初始化做完，自己退出 */
  /* USER CODE END StartTask04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
  * @brief  云台 + 发射任务
  * @note   优先级 osPriorityRealtime(48) > 底盘 High(40)，
  *         所以每个 1ms 里它先跑：先把 CAN1 那两帧发出去，底盘随后。
  */
void StartGimbalTask(void *argument)
{
  /* 等 Init_Task 把外设都"点着"了再开始跑 */
  while (g_sys_ready == 0U) {
    osDelay(1);
  }
  Task_Gimbal_Loop();           /* 1kHz 死循环，不会返回 */
}
/* USER CODE END Application */
