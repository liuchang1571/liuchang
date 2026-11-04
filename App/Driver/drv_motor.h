/**
  ******************************************************************************
  * @file    drv_motor.h
  * @brief   后轮驱动电机（M3508 + C620）的"速度环"
  *
  * ============================================================================
  *  一、什么是"速度环"？
  * ============================================================================
  *  你告诉它"我想要多快"，它自己想办法让电机转到那个速度。
  *  办法就是不断调整电流：
  *      实际比目标慢  -> 加大电流（往前推）
  *      实际比目标快  -> 减小电流（拉住它）
  *  这个"不断比较、不断调整"的过程叫闭环，"调整的算法"叫 PID。
  *
  * ============================================================================
  *  二、单位约定（★ 看错单位会让车乱跑 ★）
  * ============================================================================
  *  全工程的速度单位统一用【转子 rpm】，
  *  也就是电调反馈里直接给的那个数字。
  *
  *  为什么不换算成"轮子每秒转几圈"？
  *  因为反馈里给的就是转子 rpm，直接用，不用除也不用乘，
  *  永远不会搞混。
  *
  *      遥控器 -> 米/秒 -> 运动解算 -> 转子 rpm -> PID -> 电流
  *                                        ^
  *                                        |
  *                              电调反馈（本来就是这个单位）
  ******************************************************************************
  */
#ifndef __DRV_MOTOR_H
#define __DRV_MOTOR_H

#include "main.h"

/* 0x200 这一帧最多装 4 个电机，所以数组开 4 个 */
#define DRIVE_MOTOR_NUM  4

/**
  * @brief 一个驱动电机的全部状态
  * @note  这个结构体就是电机在程序里的"代表"。
  *        想控制轮子，就是对这块内存里的数字做加减乘除。
  */
typedef struct {

    /* ---------- PID 参数（在 config.h 里调）---------- */
    float kp;              /* 比例：误差多大，就出多大力 */
    float ki;              /* 积分：误差一直存在时，慢慢加力把差距补掉 */
    float kd;              /* 微分：速度环一般不用 */

    /* ---------- 输入（由 task_chassis.c 写）---------- */
    float target_rpm;      /* 目标转速，单位【转子 rpm】 */

    /* ---------- 反馈（由 CAN 中断写）---------- */
    float actual_rpm;      /* 实测转速，单位【转子 rpm】 */
    int16_t torque_cur;    /* 实测转矩电流，调试用 */
    uint8_t temperature;   /* 温度，调试用 */
    uint32_t last_rx_tick; /* 最后一次收到反馈的时刻，用来判断掉线 */
    uint8_t online;        /* 1 = 在线，0 = 掉线 */

    /* ---------- PID 内部用的中间量 ---------- */
    float integral;        /* 积分累计值 */
    float last_error;      /* 上一次的误差 */

    /* ---------- 输出（PID 算完写这里，task 拿去发）---------- */
    int16_t give_current;  /* 要发给电调的电流，-16384 ~ +16384 */

} Motor_t;

/* ============================================================================
 *  下面每个函数都注明了"在哪被调用"
 * ==========================================================================*/

/**
  * @brief  初始化一个电机对象
  * @note   调用位置：App/Task/task_chassis.c 的 Task_Chassis_Init()，
  *                   而它又被 freertos.c 的 Init_Task 调用。【开机一次】
  * @note   实现位置：App/Driver/drv_motor.c
  * @param  idx  电机编号 0~3（0 对应 CAN 上的 ID1，也就是 0x201）
  * @param  kp   比例系数
  * @param  ki   积分系数
  * @param  kd   微分系数
  * @retval  无
  */
void Motor_Init(uint8_t idx, float kp, float ki, float kd);

/**
  * @brief  按编号取出电机对象
  * @note   调用位置：task_chassis.c 里到处都在用
  * @note   实现位置：App/Driver/drv_motor.c
  * @param  idx  电机编号 0~3
  * @retval  电机指针；编号不对时返回 0（NULL）
  */
Motor_t *Motor_Get(uint8_t idx);

/**
  * @brief  更新电机的反馈数据（只能在 CAN 中断里调用）
  * @note   调用位置：App/Bsp/bsp_can.c 的
  *                   HAL_CAN_RxFifo0MsgPendingCallback()，
  *                   每收到一帧 0x201~0x204 就调一次（每秒约 2000 次）
  * @note   实现位置：App/Driver/drv_motor.c
  * @param  idx        电机编号 0~3
  * @param  rotor_rpm  电调反馈的转子转速（原样传进来，不要换算）
  * @param  torque     电调反馈的转矩电流
  * @param  temp       电调反馈的温度
  * @retval  无
  *
  * @note   ★ 这个函数在中断里执行，里面只能做简单赋值，
  *           不要加除法、不要 printf、不要发 CAN。
  */
void Motor_UpdateFeedback(uint8_t idx, int16_t rotor_rpm,
                          int16_t torque, uint8_t temp);

/**
  * @brief  算一次速度环 PID
  * @note   调用位置：App/Task/task_chassis.c 的 Task_Chassis_Loop()，
  *                   每 1 毫秒调用一次，dt 传 0.001f
  * @note   实现位置：App/Driver/drv_motor.c
  * @param  m   电机对象
  * @param  dt  距离上次调用过了多少【秒】
  * @retval  无（结果写在 m->give_current 里）
  */
void Motor_Speed_PID_Calc(Motor_t *m, float dt);

/**
  * @brief  检查电机是否掉线（超过 50ms 没反馈就算掉线）
  * @note   调用位置：Core/Src/freertos.c 的 StartDefaultTask()，每 10ms 一次
  * @note   实现位置：App/Driver/drv_motor.c
  * @param  now_tick  当前时刻，一般传 HAL_GetTick()
  * @retval  无（结果写在每个电机的 online 字段里）
  */
void Motor_CheckOnline(uint32_t now_tick);

#endif
