/**
  ******************************************************************************
  * @file    bsp_can.c
  * @brief   两条 CAN 的收发实现
  *
  *  阅读顺序：
  *     1. CAN_Send4()                       内部工具：发一帧 4 路数据
  *     2. BSP_CAN_Send_Drive_Current()      CAN2 后轮电流
  *     3. BSP_CAN_Send_Steer_Voltage()      CAN2 转向电压
  *     4. BSP_CAN_Init()                    两条总线：过滤器 + Start + 中断
  *     5. HAL_CAN_RxFifo0MsgPendingCallback()  中断回调：按"哪条总线"分发
  ******************************************************************************
  */
#include "bsp_can.h"
#include "can.h"           /* hcan1 / hcan2 */
#include "config.h"        /* 底盘：电机 ID、帧 ID、各种上限 */
#include "config_gimbal.h" /* 云台：yaw/pitch 的 ID */
#include "drv_motor.h"     /* Motor_UpdateFeedback() */
#include "drv_steer.h"     /* Steer_UpdateFeedback() */
#include "driver_gimbal.h" /* g_gimbal */
#include "driver_shooter.h"/* g_shooter */
#include "bsp_motor_dji3508.h"
#include "bsp_motor_gm6020.h"
#include "config_shooter.h"

/* 编译期检查：电调 ID 必须在 1~4（下面用 "ID 减 1" 当数组下标） */
#if (DRIVE_ID_L < 1) || (DRIVE_ID_L > 4) || (DRIVE_ID_R < 1) || (DRIVE_ID_R > 4)
#error "config.h 里 DRIVE_ID_L / DRIVE_ID_R 必须在 1~4 之间"
#endif
#if (STEER_ID < 1) || (STEER_ID > 4)
#error "config.h 里 STEER_ID 必须在 1~4 之间"
#endif
#if (PITCH_3508_ID < 1) || (PITCH_3508_ID > 4)
#error "config_gimbal.h 里 PITCH_3508_ID 必须在 1~4 之间"
#endif
#if (YAW_6020_ID < 1) || (YAW_6020_ID > 4)
#error "config_gimbal.h 里 YAW_6020_ID 必须在 1~4 之间"
#endif
#if (FEEDER_3508_ID < 1) || (FEEDER_3508_ID > 4) || \
    (FRIC_L_3508_ID < 1) || (FRIC_L_3508_ID > 4) || \
    (FRIC_R_3508_ID < 1) || (FRIC_R_3508_ID > 4)
#error "config_shooter.h 里 3508 的 ID 必须在 1~4 之间"
#endif
#if (FEEDER_3508_ID == PITCH_3508_ID) || (FRIC_L_3508_ID == PITCH_3508_ID) || \
    (FRIC_R_3508_ID == PITCH_3508_ID) || (FRIC_L_3508_ID == FEEDER_3508_ID) || \
    (FRIC_R_3508_ID == FEEDER_3508_ID) || (FRIC_R_3508_ID == FRIC_L_3508_ID)
#error "CAN1 上 pitch/拨弹/摩擦L/摩擦R 的 ID 不能重复"
#endif

/* ============================================================================
 *  1. CAN_Send4 —— 发一帧 4 路数据
 * ----------------------------------------------------------------------------
 *  大疆电调"一帧控 4 个电机"的格式：
 *      第 1 个电机的值 -> 字节 0~1   ...   第 4 个 -> 字节 6~7
 *  每个值是 int16，【高字节在前】。
 *
 *  ★ 如果写成"低字节在前"，电调读到的电流会完全错误
 *    （比如 -100 会被当成 -25600），车会失控。
 * ==========================================================================*/
void CAN_Send4(CAN_HandleTypeDef *hcan, uint16_t std_id, const int16_t val[4])
{
    CAN_TxHeaderTypeDef tx;
    uint32_t mailbox;
    uint8_t data[8];
    uint16_t u;
    uint8_t i;

    if (hcan == 0 || val == 0) return;

    /* ---- 第 1 步：4 个 int16 拆成 8 个字节，高字节在前 ---- */
    for (i = 0; i < 4; i++)
    {
        u = (uint16_t)val[i];           /* 先转无符号，避免负数右移 */

        data[i * 2]     = (uint8_t)(u >> 8);
        data[i * 2 + 1] = (uint8_t)(u & 0xFFU);
    }

    /* ---- 第 2 步：填报文头 ---- */
    tx.StdId              = std_id;
    tx.ExtId              = 0;
    tx.IDE                = CAN_ID_STD;
    tx.RTR                = CAN_RTR_DATA;
    tx.DLC                = 8;
    tx.TransmitGlobalTime = DISABLE;    /* ★ 结构体是局部变量，不赋值就是随机数 */

    /* ---- 第 3 步：没有空闲邮箱就放弃这一帧 ----
     * F407 每个 CAN 只有 3 个发送邮箱。总线断了的时候邮箱会一直满，
     * 死等会把 1kHz 循环卡死，宁可丢一帧（下一毫秒还会再发）。 */
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
    {
        return;
    }

    /* ---- 第 4 步：真正发送 ---- */
    HAL_CAN_AddTxMessage(hcan, &tx, data, &mailbox);
}

/* ============================================================================
 *  2. BSP_CAN_Send_Drive_Current —— 后轮电流（CAN2 的 0x200）
 * ==========================================================================*/
void BSP_CAN_Send_Drive_Current(int16_t left, int16_t right)
{
    int16_t c[4] = { 0, 0, 0, 0 };

    c[DRIVE_ID_L - 1] = left;
    c[DRIVE_ID_R - 1] = right;

    CAN_Send4(&hcan2, DRIVE_CAN_ID, c);
}

/* ============================================================================
 *  3. BSP_CAN_Send_Steer_Voltage —— 转向电压（CAN2 的 0x1FF）
 * ==========================================================================*/
void BSP_CAN_Send_Steer_Voltage(int16_t voltage)
{
    int16_t v[4] = { 0, 0, 0, 0 };

    v[STEER_ID - 1] = voltage;

    CAN_Send4(&hcan2, STEER_CAN_ID, v);
}

/* ============================================================================
 *  4. BSP_CAN_Init —— "点火"（CAN1 + CAN2）
 * ==========================================================================*/
void BSP_CAN_Init(void)
{
    CAN_FilterTypeDef f = {0};

    /* ---- 过滤器：只放行 0x200~0x20F（所有电机的反馈都在这一段） ----
     * 掩码 0x7F0 的含义：高 7 位必须等于 0x200 的高 7 位，低 4 位随便。
     *
     * 【F407 的坑】CAN1 和 CAN2 共用 28 组过滤器（0~27）：
     *      0 ~ 13 号归 CAN1，14 ~ 27 号归 CAN2
     * 而"CAN2 从第几组开始"写在 CAN1 的寄存器里，
     * 所以不管配哪条 CAN，都要带 SlaveStartFilterBank = 14。 */
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = (0x200 << 5);          /* 11 位 ID 从第 5 位开始摆 */
    f.FilterIdLow          = 0;
    f.FilterMaskIdHigh     = (0x7F0 << 5);
    f.FilterMaskIdLow      = 0x0006;                /* bit2=IDE bit1=RTR：只要标准数据帧 */
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;

    f.FilterBank = 0;                               /* CAN1 用 0 号 */
    if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK)
    {
        Error_Handler();
    }

    f.FilterBank = 14;                              /* CAN2 用 14 号 */
    if (HAL_CAN_ConfigFilter(&hcan2, &f) != HAL_OK)
    {
        Error_Handler();
    }

    /* ---- 启动：不调这个，CAN 一直停在初始化模式，收发全无 ---- */
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
        Error_Handler();
    }

    /* ---- 打开"接收 FIFO0 非空"中断 ---- */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ============================================================================
 *  5. HAL_CAN_RxFifo0MsgPendingCallback —— 中断回调
 * ----------------------------------------------------------------------------
 *  ★ 两条总线的反馈 ID 是【重号】的：
 *       CAN2 的 0x201/0x205 = 后轮 / 转向
 *       CAN1 的 0x201/0x205 = pitch / yaw
 *    所以必须按 hcan->Instance 分支，不能只看 ID。
 * ==========================================================================*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx;
    uint8_t data[8];
    uint16_t id;
    uint16_t angle;
    int16_t  rpm;
    int16_t  torque;
    uint8_t  temp;

    /* 一次中断里可能堆了好几帧，读到 FIFO 空为止 */
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, data) != HAL_OK)
        {
            return;
        }

        /* 只用标准帧、8 字节数据帧 */
        if (rx.IDE != CAN_ID_STD || rx.DLC != 8)
        {
            continue;
        }

        id = rx.StdId;

        /* 按协议拼数值，同样是高字节在前 */
        angle  = (uint16_t)((data[0] << 8) | data[1]);
        rpm    = (int16_t)(((uint16_t)data[2] << 8) | (uint16_t)data[3]);
        torque = (int16_t)(((uint16_t)data[4] << 8) | (uint16_t)data[5]);
        temp   = data[6];

        if (hcan->Instance == CAN2)
        {
            /* ---------------- CAN2：底盘 ---------------- */
            if (id >= 0x201U && id <= 0x204U)
            {
                /* 0x201~0x204 是后轮 3508 的反馈（ID1 -> 0x201） */
                Motor_UpdateFeedback((uint8_t)(id - 0x201), rpm, torque, temp);
            }
            else if (id == (uint16_t)(0x204U + STEER_ID))
            {
                /* 前轮转向 6020（ID1 -> 0x205） */
                Steer_UpdateFeedback(angle, rpm, torque, temp);
            }
        }
        else
        {
            /* ---------------- CAN1：云台 + 发射 ---------------- */
            if (id == (uint16_t)(0x200U + PITCH_3508_ID))
            {
                /* pitch 3508（ID1 -> 0x201），需要角度累加 */
                MOTOR3508_Feedback(&g_gimbal.pitch_motor, data);
            }
            else if (id == (uint16_t)(0x204U + YAW_6020_ID))
            {
                /* yaw 6020（ID1 -> 0x205），单圈绝对角 */
                GM6020_Feedback(&g_gimbal.yaw_motor, data);
            }
            else if (id == (uint16_t)(0x200U + FEEDER_3508_ID))
            {
                /* 拨弹盘（ID2 -> 0x202） */
                MOTOR3508_Feedback(&g_shooter.feeder, data);
            }
            else if (id == (uint16_t)(0x200U + FRIC_L_3508_ID))
            {
                /* 摩擦轮左（ID3 -> 0x203） */
                MOTOR3508_Feedback(&g_shooter.fric_l, data);
            }
            else if (id == (uint16_t)(0x200U + FRIC_R_3508_ID))
            {
                /* 摩擦轮右（ID4 -> 0x204） */
                MOTOR3508_Feedback(&g_shooter.fric_r, data);
            }
        }
    }
}
