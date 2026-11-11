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

/* ============================================================================
 *  CAN 收帧监视（★ 只给调试用，不参与任何控制）
 * ============================================================================
 *  用途：想知道电机 ID 到底设成了几号？看下面这几个变量就够了 ——
 *        6020 反馈帧 = 0x204 + ID   -> ID1 回 0x205、ID2 回 0x206
 *        3508 反馈帧 = 0x200 + ID   -> ID1 回 0x201、ID2 回 0x202
 *
 *  用法：上电后看 g_can1_rx_id_last（最近一帧的 ID），
 *        再看 g_can1_rx_cnt[] 里哪个格子一直在涨 ——
 *        那个格子对应的 ID 就是有电机在回话的 ID。
 *        下标 = 帧 ID - 0x200。
 * ==========================================================================*/
extern volatile uint16_t g_can1_rx_id_last;   /* CAN1 最近一帧的标准帧 ID */
extern volatile uint16_t g_can2_rx_id_last;   /* CAN2 最近一帧的标准帧 ID */
extern volatile uint16_t g_can1_rx_cnt[16];   /* CAN1 各 ID 收帧计数，下标 = ID-0x200 */
extern volatile uint16_t g_can2_rx_cnt[16];   /* CAN2 各 ID 收帧计数，下标 = ID-0x200 */

/* ----------------------------------------------------------------------------
 *  CAN1 底层诊断（收不到数据时先看这组）
 * ----------------------------------------------------------------------------
 *  g_can1_state    HAL 状态：2 = LISTENING 正常；不是 2 = CAN 没启动
 *  g_can1_tx_cnt   发出帧数。正常约 2000/秒（2 帧/ms）；一直是 0 说明发送被掐住了
 *  g_can1_tec      发送错误计数：一直涨 = 发出去没人应答（接线/终端电阻/波特率）
 *  g_can1_rec      接收错误计数：>0 = 帧在进来但解错（波特率/干扰）
 *  g_can1_lec      最近错误码：3 = ACK 错误（总线上没人在应答）
 *  g_can1_tx_fail  因邮箱满丢弃的帧数
 *
 *  判断口诀：
 *    state != 2                       -> CAN 没启动（查 Error_Handler）
 *    tec 涨 + lec = 3                 -> 发得出去但没人应答
 *    tx_cnt / tec 都是 0，rx_cnt 全 0  -> 只收不发模式下完全没收到帧（硬件/接线）
 *    rx_cnt 有值时看下标：下标 = ID-0x200，例如下标 5 = 收到 0x205 = 6020 的 ID1
 * -------------------------------------------------------------------------- */
extern volatile uint32_t g_can1_state;
extern volatile uint8_t  g_can1_tec;
extern volatile uint8_t  g_can1_rec;
extern volatile uint8_t  g_can1_lec;
extern volatile uint32_t g_can1_tx_cnt;
extern volatile uint32_t g_can1_tx_fail;

/* 收帧汇总：收到任何一帧就 +1。
 * ★ 排查时最先看这两个数：
 *    g_can1_rx_total 在涨 -> CAN1 上确实有信号，接着看 g_can1_rx_id_last 是谁
 *    g_can1_rx_total 恒 0 -> CAN1 这根线上什么都没来（硬件问题）
 *    g_can2_rx_total 在涨 -> 电机其实插在 CAN2 口上 */
extern volatile uint32_t g_can1_rx_total;
extern volatile uint32_t g_can2_rx_total;

/* ----------------------------------------------------------------------------
 *  CAN1 配置寄存器快照（证明过滤器/中断真的配上了）
 * ----------------------------------------------------------------------------
 *  初始化成功后的期望值：
 *    g_can1_msr   低 2 位 = 0        INAK=0 退出初始化、SLAK=0 正常模式
 *    g_can1_ier   bit1 = 1           FIFO0 非空中断已使能
 *    g_can1_fa1r  0x00004001         过滤器 0（CAN1）和 14（CAN2）都激活了
 *    g_can1_fr1   0x40000000         ID = 0x200
 *    g_can1_fr2   0xFE000006         掩码 0x7F0 + IDE/RTR
 *    g_can1_fmr   FINIT=0, CAN2SB=14
 *    g_can1_rf0r  0                  没有堆积未取的消息
 *  任一条对不上 = 代码配置问题，不是接线问题。
 * -------------------------------------------------------------------------- */
extern volatile uint32_t g_can1_msr;
extern volatile uint32_t g_can1_ier;
extern volatile uint32_t g_can1_fmr;
extern volatile uint32_t g_can1_fa1r;
extern volatile uint32_t g_can1_fr1;
extern volatile uint32_t g_can1_fr2;
extern volatile uint32_t g_can1_rf0r;

#endif
