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

/* 转向 6020 接在 CAN1 上、云台 yaw 6020 也在 CAN1 上时，两者 ID 不能相同
 * （都是 6020，反馈帧都是 "0x204 + ID"，同号就分不清是谁）。
 *
 * ★ 现在云台走 CAN2、转向走 CAN1，两条总线的 ID 各自独立，
 *   所以两边都用 ID1 也不会冲突 —— 这正是本文件原来那句
 *   "两条总线上的 ID 是各自独立的" 的意思。
 * ★ 但以后要是把云台也挪回 CAN1，就必须让 STEER_ID != YAW_6020_ID，
 *   并且把接收回调里 steer 和 yaw 两个分支的先后顺序理清楚。 */
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
 *  CAN 收帧监视（★ 只给调试用，不参与控制）
 * ----------------------------------------------------------------------------
 *  为什么要它？电机不会主动告诉你自己是几号。唯一的办法是看它回的帧 ID：
 *      6020 反馈帧 = 0x204 + ID   -> ID1 回 0x205、ID2 回 0x206
 *      3508 反馈帧 = 0x200 + ID   -> ID1 回 0x201、ID2 回 0x202
 *  所以上电后看哪个 ID 的计数在涨，电机就是几号。
 *
 *  ★ 只有 CAN 中断会写它们（单写者），所以不用担心竞争。
 * ==========================================================================*/
volatile uint16_t g_can1_rx_id_last = 0;   /* CAN1 最近一帧的标准帧 ID */
volatile uint16_t g_can2_rx_id_last = 0;   /* CAN2 最近一帧的标准帧 ID */
volatile uint16_t g_can1_rx_cnt[16];       /* CAN1 各 ID 的收帧计数，下标 = ID - 0x200 */
volatile uint16_t g_can2_rx_cnt[16];       /* CAN2 各 ID 的收帧计数，下标 = ID - 0x200 */

/* ============================================================================
 *  CAN1 底层诊断（★ 只给调试用）
 * ----------------------------------------------------------------------------
 *  收不到数据时，靠这几个值就能判断断在哪一环：
 *
 *    g_can1_state  HAL 状态。2 = LISTENING = 正常。
 *                  0 = RESET（没初始化）、1 = READY（还没 Start）、
 *                  3 = BUS_OFF、4 = SLEEP、5 = ERROR
 *                  → 只要不是 2，问题就在"CAN 没启动"，和接线无关。
 *
 *    g_can1_tx_cnt 我们往 CAN1 发了多少帧（云台任务每 1ms 发 2 帧，应该飞快地涨）
 *                  → 一直是 0 说明云台任务根本没跑起来
 *
 *    g_can1_tec    发送错误计数（0~255）
 *                  → 我们发出去的帧没人应答（总线上没有其它节点 /
 *                    接反 / 没终端电阻 / 波特率不对）时，它会一路涨到 128 以上
 *
 *    g_can1_rec    接收错误计数
 *                  → 大于 0 说明"有帧在进来，但是错的"，典型是波特率不匹配或干扰
 *
 *    g_can1_lec    最近一次错误码
 *                  0=无错 1=位填充 2=格式 3=应答(ACK) 4=位显性 6=位隐性
 *                  → 3 = ACK 错误，意思"我发了一帧，总线上没人在应答"
 *
 *  ★ 判断口诀：
 *      state 不是 2            -> CAN 没启动（查初始化 / Error_Handler）
 *      tx_cnt 不涨             -> 云台任务没跑
 *      tec 一直涨 + lec = 3    -> 发得出去但没人应答（硬件/接线/终端电阻）
 *      tec = 0 且 rec = 0      -> 总线完全安静，电机没在发（或接的不是这对脚）
 *      rec 涨                  -> 帧在进来但解码错（波特率/干扰）
 * ==========================================================================*/
volatile uint32_t g_can1_state   = 0;   /* hcan1.State */
volatile uint8_t  g_can1_tec     = 0;   /* 发送错误计数 */
volatile uint8_t  g_can1_rec     = 0;   /* 接收错误计数 */
volatile uint8_t  g_can1_lec     = 0;   /* 最近一次错误码 */
volatile uint32_t g_can1_tx_cnt  = 0;   /* 累计发出多少帧 */
volatile uint32_t g_can1_tx_fail = 0;   /* 因为没空闲邮箱丢掉多少帧 */

/* ============================================================================
 *  CAN1 配置寄存器快照（★ 只给调试用）
 * ----------------------------------------------------------------------------
 *  这组值的用途：证明"过滤器/中断到底有没有真的配上"。
 *  光看 HAL 返回 HAL_OK 不够，直接看寄存器才是硬证据。
 *
 *  上电初始化成功后，期望值：
 *    g_can1_msr   低 2 位应为 0        INAK=0 已退出初始化；SLAK=0 正常模式
 *    g_can1_ier   bit1 (0x02) 为 1     FMPIE0 = FIFO0 非空中断已使能
 *    g_can1_fa1r  0x00004001           bit0=1 (CAN1 过滤器0) bit14=1 (CAN2 过滤器)
 *    g_can1_fr1   0x40000000           → ID 寄存器里 STID = 0x200
 *    g_can1_fr2   0xFE000006           → 掩码 0x7F0 + IDE/RTR 必配 0
 *    g_can1_fmr   0x0E00 附近          CAN2SB=14；FINIT=0（不在过滤器初始化态）
 *    g_can1_rf0r  0                    没有待取的消息（就算有数据也会立刻被取走）
 *
 *  ★ 只要上面有一条对不上，就是代码配置问题，不是接线问题。
 * ==========================================================================*/
volatile uint32_t g_can1_msr  = 0;      /* 主状态：INAK/SLAK */
volatile uint32_t g_can1_ier  = 0;      /* 中断使能 */
volatile uint32_t g_can1_fmr  = 0;      /* 过滤器主寄存器（CAN2SB / FINIT） */
volatile uint32_t g_can1_fa1r = 0;      /* 过滤器激活位 */
volatile uint32_t g_can1_fr1  = 0;      /* 0 号过滤器的 FR1（ID） */
volatile uint32_t g_can1_fr2  = 0;      /* 0 号过滤器的 FR2（掩码） */
volatile uint32_t g_can1_rf0r = 0;      /* 接收 FIFO0 状态 */

/* 收帧汇总：只要收到任何一帧就 +1。
 * 这是回答"总线上到底有没有信号"最省事的一个数。 */
volatile uint32_t g_can1_rx_total = 0;
volatile uint32_t g_can2_rx_total = 0;

/* 采一次 CAN1 的底层状态（任务上下文里调用，开销极小） */
static void can1_diag_snapshot(void)
{
    uint32_t esr;

    esr = hcan1.Instance->ESR;

    g_can1_state = (uint32_t)hcan1.State;
    g_can1_tec   = (uint8_t)((esr >> 16) & 0xFFU);   /* bits 23:16 = TEC */
    g_can1_rec   = (uint8_t)((esr >> 8)  & 0xFFU);   /* bits 15:8  = REC */
    g_can1_lec   = (uint8_t)((esr >> 4)  & 0x07U);   /* bits 6:4   = LEC */

    /* 配置寄存器：用来验证过滤器/中断是否真的写进去了。
     * ★ 过滤器寄存器挂在 CAN1 这个外设上（CAN2 共享同一组），所以用 CAN1。 */
    g_can1_msr  = hcan1.Instance->MSR;
    g_can1_ier  = hcan1.Instance->IER;
    g_can1_rf0r = hcan1.Instance->RF0R;
    g_can1_fmr  = CAN1->FMR;
    g_can1_fa1r = CAN1->FA1R;
    g_can1_fr1  = CAN1->sFilterRegister[0].FR1;
    g_can1_fr2  = CAN1->sFilterRegister[0].FR2;
}

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

    /* ★ 顺手采一次 CAN1 的底层状态。
     *   这个函数每 1ms 都被调用（云台任务发 CAN1、底盘任务发 CAN2），
     *   所以在调试器里看到的诊断值永远是最新的。 */
    if (hcan == &hcan1)
    {
        can1_diag_snapshot();

#if (CAN1_TX_DISABLE == 1) && (CAN1_LOOPBACK_TEST == 0)
        /* ★ 调试模式：CAN1 只收不发。
         *   目的是让接收不受"发送没人应答 -> bus-off -> 连收也停"的影响。
         *   详见 config.h 里 CAN1_TX_DISABLE 的说明。
         *   回环自测时这个屏蔽必须让开（回环要靠自己发帧）。 */
        return;
#endif
    }

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
        if (hcan == &hcan1)
        {
            g_can1_tx_fail = g_can1_tx_fail + 1U;
        }
        return;
    }

    /* ---- 第 4 步：真正发送 ---- */
    HAL_CAN_AddTxMessage(hcan, &tx, data, &mailbox);

    if (hcan == &hcan1)
    {
        g_can1_tx_cnt = g_can1_tx_cnt + 1U;
    }
}

/* ============================================================================
 *  2. BSP_CAN_Send_Drive_Current —— 后轮电流（CAN2 的 0x200）
 * ==========================================================================*/
void BSP_CAN_Send_Drive_Current(int16_t left, int16_t right)
{
    int16_t c[4] = { 0, 0, 0, 0 };

    /* 顺手采一次 CAN1 诊断。这个函数由底盘任务每 1ms 调用，
     * 所以就算云台任务没跑，诊断值也是最新的（双保险）。 */
    can1_diag_snapshot();

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

#if (STEER_ON_CAN1 == 1)
    /* 转向电机接在 CAN1 上，所以控制帧也发 CAN1。
     *
     * ★ 这里能这么写的前提：CAN1 的 0x1FF【只有这一个发送者】。
     *   云台任务那一帧 0x1FF 已经在 task_gimbal.c 里被 STEER_ON_CAN1
     *   开关屏蔽掉了 —— 否则两帧会互相把对方的槽清零，
     *   电机会一半时间有力、一半时间没力。 */
    CAN_Send4(&hcan1, STEER_CAN_ID, v);
#else
    CAN_Send4(&hcan2, STEER_CAN_ID, v);
#endif
}

/* ============================================================================
 *  4. BSP_CAN_Init —— "点火"（CAN1 + CAN2）
 * ==========================================================================*/
void BSP_CAN_Init(void)
{
    CAN_FilterTypeDef f = {0};

#if (CAN1_LOOPBACK_TEST == 1)
    /* ★ 自测：把 CAN1 改成回环模式。
     *   回环下自己发的帧会被自己收到，不需要总线上有其它节点应答。
     *   用途和原理见 config.h 里 CAN1_LOOPBACK_TEST 的说明。
     *   ★ 必须在配过滤器之前重新 HAL_CAN_Init，让模式生效。 */
    hcan1.Init.Mode = CAN_MODE_LOOPBACK;
    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
#endif

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
    g_init_step = 1;
    if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK)
    {
        Error_Handler();
    }

    f.FilterBank = 14;                              /* CAN2 用 14 号 */
    g_init_step = 2;
    if (HAL_CAN_ConfigFilter(&hcan2, &f) != HAL_OK)
    {
        Error_Handler();
    }

    /* ---- 启动：不调这个，CAN 一直停在初始化模式，收发全无 ---- */
    g_init_step = 3;
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    g_init_step = 4;
    if (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
        Error_Handler();
    }

    /* ---- 打开"接收 FIFO0 非空"中断 ---- */
    g_init_step = 5;
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }

    g_init_step = 6;
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }

    /* 全部成功。
     * ★ 状态快照放在【最后】：以前放在"启动 CAN1"之后，
     *   万一后面某步失败死住，g_can1_state 会永远显示 2，非常误导人。
     *   现在只有全部成功才会写它。 */
    g_init_step = 7;
    can1_diag_snapshot();
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

        /* ---- 收帧监视（只给调试用，用来看电机到底是几号）---- */
        if (hcan->Instance == CAN1)
        {
            g_can1_rx_id_last = id;
            g_can1_rx_total = g_can1_rx_total + 1U;
            if ((id >= 0x200U) && (id <= 0x20FU))
            {
                g_can1_rx_cnt[id - 0x200U] = (uint16_t)(g_can1_rx_cnt[id - 0x200U] + 1U);
            }
        }
        else
        {
            g_can2_rx_id_last = id;
            g_can2_rx_total = g_can2_rx_total + 1U;
            if ((id >= 0x200U) && (id <= 0x20FU))
            {
                g_can2_rx_cnt[id - 0x200U] = (uint16_t)(g_can2_rx_cnt[id - 0x200U] + 1U);
            }
        }

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
#if (STEER_ON_CAN1 == 0)
            else if (id == (uint16_t)(0x204U + STEER_ID))
            {
                /* 前轮转向 6020（反馈 = 0x204 + STEER_ID） */
                Steer_UpdateFeedback(angle, rpm, torque, temp);
            }
#endif
        }
        else
        {
            /* ---------------- CAN1：云台 + 发射（+ 转向，见下）---------------- */

#if (STEER_ON_CAN1 == 1)
            /* ★ 转向 6020 也接在 CAN1 上，所以写在最前面。
             *   它和 yaw 6020 的反馈帧都是 "0x204 + ID"；
             *   现在 STEER_ID = 1 会先命中这里，yaw 那一支就轮不到了 ——
             *   但云台已经挪到 CAN2，这条总线上没有 yaw 电机，所以没有影响。
             *   以后云台回到 CAN1 时，要把这两个分支的 ID 分开再理顺序。 */
            if (id == (uint16_t)(0x204U + STEER_ID))
            {
                Steer_UpdateFeedback(angle, rpm, torque, temp);
            }
            else
#endif
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
