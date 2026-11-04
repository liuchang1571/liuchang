/**
  ******************************************************************************
  * @file    bsp_can.c
  * @brief   CAN2 收发实现
  *
  *  阅读顺序（也是本文件的书写顺序）：
  *     1. CAN_Send4()                        内部工具：发一帧 4 路数据
  *     2. BSP_CAN_Send_Drive_Current()       对外：后轮电流
  *     3. BSP_CAN_Send_Steer_Voltage()       对外：前轮转向电压
  *     4. BSP_CAN_Init()                     对外：过滤器 + 启动 + 开中断
  *     5. HAL_CAN_RxFifo0MsgPendingCallback() 中断回调：解析反馈
  ******************************************************************************
  */
#include "bsp_can.h"
#include "can.h"           /* hcan2（CubeMX 在 can.c 里定义的 CAN 句柄） */
#include "config.h"        /* 电机 ID、帧 ID、各种上限 */
#include "drv_motor.h"     /* Motor_UpdateFeedback() */
#include "drv_steer.h"     /* Steer_UpdateFeedback() */

/* hcan2 是在 Core/Src/can.c 里定义的，这里声明一下才能用 */
extern CAN_HandleTypeDef hcan2;

/* ---------------------------------------------------------------------------
 *  编译期检查：电调 ID 必须在 1~4
 * ---------------------------------------------------------------------------
 *  因为下面的代码用 "ID 减 1" 当作数组下标。
 *  如果 ID 写成 0 或 5，数组就跑到外面去了，会写坏别的内存。
 *  #if 是在【编译的时候】检查，比到现场才发现问题好得多。
 * ------------------------------------------------------------------------- */
#if (DRIVE_ID_L < 1) || (DRIVE_ID_L > 4) || (DRIVE_ID_R < 1) || (DRIVE_ID_R > 4)
#error "config.h 里 DRIVE_ID_L / DRIVE_ID_R 必须在 1~4 之间"
#endif
#if (STEER_ID < 1) || (STEER_ID > 4)
#error "config.h 里 STEER_ID 必须在 1~4 之间"
#endif

/* ============================================================================
 *  1. CAN_Send4 —— 内部工具函数
 * ----------------------------------------------------------------------------
 *  大疆电调"一帧控 4 个电机"的格式：
 *      第 1 个电机的值 -> 字节 0~1
 *      第 2 个电机的值 -> 字节 2~3
 *      第 3 个电机的值 -> 字节 4~5
 *      第 4 个电机的值 -> 字节 6~7
 *  每个值是 int16（有符号 16 位），而且【高字节在前】。
 *
 *  @param  std_id  帧 ID：0x200（电流）或 0x1FF（电压）
 *  @param  val     4 个值。val[0] 对应 ID1，val[1] 对应 ID2 ...
 *
 *  @note 调用者：BSP_CAN_Send_Drive_Current / BSP_CAN_Send_Steer_Voltage
 * ==========================================================================*/
static void CAN_Send4(uint16_t std_id, const int16_t val[4])
{
    CAN_TxHeaderTypeDef tx;     /* 发送报文头，相当于快递单 */
    uint32_t mailbox;           /* 硬件会告诉你是用了哪个邮箱，这里接着就行 */
    uint8_t data[8];            /* 要发出去的 8 个字节 */
    uint16_t u;                 /* 把 val[i] 当成无符号数来看 */
    uint8_t i;                  /* 循环变量 */

    /* ---- 第 1 步：把 4 个 int16 拆成 8 个字节，高字节在前 ---- */
    for (i = 0; i < 4; i++)
    {
        /* 先强转成 uint16 再移位。
         * 为什么必须转？因为 val[i] 可能是负数，
         * 而 C 语言里"负数右移"的行为不太好把握。
         * 转成无符号之后右移，就是纯粹的"把高 8 位挪下来"，绝对不会错。 */
        u = (uint16_t)val[i];

        data[i * 2]     = u >> 8;      /* 高 8 位放在前面 */
        data[i * 2 + 1] = u & 0xFF;    /* 低 8 位放在后面 */
    }
    /* ★ 如果这里写成"低字节在前"，电调读到的电流会是一个完全错误的值
     *   （比如 -100 会被当成 -25600），车会直接失控。
     *   这是最容易犯、后果最严重的错。 */

    /* ---- 第 2 步：填"快递单" ---- */
    tx.StdId              = std_id;        /* 标准帧 ID */
    tx.ExtId              = 0;             /* 扩展帧 ID，我们不用 */
    tx.IDE                = CAN_ID_STD;    /* 用标准帧（11 位 ID） */
    tx.RTR                = CAN_RTR_DATA;  /* 数据帧（不是远程帧） */
    tx.DLC                = 8;             /* 数据长度 8 字节 */
    tx.TransmitGlobalTime = DISABLE;       /* 这一项一定要赋值！
                                              结构体是局部变量，不赋值就是随机数，
                                              可能导致发送失败。 */

    /* ---- 第 3 步：看看还有没有空闲邮箱 ---- */
    /* F407 的 CAN 有 3 个发送邮箱，同一时间最多排 3 帧。
     * 如果 3 个都在用（总线忙，或者没人应答），这里就直接放弃这一帧。
     *
     * ★ 为什么不用 while 死等？
     *   如果总线断了（比如没接电调，没人给应答信号），
     *   邮箱会一直是满的。死等会把 1kHz 的控制任务彻底卡死，
     *   整辆车失去响应。
     *   宁可丢一帧（下一毫秒还会再发），也不能卡住。 */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0)
    {
        return;
    }

    /* ---- 第 4 步：真正发送 ---- */
    /* 这个函数只是把数据放进邮箱，
     * 硬件什么时候真正发到总线上，是下一微秒的事。 */
    HAL_CAN_AddTxMessage(&hcan2, &tx, data, &mailbox);
}

/* ============================================================================
 *  2. BSP_CAN_Send_Drive_Current —— 后轮电流
 *  调用者：task_chassis.c 的 Task_Chassis_Loop()，1kHz
 * ==========================================================================*/
void BSP_CAN_Send_Drive_Current(int16_t left, int16_t right)
{
    /* 先准备 4 个位置，全部填 0。
     * 我们只用了 2 个电机，剩下 2 个位置必须填 0，
     * 否则对应 ID 的电机会乱动。 */
    int16_t c[4] = { 0, 0, 0, 0 };

    /* 按 ID 放到正确的位置。
     * 数组从 0 开始数，ID 从 1 开始数，
     * 所以"ID 减 1"就是数组下标。
     *   例：DRIVE_ID_L 是 1，就放到 c[0]。 */
    c[DRIVE_ID_L - 1] = left;
    c[DRIVE_ID_R - 1] = right;

    /* 用 0x200（C620 的电流控制帧）发出去 */
    CAN_Send4(DRIVE_CAN_ID, c);
}

/* ============================================================================
 *  3. BSP_CAN_Send_Steer_Voltage —— 前轮转向电压
 *  调用者：task_chassis.c 的 Task_Chassis_Loop()，1kHz
 *
 *  只有一台转向电机（6020 推齿条），所以只填一个位置，其余 3 个留 0。
 * ==========================================================================*/
void BSP_CAN_Send_Steer_Voltage(int16_t voltage)
{
    int16_t v[4] = { 0, 0, 0, 0 };

    /* STEER_ID 是 1，就放到 v[0]；是 2，就放到 v[1] ... */
    v[STEER_ID - 1] = voltage;

    /* 用 0x1FF（GM6020 的电压控制帧）发出去。
     * 注意：0x1FF 同时也是"C620 的 ID 5~8"的控制帧，
     * 所以千万不要把 3508 的 ID 设成 5~8。 */
    CAN_Send4(STEER_CAN_ID, v);
}

/* ============================================================================
 *  4. BSP_CAN_Init —— "点火"
 *  调用者：freertos.c 的 Init_Task（StartTask04），开机一次
 *
 *  一共三步，缺一不可：
 *     ① 配置过滤器      （不配就收不到反馈）
 *     ② HAL_CAN_Start   （不调就发不出去，也收不进来）
 *     ③ 打开接收中断    （不开就进不了下面第 5 节的回调函数）
 * ==========================================================================*/
void BSP_CAN_Init(void)
{
    CAN_FilterTypeDef f = {0};      /* 过滤器配置，先全部清 0 */

    /* ==================================================================
     * 第 ① 步：配置接收过滤器
     * ==================================================================
     * 【F407 的一个坑】CAN1 和 CAN2 共用 28 组过滤器（编号 0~27）：
     *      0 ~ 13 号   归 CAN1
     *      14 ~ 27 号  归 CAN2
     * 而"CAN2 从第几组开始"这个设置，是写在 CAN1 的寄存器里的。
     * 每次调用 HAL_CAN_ConfigFilter 都会重新写这个设置，
     * 所以【不管配哪个 CAN，都必须带 SlaveStartFilterBank = 14】。
     * ================================================================== */
    f.FilterMode           = CAN_FILTERMODE_IDMASK;  /* 掩码模式：匹配一段 ID 范围 */
    f.FilterScale          = CAN_FILTERSCALE_32BIT;  /* 32 位过滤器 */
    f.FilterIdHigh         = (0x200 << 5);           /* 要匹配的 ID 是 0x200。
                                                        ★ 左移 5 位是 STM32 的硬件规定：
                                                          11 位 ID 在寄存器里从第 5 位开始摆。 */
    f.FilterIdLow          = 0;                      /* 低位不用管 */
    f.FilterMaskIdHigh     = (0x7F0 << 5);           /* 掩码 0x7F0。
                                                        ★ 掩码的含义是"哪几位必须一模一样"。
                                                          0x7F0 = 二进制 0111 1111 0000，
                                                          高 7 位必须等于 0x200 的高 7 位，
                                                          低 4 位随便。
                                                          所以 0x200 ~ 0x20F 全部放行，
                                                          正好覆盖 0x201~0x208 的反馈。 */
    f.FilterMaskIdLow      = 0;
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;       /* 收到的帧放进 FIFO0（对应 RX0 中断） */
    f.FilterBank           = 14;                     /* 用 14 号过滤器（CAN2 的第一组） */
    f.FilterActivation     = ENABLE;                 /* 启用 */
    f.SlaveStartFilterBank = 14;                     /* ★ 固定填 14，别改，原因见上面 */

    if (HAL_CAN_ConfigFilter(&hcan2, &f) != HAL_OK)
    {
        Error_Handler();        /* 配置失败就停在这里，方便查问题 */
    }

    /* ==================================================================
     * 第 ② 步：启动 CAN（最关键的一行）
     * ==================================================================
     * MX_CAN2_Init() 执行完之后，CAN 还停在"初始化模式"。
     * 在这个模式下：
     *     - 不会往总线上发任何东西
     *     - 不会接收任何东西
     *     - 但 AddTxMessage() 却会返回"成功"（因为邮箱确实是空的）
     * 这就是"程序说发送成功、示波器上却什么都没有"的原因。
     * HAL_CAN_Start() 会让 CAN 真正加入总线。
     * ================================================================== */
    if (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
        Error_Handler();
    }

    /* ==================================================================
     * 第 ③ 步：打开"接收 FIFO0 非空"中断
     * ==================================================================
     * 只有打开这个中断，收到帧的时候才会跳到
     * 下面第 5 节的 HAL_CAN_RxFifo0MsgPendingCallback()。
     * ================================================================== */
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ============================================================================
 *  5. HAL_CAN_RxFifo0MsgPendingCallback —— CAN 接收中断回调
 * ----------------------------------------------------------------------------
 *  【背景知识】HAL 库里本来就有这个函数，但它是个空函数（弱符号）。
 *  我们自己在这里重新写一份，HAL 就会调用我们这份。
 *
 *  调用链：
 *      CAN2_RX0_IRQHandler()（stm32f4xx_it.c）
 *          -> HAL_CAN_IRQHandler(&hcan2)
 *          -> 本函数
 *
 *  调用频率：每个电机每 1ms 发一次，4 个电机就是每秒 4000 次。
 *
 *  ★ 中断里只做"解析 + 赋值"，绝对不要做 PID、不要做浮点除法、
 *    不要调 HAL_Delay、不要发 CAN。中断要尽快结束，
 *    否则会把 1kHz 的控制循环拖慢。
 * ==========================================================================*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx;     /* 收到的报文头（里面有 ID） */
    uint8_t data[8];            /* 收到的 8 个字节 */
    uint16_t id;                /* 帧 ID */
    uint16_t angle;             /* 转子机械角度 0~8191 */
    int16_t  rpm;               /* 转子转速 */
    int16_t  torque;            /* 转矩电流 */
    uint8_t  temp;              /* 温度 */

    /* 只处理 CAN2（以后如果 CAN1 也开了中断，这句能防止串台） */
    if (hcan->Instance != CAN2)
    {
        return;
    }

    /* 一次中断里可能堆积了好几帧，所以循环读到 FIFO 空为止。
     * ★ 如果只读一帧就走，堆积的帧会越来越多，
     *   最后 FIFO 溢出，反馈就丢了，
     *   PID 就"看不见"车轮的真实转速了。 */
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        /* 从 FIFO 里取一帧 */
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, data) != HAL_OK)
        {
            return;
        }

        /* 我们只用标准帧、只用 8 字节的数据帧，别的跳过 */
        if (rx.IDE != CAN_ID_STD || rx.DLC != 8)
        {
            continue;
        }

        id = rx.StdId;

        /* 按协议把字节拼成数值，同样是【高字节在前】 */
        angle  = (data[0] << 8) | data[1];    /* 字节 0~1 */
        rpm    = (data[2] << 8) | data[3];    /* 字节 2~3 */
        torque = (data[4] << 8) | data[5];    /* 字节 4~5 */
        temp   = data[6];                     /* 字节 6    */

        /* ---- 按 ID 分发给对应的电机 ---- */
        if (id >= 0x201 && id <= 0x204)
        {
            /* 0x201~0x204 是【后轮 3508】的反馈。
             *   ID 1 -> 0x201 -> 数组下标 0
             *   ID 2 -> 0x202 -> 数组下标 1
             * 后轮做速度环，只需要转速，不需要角度。 */
            Motor_UpdateFeedback(id - 0x201, rpm, torque, temp);
        }
        else if (id == 0x204 + STEER_ID)
        {
            /* 前轮 6020 的反馈。
             *   ID 1 -> 0x205
             * 转向做位置环，必须要角度。 */
            Steer_UpdateFeedback(angle, rpm, torque, temp);
        }
        else
        {
            /* 其它 ID（比如 0x200 是我们自己发的）忽略 */
        }
    }
}
