/**
  ******************************************************************************
  * @file    bsp_rc.h
  * @brief   遥控器接收（BSP 层）—— USART3 + DMA + 空闲中断
  *
  * ============================================================================
  *  这一层只干一件事：把串口线上的字节，安全地搬进一块内存。
  *  它不解析数据、不判断关控、不碰电机。
  * ============================================================================
  *
  * ============================================================================
  *  整个接收过程（这就是你说的"空闲中断"方案）
  * ============================================================================
  *
  *  第 1 步：BSP_RC_Init() 被调用（在初始化任务里，开机一次）
  *          -> 告诉 DMA：串口来一个字节，你就自动往 s_dma_buf 里放一个，
  *             一共放 18 个
  *          -> 再告诉串口：数据线一空闲下来，就产生中断叫我
  *
  *  第 2 步：接收机开始每 14ms 送 18 个字节。
  *          这 18 个字节全靠 DMA 硬件搬，CPU 完全不参与。
  *
  *  第 3 步：18 个字节送完之后，数据线会空闲下来
  *          （下一帧要 14ms 后才来）。
  *          串口硬件发现"线路空闲超过 1 个字节的时间"，
  *          就把 IDLE 标志置起来，产生中断。
  *          ★ 这就是"空闲中断"这个名字的来历。
  *
  *  第 4 步：USART3_IRQHandler 里调用 BSP_RC_IrqHandler()，
  *          把 18 个字节从 s_dma_buf 拷到 s_frame，
  *          记下时间，把 s_new_frame 置 1。
  *
  *  第 5 步：底盘任务（1kHz）每次循环问一次
  *          BSP_RC_TakeNewFrame()，
  *          返回 1 就说明有新数据，去拿。
  *
  * ============================================================================
  *  为什么要两个缓冲区？（s_dma_buf 和 s_frame）★ 重点 ★
  * ============================================================================
  *  DMA 是"自己偷偷往内存里写"的，它不等你。
  *
  *  假如任务直接读 s_dma_buf，可能读到一半的时候，
  *  下一帧的第一个字节进来了，把后面改掉了。
  *  你就拿到"前 9 个字节是新的、后 9 个是旧的"这种烂数据，
  *  摇杆值会瞬间乱跳，车会抽搐一下。
  *
  *  所以：
  *      DMA 只写 s_dma_buf
  *      中断里一次性拷到 s_frame
  *      任务只读 s_frame
  *  （18 个字节的 memcpy 大约 1 微秒，完全不影响 14ms 的周期）
  ******************************************************************************
  */
#ifndef __BSP_RC_H
#define __BSP_RC_H

#include "main.h"
#include "drv_dbus.h"      /* DBUS_LEN（=18）和 RC_Ctl_Define_t */

/**
  * @brief  启动遥控器接收（开 DMA + 开空闲中断）
  * @note   调用位置：App/Task/task_rc.c 的 Task_RC_Init()，
  *                   而它又被 Core/Src/freertos.c 的 Init_Task 调用。
  *                   【开机只调用一次】
  * @note   实现位置：App/Bsp/bsp_rc.c
  * @param   无
  * @retval  无
  */
void BSP_RC_Init(void);

/**
  * @brief  串口3中断的处理入口
  * @note   调用位置：Core/Src/stm32f4xx_it.c 的 USART3_IRQHandler() 里，
  *                   而且必须在 HAL_UART_IRQHandler(&huart3) 【之前】调用。
  *                   【每收到一包数据（14ms）调用一次】
  * @note   实现位置：App/Bsp/bsp_rc.c
  * @param   无
  * @retval  无
  *
  * @note   ★ 为什么必须放在 HAL 的函数之前？
  *           HAL 的函数会把"溢出错误"标志清掉。
  *           如果你放在后面，你的"错误自愈"代码永远进不去。
  */
void BSP_RC_IrqHandler(void);

/**
  * @brief  拿到"影子缓冲"的地址（只读，不要往里写）
  * @note   调用位置：App/Task/task_rc.c 的 Task_RC_Loop()，
  *                   确认有新帧之后才调用
  * @note   实现位置：App/Bsp/bsp_rc.c
  * @param   无
  * @retval  指向 18 字节数据的指针
  */
const uint8_t *BSP_RC_GetFrame(void);

/**
  * @brief  问一句"有没有新的一帧"，并且【自动清零】
  * @note   调用位置：App/Task/task_rc.c 的 Task_RC_Loop()，每 1ms 一次
  * @note   实现位置：App/Bsp/bsp_rc.c
  * @param   无
  * @retval  1 = 有新数据，0 = 没有
  *
  * @note   ★ 为什么叫 Take（拿走）？
  *           读的同时把标志清 0，所以同一帧不会被处理两次。
  *           比"先 Get 再 Set(0)"两步操作安全。
  */
uint8_t BSP_RC_TakeNewFrame(void);

/**
  * @brief  拿"最后一帧数据到达的时刻"
  * @note   调用位置：App/Task/task_rc.c 的 Task_RC_Loop()，
  *                   用来算"距离上一帧过了多少毫秒"，判断关控
  * @note   实现位置：App/Bsp/bsp_rc.c
  * @param   无
  * @retval  收到最后一帧时 HAL_GetTick() 的值（毫秒）
  */
uint32_t BSP_RC_GetLastTick(void);

/**
  * @brief  拿"半帧 / 错帧的累计次数"
  * @note   调用位置：App/Task/task_rc.c 里同步给 dbg_drop_cnt，供调试查看
  * @note   实现位置：App/Bsp/bsp_rc.c
  * @param   无
  * @retval  累计丢弃的帧数
  *
  * @note   ★ 这个值很有用。
  *           如果它在飞快地涨，说明串口线接触不良、
  *           或者波特率不对、或者接收机供电不稳。
  *           正常工作时它应该一直不动。
  */
uint16_t BSP_RC_GetDropCount(void);

#endif
