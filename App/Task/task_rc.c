/**
  ******************************************************************************
  * @file    task_rc.c
  * @brief   遥控器任务：把中断收到的字节变成全局可用的 rc_data，并判断关控
  *
  * ============================================================================
  *  这一层只做三件事：
  *     1. 有新帧  -> memcpy 进全局 rc_data
  *     2. 超过 21ms 没新帧 -> 摇杆回中 + 置关控标志
  *     3. 同步几个调试变量（给人看，不参与控制）
  ******************************************************************************
  */
#include "task_rc.h"
#include "config.h"     /* rc_data / rc_lost_flag / RC_LOST_TIMEOUT_MS */
#include "bsp_rc.h"     /* BSP_RC_TakeNewFrame 等 */
#include "drv_dbus.h"   /* DBUS_LEN / DBUS_SetSafeValue */
#include <string.h>     /* memcpy */

/* ============================================================================
 *  调试变量
 *  （不参与任何控制，只是方便你在调试器 / FreeMASTER 里看）
 * ==========================================================================*/

/* 原始的前 6 个字节。
 * 推摇杆时这几个数会变，说明串口数据真的进来了。 */
uint8_t dbg_b0;
uint8_t dbg_b1;
uint8_t dbg_b2;
uint8_t dbg_b3;
uint8_t dbg_b4;
uint8_t dbg_b5;

/* 半帧累计数。
 * ★ 正常工作时它应该一直不动。
 *   如果它在飞快地涨，说明串口线接触不良 / 波特率不对 / 接收机供电不稳。 */
uint16_t dbg_drop_cnt;

/* 最后一帧的时刻。
 * ★ 正常时它每 14ms 左右跳一次。
 *   如果它不跳，说明一帧数据都没收到。 */
uint32_t dbg_last_tick;

/* ============================================================================
 *  Task_RC_Init —— 启动接收
 *  调用者：freertos.c 的 Init_Task（StartTask04），开机一次
 * ==========================================================================*/
void Task_RC_Init(void)
{
    /* 只是转手调用 BSP 层。
     * 为什么要多这一层？
     * 因为 App 层不应该直接碰 HAL 的寄存器操作。
     * 以后要换接收机型号（比如换成 SBUS），只改 BSP 层就行。 */
    BSP_RC_Init();
}

/* ============================================================================
 *  Task_RC_Loop —— 每 1ms 执行一次
 *  调用者：task_chassis.c 的 Task_Chassis_Loop() 里的第一件事
 *
 *  一共三种情况：
 *     A. 收到新的一帧        -> 拷贝 + 解除关控
 *     B. 没新帧，但没超 21ms -> 什么都不做（继续用上一帧）
 *     C. 没新帧，且超过 21ms -> 摇杆回中 + 关控
 * ==========================================================================*/
void Task_RC_Loop(void)
{
    uint32_t now;      /* 现在的时间（毫秒） */
    uint32_t elapsed;  /* 距离上一帧过了多少毫秒 */

    /* ---------- 情况 A：收到新的一帧 ---------- */
    if (BSP_RC_TakeNewFrame())
    {
        /* 把 18 个字节从影子缓冲搬到全局 rc_data.buf。
         * 之后全工程任何地方都能用
         *     DBUS_GetChannel(&rc_data, 通道号)
         * 来取摇杆值。 */
        memcpy(rc_data.buf, BSP_RC_GetFrame(), DBUS_LEN);

        /* 收到数据了，说明遥控器活着 -> 解除关控 */
        rc_lost_flag = 0;

        /* 同步调试变量（只在有新帧时才更新，省点 CPU） */
        dbg_b0 = rc_data.buf[0];
        dbg_b1 = rc_data.buf[1];
        dbg_b2 = rc_data.buf[2];
        dbg_b3 = rc_data.buf[3];
        dbg_b4 = rc_data.buf[4];
        dbg_b5 = rc_data.buf[5];
        dbg_drop_cnt  = BSP_RC_GetDropCount();
        dbg_last_tick = BSP_RC_GetLastTick();
    }
    else
    {
        /* ---------- 情况 B / C：这一毫秒没有新数据 ---------- */
        /* 遥控器是 14ms 一帧，所以 1ms 里没新数据是正常的，
         * 只有当"距离上一帧超过 21ms"才说明真的掉线了。 */

        now = HAL_GetTick();
        elapsed = now - BSP_RC_GetLastTick();
        /* 用无符号减法：即使 HAL_GetTick() 溢出也不会算错 */

        if (elapsed > RC_LOST_TIMEOUT_MS)
        {
            /* ============ 情况 C：关控 ============ */

            rc_lost_flag = 1;

            /* ★ 摇杆回中 ★
             * 这一步就是你说的"关控时摇杆回中"。
             *
             * 为什么必须做？
             *   关控之后，rc_data 里还留着"关控前最后一帧"的摇杆位置。
             *   万一有其他代码去读它，读到的就是关控那一刻你手里的杆位，
             *   非常危险。
             *
             * DBUS_SetSafeValue 会把 rc_data 的 18 个字节
             * 改成"4 个摇杆都在中位 1024、2 个拨杆都在中位 2"。
             * 这样任何地方读 rc_data 都是安全值。
             *
             * （具体怎么算出那 6 个字节，见 drv_dbus.c 里的推导） */
            DBUS_SetSafeValue(&rc_data);
        }
        else
        {
            /* ============ 情况 B：还在 21ms 内 ============ */
            /* 什么都不做，rc_data 保持上一帧的内容 */
        }
    }
}
