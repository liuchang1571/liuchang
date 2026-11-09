/**
  ******************************************************************************
  * @file    drv_steer.h
  * @brief   前轮转向：GM6020 推齿条 的"位置环"
  *
  * ============================================================================
  *  一、转向是怎么工作的？
  * ============================================================================
  *      6020 电机转 -> 小齿轮转 -> 齿条左右移动 -> 拉杆推前轮 -> 车轮转向
  *
  *  所以这个模块控制的【不是车轮角度】，而是【齿条位置（毫米）】。
  *  车轮角度是机械梯形决定的，我们管不着，也不需要管。
  *
  * ============================================================================
  *  二、为什么用"位置环"？
  * ============================================================================
  *  后轮要的是"转多快"，所以用速度环。
  *  齿条要的是"停在第几毫米"，所以用位置环：
  *      目标 +15mm，实际 +3mm  -> 差 12mm -> 往正方向推
  *      目标 +15mm，实际 +15mm -> 差 0mm  -> 不推（停住）
  *
  * ============================================================================
  *  三、三个必须处理的问题
  * ============================================================================
  *  问题 1：编码器会回绕（0 和 8191 是挨着的）
  *         如果直接算"目标 - 实际"，可能得到 -8000 这种巨大差值，
  *         电机会朝反方向转一大圈。
  *         解决：把差值折算到 ±4096（±180 度），走最近的那一边。
  *
  *  问题 2：上电时不知道齿条在哪
  *         如果一上电就把目标设成 0mm（中间），
  *         而齿条恰好偏在一侧，电机会猛地把它甩过去 ——
  *         可能打到手、撞坏齿条。
  *         解决：收到第一帧反馈时，把目标设成"当前位置"，先原地不动。
  *
  *  问题 3：机械限位
  *         齿条只有 ±23mm 的行程。
  *         目标要钳位在 ±22mm；
  *         实测如果已经超过 ±24mm，说明顶死了，立刻切断输出。
  ******************************************************************************
  */
#ifndef __DRV_STEER_H
#define __DRV_STEER_H

#include "main.h"

/* 编码器一圈的计数：8192 */
#define STEER_ENC_PER_REV  8192.0f

/**
  * @brief 转向电机的全部状态
  * @note  只有一台转向电机，所以没有数组，就一个对象。
  */
typedef struct {

    /* ---------- PID 参数 ---------- */
    float kp;              /* 比例：差 1mm 给多少电压 */
    float ki;              /* 积分：一般保持 0 */
    float kd;              /* 微分：一般不用 */

    /* ---------- 标定参数（来自 config.h）---------- */
    int16_t center_raw;    /* 齿条在正中间时的编码器值 */
    float raw_per_mm;      /* 每毫米对应多少编码器计数（负数表示方向相反） */

    /* ---------- 目标 ---------- */
    int16_t target_raw;    /* 目标编码器值 0~8191 */
    float target_mm;       /* 目标齿条位置（毫米，正=向左） */

    /* ---------- 反馈 ---------- */
    int16_t actual_raw;    /* 实测编码器值 0~8191 */
    float actual_mm;       /* 实测齿条位置（毫米） */
    int16_t actual_rpm;    /* 实测转速（只显示，不参与控制） */
    int16_t torque_cur;    /* 转矩电流（只显示） */
    uint8_t temperature;   /* 温度 */
    uint32_t last_rx_tick; /* 最后一次收到反馈的时刻 */
    uint8_t have_fb;       /* 1 = 收到过反馈（上电保护的关键） */
    uint8_t online;        /* 1 = 在线 */
    uint8_t limit_hit;     /* 1 = 顶到限位了，输出被切成 0 */

    /* ---------- 编码值软件限位（用实测的两端编码值当边界）---------- */
    uint8_t limit_enabled; /* 1 = 编码值限位已生效 */
    uint8_t limit_cfg_bad; /* 1 = 实测值自相矛盾，限位被安全关闭 */
    int16_t lim_delta_lo;  /* 限位下界：相对 center_raw 的计数（一般是负的） */
    int16_t lim_delta_hi;  /* 限位上界：相对 center_raw 的计数（一般是正的） */
    int16_t actual_delta;  /* 实测位置相对中位的计数（已折算回绕） */
    uint8_t raw_limit_hit; /* 1 = 实测越界，输出被切断 */

    /* ---------- 中间量 ---------- */
    float integral;        /* 积分累计，单位"毫米·秒" */

    /* ---------- 输出 ---------- */
    int16_t give_out;      /* 发给电调的电压 -30000 ~ +30000 */

} Steer_t;

/* ============================================================================
 *  ★★ 标定用的三个变量 —— 你现在要实测的就是这三个 ★★
 * ============================================================================
 *  怎么用（全程车架空 / 前轮离地）：
 *     1. 先把 config.h 里的 STEER_PID_KP 临时改成 0.0f，烧录
 *        （kp=0 且 ki=0 时转向输出恒为 0，6020 相当于松开，用手推得动）
 *     2. 上电，调试器里展开下面这几个全局变量
 *     3. 用手把齿条推到【正中间】，读 g_steer_raw_now，记进 g_steer_raw_center
 *     4. 推到【最左端】，读 g_steer_raw_now，记进 g_steer_raw_left
 *     5. 推到【最右端】，读 g_steer_raw_now，记进 g_steer_raw_right
 *     6. STEER_PID_KP 改回 500.0f
 *
 *  ★ 这三个是普通全局变量，不是 #define 宏 ——
 *    好处是可以在调试器里直接改数值，不用重新编译烧录。
 *  ★ 初值 0 表示"还没测"。编码值范围是 0~8191，
 *    0 这个位置不可能真的拿来当机械边界，所以拿 0 当"未标定"是安全的。
 * ==========================================================================*/
extern volatile int16_t g_steer_raw_center;   /* 齿条在正中间时的编码值 0~8191 */
extern volatile int16_t g_steer_raw_left;     /* 齿条推到最左端时的编码值 0~8191 */
extern volatile int16_t g_steer_raw_right;    /* 齿条推到最右端时的编码值 0~8191 */

/* 当前编码值的镜像。
 * 只为了让你在调试器里"一眼看到现在是读数多少"，不参与任何控制。
 * 它和 Steer_Get()->actual_raw 是同一个值。 */
extern volatile int16_t g_steer_raw_now;

/* ============================================================================
 *  下面每个函数都注明了"在哪被调用"
 * ==========================================================================*/

/**
  * @brief  初始化转向电机
  * @note   调用位置：App/Task/task_chassis.c 的 Task_Chassis_Init()。【开机一次】
  * @note   实现位置：App/Driver/drv_steer.c
  * @param  kp ki kd       PID 参数
  * @param  center_raw     齿条居中时的编码器值
  * @param  raw_per_mm     每毫米多少计数
  * @retval  无
  */
void Steer_Init(float kp, float ki, float kd, int16_t center_raw, float raw_per_mm);

/**
  * @brief  拿到转向电机对象
  * @note   调用位置：task_chassis.c
  * @note   实现位置：App/Driver/drv_steer.c
  * @param   无
  * @retval  指针（永远有效）
  */
Steer_t *Steer_Get(void);

/**
  * @brief  更新反馈（只能在 CAN 中断里调用）
  * @note   调用位置：App/Bsp/bsp_can.c 的
  *                   HAL_CAN_RxFifo0MsgPendingCallback()，
  *                   每收到一帧 0x205 调一次
  * @note   实现位置：App/Driver/drv_steer.c
  * @param  angle_raw  反馈的编码器值 0~8191
  * @param  rpm        反馈转速
  * @param  torque     反馈转矩
  * @param  temp       反馈温度
  * @retval  无
  */
void Steer_UpdateFeedback(uint16_t angle_raw, int16_t rpm,
                          int16_t torque, uint8_t temp);

/**
  * @brief  设置目标齿条位置
  * @note   调用位置：App/Task/task_chassis.c 的 Task_Chassis_Loop()，1kHz
  * @note   实现位置：App/Driver/drv_steer.c
  * @param  mm  目标齿条位置（毫米，正=向左）。函数内部会做软件限位
  * @retval  无
  */
void Steer_SetTargetMM(float mm);

/**
  * @brief  算一次位置环 PID（毫米误差 -> 电压）
  * @note   调用位置：App/Task/task_chassis.c 的 Task_Chassis_Loop()，1kHz
  * @note   实现位置：App/Driver/drv_steer.c
  * @param  dt  距离上次调用的秒数
  * @retval  无（结果写在对象的 give_out 里）
  */
void Steer_Pos_PID_Calc(float dt);

/**
  * @brief  掉线检测
  * @note   调用位置：Core/Src/freertos.c 的 StartDefaultTask()，每 10ms 一次
  * @note   实现位置：App/Driver/drv_steer.c
  * @param  now_tick  当前时刻
  * @retval  无
  */
void Steer_CheckOnline(uint32_t now_tick);

#endif
