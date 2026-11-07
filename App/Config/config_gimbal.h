/**
  ******************************************************************************
  * @file    config_gimbal.h
  * @brief   云台（yaw 6020 + pitch 3508）的参数表
  *
  * ============================================================================
  *  总线分配（整车两条 CAN）
  * ============================================================================
  *   CAN2：后轮 3508(ID1) 3508(ID2)  -> 0x200   （App/Bsp/bsp_can.c 里已有）
  *        转向 6020(ID1)             -> 0x1FF
  *
  *   CAN1：pitch 3508(ID1)            -> 0x200 槽0   ← 本文件管
  *        yaw   6020(ID1)            -> 0x1FF 槽0
  *        （以后）拨弹(ID2) 摩擦L(ID3) 摩擦R(ID4) -> 0x200 槽1~3
  *
  *   ★ 两条总线上的 ID 各自独立，所以 yaw 和转向都可以是 ID1、都用 0x1FF/0x205。
  *   ★ CAN1 的 0x1FF 只有 yaw 一个主人、0x200 只有 3508，不存在共用帧。
  ******************************************************************************
  */
#ifndef __CONFIG_GIMBAL_H
#define __CONFIG_GIMBAL_H

/* ============================================================================
 *  一、YAW（GM6020，CAN1）
 * ==========================================================================*/
#define YAW_6020_ID            1        /* 反馈 = 0x204+1 = 0x205（在 CAN1 上） */

/* 给正电压时 vel_dps 应该变正；台架验证，反了改 -1.0f */
#define YAW_ESC_DIR            (+1.0f)

/* 编码器转 1° 对应关节转多少度：
     直驱 -> 1.0f
     有减速 G:1 -> 1.0f/G
     方向相反 -> 取负
   ★ 这一步和 yaw_rel 的符号绑在一起，标定完 yaw_rel 往左转为正即可。 */
#define YAW_JOINT_PER_ENC      (+1.0f)

/* ★ 待标定：云台炮口与车头同向时，6020 的 angle_deg（0~360） */
#define YAW_ZERO_ENC_DEG       (0.0f)

/* ★ yaw 关节软限位（没有滑环，±90° 是线缆的安全角度） */
#define YAW_JOINT_LIMIT_MIN    (-90.0f)
#define YAW_JOINT_LIMIT_MAX    ( 90.0f)

/* 摇杆推满 = 180°/s（每个 1ms 加 0.18°） */
#define YAW_RATE_S             180.0f

/* ---- yaw 位置环：关节角误差(deg) -> 关节角速度(deg/s) ----
   ★ 现在全部为 0：先验证角度，电机不出力；调的时候按 README 的顺序解冻 */
#define YAW_POS_KP             0.0f
#define YAW_POS_KI             0.0f
#define YAW_POS_OUT_MAX        400.0f
#define YAW_POS_I_MAX          20.0f

/* ---- yaw 速度环：角速度(deg/s) -> 电压 ---- */
#define YAW_VEL_KP             0.0f
#define YAW_VEL_KI             0.0f
#define YAW_VEL_I_MAX          800.0f
#define YAW_VEL_OUT_MAX        25000.0f

/* ============================================================================
 *  二、小 PITCH（M3508 + C620，CAN1）
 * ==========================================================================*/
#define PITCH_3508_ID          1        /* 反馈 = 0x201（在 CAN1 上） */

/* ★ 待标定：3508 输出轴转 1° 对应关节转多少度（方向相反取负） */
#define PITCH_GEAR_JOINT_PER_OUT  (+2.0f)

/* ★ 上电姿态 = 关节 -15°（靠重力落在软限位最低点），使能瞬间不会动 */
#define PITCH_BOOT_JOINT_DEG   (-15.0f)

/* 关节软限位 */
#define PITCH_LIMIT_MIN        (-15.0f)
#define PITCH_LIMIT_MAX        (+25.0f)

/* 摇杆推满 = 120°/s（每个 1ms 加 0.12°） */
#define PITCH_RATE_S           120.0f

/* 电流限幅（满量程 ±16384；初调从 1500 起） */
#define PITCH_CUR_MAX          8000.0f

/* 反馈超时（ms）：超过就断电流 */
#define GIMBAL_FB_TIMEOUT_MS   50

/* ---- pitch 位置环：关节角误差(deg) -> 转子 rpm ----
   kp 内部会乘上 19/(6*K) 的换算，所以这里填的是"误差 1° 给多少关节 °/s" */
#define PITCH_POS_KP           0.0f
#define PITCH_POS_OUT_MAX      500.0f   /* 转子 rpm */

/* ---- pitch 速度环：转子 rpm -> 电流 ---- */
#define PITCH_VEL_KP           0.0f
#define PITCH_VEL_KI           0.0f
#define PITCH_VEL_I_MAX        200.0f
#define PITCH_VEL_OUT_MAX      8000.0f

/* 重力前馈（电流单位，正 = 抬头方向；先 0） */
#define PITCH_GRAV_FF_CUR      0.0f

/* ============================================================================
 *  三、摇杆（和 gimbal_board 一致：增量式，推着转、松手停）
 * ==========================================================================*/
#define GIMBAL_CH_MID          1024     /* 摇杆中位 */
#define GIMBAL_CH_RANGE        660.0f   /* 中位到最边的偏移 */
#define GIMBAL_DEAD_ZONE_RAW   20       /* 死区（12 位原始值） */
#define GIMBAL_PERIOD_S        0.001f   /* 1ms 周期 */

#endif /* __CONFIG_GIMBAL_H */
