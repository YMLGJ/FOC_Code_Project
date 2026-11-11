/**
 * @file    foc.h
 * @brief   FOC 磁场定向控制 — 数据结构 & 公共接口
 * @note    强拖模式：逆Park + SVPWM 生成马鞍波，驱动 PMSM 开环旋转
 */

#ifndef __FOC_H__
#define __FOC_H__

#include "stm32g4xx_hal.h"
#include "arm_math.h"

extern volatile uint8_t g_foc_event_flag;
#define FOC_EVENT_RAMP_DONE  0x01

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 电机 & 硬件参数 ======================== */
/* PWM 40kHz：中心对齐，TIM1 时钟 170MHz -> ARR = 170e6/(2*40000) - 1 ≈ 2124
 * 相对 20kHz 的收益：低电感航模电机的电流纹波直接减半，
 * 电流传感器(CC6920 ±20A)不再长期饱和。 */
/* PWM 从 40kHz 降到 20kHz。
 * 原因：实测 40kHz 下控制中断耗时 4953 周期 > 4250 周期预算（170MHz/40kHz），
 * CPU 被中断吃满 100%，主循环彻底停摆（printf/VOFA/命令全部卡死）。
 * 20kHz 下预算 8500 周期，余量充足；代价只是电流纹波略大一点。 */
#define FOC_PWM_PERIOD          4248u       /* TIM1 ARR, 中心对齐 20kHz     */
#define FOC_PWM_FREQ_HZ         20000u      /* PWM 频率 20kHz               */
#define FOC_ISR_PERIOD_US       50u         /* 中断周期 = 1/20000 = 50us    */
#define FOC_POLE_PAIRS          7u          /* 极对数（按实际电机修改！）   */
#define FOC_VBUS_DEFAULT        24.0f       /* 默认母线电压 [V]             */

#define FOC_TWO_PI              6.283185307f
#define FOC_PI                  3.141592654f
#define FOC_ONE_OVER_SQRT3      0.577350269f   /* 1/√3                      */
#define FOC_SQRT3_OVER_2        0.866025404f   /* √3/2                      */

/* 死区补偿：必须与 tim.c 里的 sBreakDeadTimeConfig.DeadTime 一致。
 * 40 counts / (2*ARR=4248) = 0.94% 占空比 → 24V 母线上等效 0.226V 电压误差，
 * 而 0.5A 只需要 0.035V —— 不补偿的话小电流下电流环完全没有控制权。 */
#define FOC_DEADTIME_COUNTS     40u
#define FOC_DEADTIME_COMP_DB    0.30f  /* 电流在 ±0.3A 内线性过渡，避免补偿抖振 */

/* DC 静态矢量测试的电流上限：超过就立即停机。
 * DC 测试是把直流直接加到绕组上，很容易失控（相电阻只有 ~0.1Ω），
 * 必须靠这个保护兑底，不能依赖电源限流。 */
#define FOC_DC_ABORT_A          1.50f

/* 无感切换默认阀值：指令电频率超过它、且观测器锁定、且相位接近时才切闭环。
 * 15Hz 电频率 ≈ 0.21 机械转/秒，此时反电势约 0.42V（远大于死区误差 0.23V）。
 * 设为 0 则禁止自动切换。 */
#define FOC_OBS_SWITCH_HZ_DEFAULT   15.0f

/* ======================== 强拖运行状态 ======================== */
typedef enum
{
    FOC_STATE_IDLE    = 0,      /* 电机停止                       */
    FOC_STATE_RAMPING = 1,      /* 频率斜坡上升中                 */
    FOC_STATE_RUNNING = 2,      /* 稳定旋转                       */
    FOC_STATE_FAULT   = 3       /* 故障（过流等，暂未实现）       */
} FOC_State_t;

/* ======================== FOC 核心结构体 ======================== */
typedef struct
{
    /* ---- 电气角度 & 速度 ---- */
    float    theta_elec;        /* 电角度 [rad], 0 ~ 2π            */
    float    omega_elec;        /* 电角速度 [rad/s]                */

    /* ---- 电流 (ADC 就绪后使用) ---- */
    float    Ia;                /* A 相电流 [A]                    */
    float    Ib;                /* B 相电流 [A]                    */
    float    Ic;                /* C 相电流 [A]                    */
    float    I_alpha;           /* Clarke α 轴电流                 */
    float    I_beta;            /* Clarke β 轴电流                 */
    float    Id;                /* Park d 轴电流（励磁分量）       */
    float    Iq;                /* Park q 轴电流（转矩分量）       */

    /* ---- 电流环 ---- */
    float    Id_ref;            /* d 轴电流参考 [A]，通常 0        */
    float    Iq_ref;            /* q 轴电流参考（斜坡后）[A]       */
    float    Iq_ref_target;     /* q 轴电流目标 [A]                */
    float    Kp_i;              /* 电流环 Kp [V/A]                 */
    float    Ki_i;              /* 电流环 Ki [V/(A·s)]             */
    float    Vd_int;            /* d 轴 PI 积分项 [V]              */
    float    Vq_int;            /* q 轴 PI 积分项 [V]              */
    uint8_t  current_loop_en;   /* 1 = 电流环使能                  */
    uint8_t  dtc_en;            /* 1 = 死区补偿使能（DTC 命令控制） */
    uint8_t  voltage_test_en;   /* 1 = 开环纯 d 轴电压测试（绕开 PI）*/
    float    Vd_test;           /* 测试用 d 轴电压 [V]             */
    float    Vq_test;           /* 测试用 q 轴电压 [V]             */
    uint8_t  dc_test_en;        /* 1 = 静态直流矢量测试（不开关）   */
    float    Ta_dc, Tb_dc, Tc_dc;  /* DC 测试的三相占空比         */
    uint32_t dc_ticks;          /* DC 测试计时，3s 自动退出        */

    /* ---- 电压指令 ---- */
    float    Vd;                /* d 轴电压 [V]                    */
    float    Vq;                /* q 轴电压 [V]                    */
    float    V_alpha;           /* α 轴电压 [V]                    */
    float    V_beta;            /* β 轴电压 [V]                    */

    /* ---- SVPWM 输出 ---- */
    float    Ta, Tb, Tc;        /* 三相占空比 [0, 1]               */
    float    Va_sin, Vb_sin, Vc_sin;  /* 零序注入前的三相正弦电压 [V] */
    uint16_t CCR1, CCR2, CCR3;  /* TIM1 比较寄存器值               */

    /* ---- 开环强拖参数 ---- */
    FOC_State_t state;          /* 当前运行状态                    */
    float    target_freq;       /* 目标电频率 [Hz]                 */
    float    current_freq;      /* 当前电频率 [Hz]                 */
    float    ramp_rate;         /* 频率斜坡速率 [Hz/s]             */
    float    voltage_setting;   /* 强拖电压幅值 [V] (Vq 指令)      */
    float    voltage_min;       /* 最小启动电压 [V]                */
    float    voltage_max;       /* 最大限制电压 [V]                */

    /* ---- 硬件参数 ---- */
    float    Vbus;              /* 直流母线电压 [V]                */
    uint16_t pwm_period;        /* PWM 周期 (ARR)                  */
    float    pwm_period_f;      /* PWM 周期 (float)                */
    uint8_t  pole_pairs;        /* 极对数                          */

    /* ---- 运行时计数器 ---- */
    uint32_t loop_count;        /* 电流环迭代次数                  */

    /* ---- 无感观测 / I-f 切换 ---- */
    float    theta_if;          /* I/f 积分角（开环参考）          */
    uint8_t  sensorless_en;     /* 1 = 当前使用观测器角度          */
    uint8_t  sensorless_auto;   /* 1 = 允许自动切换                */
    float    obs_switch_hz;     /* 自动切换阈值（电频率 Hz），0=禁用*/

} FOC_Handle_t;

/* ======================== 公共接口 ======================== */

/** 初始化 FOC 句柄 */
void FOC_Init(FOC_Handle_t *foc);

/** 启动强拖（设置目标频率和电压后调用） */
void FOC_StartForcedRotation(FOC_Handle_t *foc,
                             float target_freq_hz,
                             float voltage_v);

/** 启动电流环模式（I/f + d/q 电流环） */
void FOC_StartCurrentMode(FOC_Handle_t *foc,
                          float target_freq_hz,
                          float iq_ref_a);

/** 停止电机 */
void FOC_Stop(FOC_Handle_t *foc);

/** 强拖迭代 —— 每个 PWM 周期调用一次（由 TIM1 更新中断触发） */
void FOC_ForcedRotation_ISR(FOC_Handle_t *foc);

/** 向 TIM1 写入新的比较值 */
void FOC_UpdatePWM(FOC_Handle_t *foc);

/** 获取 FOC 句柄指针（供 ISR 和 Debug 使用） */
FOC_Handle_t *FOC_GetHandle(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_H__ */