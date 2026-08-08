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
#define FOC_PWM_PERIOD          4249u       /* TIM1 ARR, 中心对齐           */
#define FOC_PWM_FREQ_HZ         20000u      /* PWM 频率 20kHz               */
#define FOC_ISR_PERIOD_US       50u         /* 中断周期 = 1/20000 = 50us   */
#define FOC_POLE_PAIRS          7u          /* 极对数（按实际电机修改！）   */
#define FOC_VBUS_DEFAULT        12.0f       /* 默认母线电压 [V]             */

#define FOC_TWO_PI              6.283185307f
#define FOC_ONE_OVER_SQRT3      0.577350269f   /* 1/√3                      */
#define FOC_SQRT3_OVER_2        0.866025404f   /* √3/2                      */

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
    float    I_alpha;           /* Clarke α 轴电流                 */
    float    I_beta;            /* Clarke β 轴电流                 */
    float    Id;                /* Park d 轴电流（励磁分量）       */
    float    Iq;                /* Park q 轴电流（转矩分量）       */

    /* ---- 电压指令 ---- */
    float    Vd;                /* d 轴电压 [V]                    */
    float    Vq;                /* q 轴电压 [V]                    */
    float    V_alpha;           /* α 轴电压 [V]                    */
    float    V_beta;            /* β 轴电压 [V]                    */

    /* ---- SVPWM 输出 ---- */
    float    Ta, Tb, Tc;        /* 三相占空比 [0, 1]               */
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

} FOC_Handle_t;

/* ======================== 公共接口 ======================== */

/** 初始化 FOC 句柄 */
void FOC_Init(FOC_Handle_t *foc);

/** 启动强拖（设置目标频率和电压后调用） */
void FOC_StartForcedRotation(FOC_Handle_t *foc,
                             float target_freq_hz,
                             float voltage_v);

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