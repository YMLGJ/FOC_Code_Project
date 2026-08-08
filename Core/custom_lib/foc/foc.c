/**
 * @file    foc.c
 * @brief   FOC 核心算法 — 强拖模式（逆 Park → SVPWM 马鞍波）
 */

#include "foc.h"
#include <string.h>
#include <stdio.h> 
#include <math.h>
#include "tim.h" 



/* ================================================================== */
/*                    全局句柄（单例）                                  */
/* ================================================================== */
static FOC_Handle_t g_foc;          /* 全局 FOC 实例 */
static FOC_Handle_t *p_foc = NULL;  /* 快速访问指针   */
volatile uint8_t g_foc_event_flag = 0;//事件标志位，用于通知主循环处理事件


FOC_Handle_t *FOC_GetHandle(void)
{
    return &g_foc;
}

/* ================================================================== */
/*                        初始化                                      */
/* ================================================================== */
void FOC_Init(FOC_Handle_t *foc)
{
    if (foc == NULL) return;

    memset(foc, 0, sizeof(FOC_Handle_t));

    /* 硬件参数 */
    foc->pwm_period   = FOC_PWM_PERIOD;
    foc->pwm_period_f = (float)FOC_PWM_PERIOD;
    foc->Vbus         = FOC_VBUS_DEFAULT;
    foc->pole_pairs   = FOC_POLE_PAIRS;

    /* 默认强拖参数 */
    foc->state            = FOC_STATE_IDLE;
    foc->target_freq      = 0.0f;
    foc->current_freq     = 0.0f;
    foc->ramp_rate        = 20.0f;     /* 默认 20 Hz/s 斜坡 */
    foc->voltage_setting  = 2.0f;      /* 默认 2V，很保守   */
    foc->voltage_min      = 0.5f;      /* 最低 0.5V 启动    */
    foc->voltage_max      = foc->Vbus * 0.8f;  /* 限制 80% Vbus */


    /* 保存全局指针 */
    p_foc = foc;

    printf("FOC_Init: OK, PWM=%uHz, PolePairs=%u\r\n",
           FOC_PWM_FREQ_HZ, foc->pole_pairs);
}

/* ================================================================== */
/*                    启动强拖                                        */
/* ================================================================== */
void FOC_StartForcedRotation(FOC_Handle_t *foc,
                             float target_freq_hz,
                             float voltage_v)
{
    if (foc == NULL) return;

    /* 参数合理性检查 */
    if (target_freq_hz < 0.5f)  target_freq_hz = 0.5f;
    if (target_freq_hz > 200.0f) target_freq_hz = 200.0f;

    if (voltage_v < foc->voltage_min) voltage_v = foc->voltage_min;
    if (voltage_v > foc->voltage_max) voltage_v = foc->voltage_max;

    foc->target_freq     = target_freq_hz;
    foc->voltage_setting = voltage_v;
    foc->current_freq    = 0.0f;
    foc->theta_elec      = 0.0f;
    foc->loop_count      = 0;

    foc->state = FOC_STATE_RAMPING;

    printf("FOC_Start: Target=%.1fHz, Voltage=%.2fV\r\n",
           target_freq_hz, voltage_v);
}

/* ================================================================== */
/*                    停止电机                                        */
/* ================================================================== */
void FOC_Stop(FOC_Handle_t *foc)
{
    if (foc == NULL) return;

    foc->state        = FOC_STATE_IDLE;
    foc->target_freq  = 0.0f;
    foc->current_freq = 0.0f;
    foc->theta_elec   = 0.0f;
    foc->Vd           = 0.0f;
    foc->Vq           = 0.0f;
    foc->V_alpha      = 0.0f;
    foc->V_beta       = 0.0f;

    /* 输出 50% 占空比（三相中点，电机不转） */
    foc->Ta = 0.5f;
    foc->Tb = 0.5f;
    foc->Tc = 0.5f;
    FOC_UpdatePWM(foc);

    printf("FOC_Stop\r\n");
}

/* ================================================================== */
/*              逆 Park 变换: dq → αβ                                 */
/*              V_alpha = Vd*cos(θ) - Vq*sin(θ)                       */
/*              V_beta  = Vd*sin(θ) + Vq*cos(θ)                       */
/* ================================================================== */
static void FOC_InvPark(FOC_Handle_t *foc)
{
    float cos_theta = arm_cos_f32(foc->theta_elec);
    float sin_theta = arm_sin_f32(foc->theta_elec);

    foc->V_alpha = foc->Vd * cos_theta - foc->Vq * sin_theta;
    foc->V_beta  = foc->Vd * sin_theta + foc->Vq * cos_theta;
}

/* ================================================================== */
/*              逆 Clarke: αβ → 三相电压（SVPWM 内部）                */
/* ================================================================== */
static void FOC_InvClarke(float V_alpha, float V_beta,
                          float *Va, float *Vb, float *Vc)
{
    *Va =  V_alpha;
    *Vb = -0.5f * V_alpha + FOC_SQRT3_OVER_2 * V_beta;
    *Vc = -0.5f * V_alpha - FOC_SQRT3_OVER_2 * V_beta;
}

/* ================================================================== */
/*          SVPWM（最小值-最大值注入法 = 等效七段式 SVPWM）            */
/*          输入: V_alpha, V_beta, Vbus                                */
/*          输出: Ta, Tb, Tc ∈ [0, 1]  →  CCR1/2/3                    */
/* ================================================================== */
static void FOC_SVPWM(FOC_Handle_t *foc)
{
    float Va, Vb, Vc;
    float Vmin, Vmax, Voffset;
    float half_vbus;
    float duty;

    /* 1. αβ → 三相正弦电压 */
    FOC_InvClarke(foc->V_alpha, foc->V_beta, &Va, &Vb, &Vc);

    /* 2. 计算零序注入分量 = (Vmax + Vmin) / 2
     *    这是 SVPWM 与 SPWM 的唯一区别 —— 一行代码带来 15% 电压裕量 */
    Vmin = Va;
    if (Vb < Vmin) Vmin = Vb;
    if (Vc < Vmin) Vmin = Vc;

    Vmax = Va;
    if (Vb > Vmax) Vmax = Vb;
    if (Vc > Vmax) Vmax = Vc;

    Voffset = (Vmin + Vmax) * 0.5f;

    /* 3. 注入零序 → 马鞍波形 */
    Va -= Voffset;
    Vb -= Voffset;
    Vc -= Voffset;

    /* 4. 电压 → 占空比 [0, 1]
     *    占空比 = (Vx / (Vbus/2) + 1) / 2
     *    中点电压 Vbus/2 对应 50% 占空比 */
    half_vbus = foc->Vbus * 0.5f;
    if (half_vbus < 0.1f) half_vbus = 0.1f;  /* 防除零 */

    /* A 相 */
    duty = (Va / half_vbus + 1.0f) * 0.5f;
    if      (duty > 1.0f) duty = 1.0f;
    else if (duty < 0.0f) duty = 0.0f;
    foc->Ta = duty;

    /* B 相 */
    duty = (Vb / half_vbus + 1.0f) * 0.5f;
    if      (duty > 1.0f) duty = 1.0f;
    else if (duty < 0.0f) duty = 0.0f;
    foc->Tb = duty;

    /* C 相 */
    duty = (Vc / half_vbus + 1.0f) * 0.5f;
    if      (duty > 1.0f) duty = 1.0f;
    else if (duty < 0.0f) duty = 0.0f;
    foc->Tc = duty;

    /* 5. 占空比 → TIM1 CCR 寄存器值
     *    PWM 模式 2, 中心对齐:
     *    CCR = ARR * (1 - duty)  →  duty=0.5 时 CCR=ARR/2=2125 ✓ */
    foc->CCR1 = (uint16_t)((1.0f - foc->Ta) * foc->pwm_period_f);
    foc->CCR2 = (uint16_t)((1.0f - foc->Tb) * foc->pwm_period_f);
    foc->CCR3 = (uint16_t)((1.0f - foc->Tc) * foc->pwm_period_f);

    /* 安全钳位 */
    if (foc->CCR1 > foc->pwm_period) foc->CCR1 = foc->pwm_period;
    if (foc->CCR2 > foc->pwm_period) foc->CCR2 = foc->pwm_period;
    if (foc->CCR3 > foc->pwm_period) foc->CCR3 = foc->pwm_period;
}

/* ================================================================== */
/*              更新 TIM1 比较寄存器                                   */
/*              在更新中断中调用，靠预装载自动同步到下一周期           */
/* ================================================================== */
void FOC_UpdatePWM(FOC_Handle_t *foc)
{
    if (foc == NULL) return;

    TIM1->CCR1 = foc->CCR1;
    TIM1->CCR2 = foc->CCR2;
    TIM1->CCR3 = foc->CCR3;
}

/* ================================================================== */
/*      强拖迭代（在 TIM1 更新中断 HAL 回调中调用，20kHz）             */
/*      数据流: 角度推进 → 逆Park → SVPWM → CCR 写入                  */
/* ================================================================== */
void FOC_ForcedRotation_ISR(FOC_Handle_t *foc)
{
    if (foc == NULL) return;

    /* ---- 状态机 ---- */
    switch (foc->state)
    {
    case FOC_STATE_IDLE:
        /* 不输出，保持当前 CCR 不变 */
        return;

    case FOC_STATE_RAMPING:
    {
        /* 频率斜坡上升 */
        float dt     = (float)FOC_ISR_PERIOD_US * 1e-6f;   /* 50us = 0.00005s */
        float step   = foc->ramp_rate * dt;                 /* 每步增加的频率 */
        foc->current_freq += step;

        if (foc->current_freq >= foc->target_freq)
        {
            foc->current_freq = foc->target_freq;
            foc->state = FOC_STATE_RUNNING;
            g_foc_event_flag |= FOC_EVENT_RAMP_DONE;  // ← 仅设标志，不 printf
        }

        /* RAMPING 继续往下执行 */
        break;
    }

    case FOC_STATE_RUNNING:
        /* 稳定运行，继续往下执行 */
        break;

    case FOC_STATE_FAULT:
    default:
        return;
    }

    /* ---- 角度推进 ---- */
    foc->omega_elec = FOC_TWO_PI * foc->current_freq;
    {
        float dt = (float)FOC_ISR_PERIOD_US * 1e-6f;
        foc->theta_elec += foc->omega_elec * dt;
    }

    /* 角度归一化到 [0, 2π) */
    while (foc->theta_elec >= FOC_TWO_PI)
        foc->theta_elec -= FOC_TWO_PI;
    while (foc->theta_elec < 0.0f)
        foc->theta_elec += FOC_TWO_PI;

    /* ---- 电压指令: Vd=0, Vq=电压幅值 ---- */
    foc->Vd = 0.0f;
    foc->Vq = foc->voltage_setting;

    /* ---- 逆 Park 变换: dq → αβ ---- */
    FOC_InvPark(foc);

    /* ---- SVPWM 调制: αβ → 马鞍波占空比 → CCR ---- */
    FOC_SVPWM(foc);

    /* ---- 写入 TIM1 硬件 ---- */
    FOC_UpdatePWM(foc);

    /* ---- 计数器 ---- */
    foc->loop_count++;
}