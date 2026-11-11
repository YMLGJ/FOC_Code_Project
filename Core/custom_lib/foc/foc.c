/**
 * @file    foc.c
 * @brief   FOC 核心算法 — 强拖模式（逆 Park → SVPWM 马鞍波）
 */

#include "foc.h"
#include "foc_current.h"
#include "foc_obs.h"
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
    foc->pwm_period   = FOC_PWM_PERIOD;         /* 2124 -> 40kHz */
    foc->pwm_period_f = (float)FOC_PWM_PERIOD;  /* 2124.0f       */
    foc->Vbus         = FOC_VBUS_DEFAULT;// 24.0f
    foc->pole_pairs   = FOC_POLE_PAIRS;// 7

    /* 默认强拖参数 */
    foc->state            = FOC_STATE_IDLE;
    foc->target_freq      = 0.0f;       // 目标频率
    foc->current_freq     = 0.0f;       // 当前频率
    foc->ramp_rate        = 20.0f;      // 斜坡速率 Hz/s  
    foc->voltage_setting  = 2.0f;       // 电压幅值 V
    foc->voltage_min      = 0.1f;       // 电流环/预对齐时使用
    foc->voltage_max      = foc->Vbus * 0.8f;  /* 限制 80% Vbus */

    /* ---- 电流环默认参数 ---- */
    foc->Ia = foc->Ib = foc->Ic = 0.0f;
    foc->I_alpha = foc->I_beta = 0.0f;
    foc->Id = foc->Iq = 0.0f;
    foc->Id_ref = 0.0f;
    foc->Iq_ref = 0.0f;
    foc->Iq_ref_target = 0.0f;
    foc->Kp_i = FOC_CURRENT_KP;
    foc->Ki_i = FOC_CURRENT_KI;
    foc->Vd_int = 0.0f;
    foc->Vq_int = 0.0f;
    foc->current_loop_en = 0U;
    foc->dtc_en          = 1U;      /* 死区补偿默认开 */
    foc->voltage_test_en = 0U;
    foc->dc_test_en      = 0U;

    /* ---- 无感观测默认值 ---- */
    foc->theta_if        = 0.0f;
    foc->sensorless_en   = 0U;
    foc->sensorless_auto = 1U;
    foc->obs_switch_hz   = FOC_OBS_SWITCH_HZ_DEFAULT;

    /* 上电先 50%：三相平均 Vbus/2，给自举电容充电 */
    foc->Ta = foc->Tb = foc->Tc = 0.5f;
    foc->CCR1 = foc->CCR2 = foc->CCR3 = (uint16_t)(foc->pwm_period_f * 0.5f);
    FOC_UpdatePWM(foc);

    /* 保存全局指针 */
    p_foc = foc;

    printf("FOC_Init: OK, PWM=%uHz, PolePairs=%u\r\n",
           (unsigned)FOC_PWM_FREQ_HZ, (unsigned)foc->pole_pairs);
}

/* ================================================================== */
/*                    启动强拖                                        */
/* ================================================================== */
void FOC_StartForcedRotation(FOC_Handle_t *foc,
                             float target_freq_hz,
                             float voltage_v)   
{
    if (foc == NULL) return;        /* 空指针检查 */

    /* 参数合理性检查 */
    if (target_freq_hz < 0.5f)  target_freq_hz = 0.5f;      // 最低 0.5Hz
    if (target_freq_hz > 200.0f) target_freq_hz = 200.0f;       // 最高 200Hz

    if (voltage_v < foc->voltage_min) voltage_v = foc->voltage_min;     // 最低 0.5V
    if (voltage_v > foc->voltage_max) voltage_v = foc->voltage_max;     // 最高 80% Vbus

    foc->target_freq     = target_freq_hz;      // 目标频率
    foc->voltage_setting = voltage_v;       // 电压幅值
    foc->current_freq    = 0.0f;        // 当前频率
    foc->theta_elec      = 0.0f;       // 电角度
    foc->loop_count      = 0;          // 计数器清零

    foc->state = FOC_STATE_RAMPING;     // 进入斜坡上升状态

    printf("FOC_Start: target=%dHz, voltage=%dmV\r\n",
           (int)target_freq_hz, (int)(voltage_v * 1000.0f));// 打印启动信息
}

/* ================================================================== */
/*             启动电流环模式（I/f + d/q 电流环）                      */
/* ================================================================== */
void FOC_StartCurrentMode(FOC_Handle_t *foc,
                          float target_freq_hz,
                          float iq_ref_a)
{
    if (foc == NULL) return;

    if (target_freq_hz < 0.5f)  target_freq_hz = 0.5f;
    if (target_freq_hz > 200.0f) target_freq_hz = 200.0f;

    if (iq_ref_a >  FOC_CURRENT_LIMIT_A) iq_ref_a =  FOC_CURRENT_LIMIT_A;
    if (iq_ref_a < -FOC_CURRENT_LIMIT_A) iq_ref_a = -FOC_CURRENT_LIMIT_A;

    foc->target_freq   = target_freq_hz;
    foc->Iq_ref_target = iq_ref_a;
    foc->Iq_ref        = 0.0f;
    foc->current_freq  = 0.0f;
    foc->theta_elec    = 0.0f;
    foc->theta_if      = 0.0f;
    foc->sensorless_en = 0U;
    foc->Vd_int        = 0.0f;
    foc->Vq_int        = 0.0f;
    foc->state         = FOC_STATE_RAMPING;
    foc->current_loop_en = 1U;

    printf("FOC_CurrentMode: %dHz %dmA\r\n",
           (int)target_freq_hz, (int)(iq_ref_a * 1000.0f));
}

/* ================================================================== */
/*                    停止电机                                        */
/* ================================================================== */
void FOC_Stop(FOC_Handle_t *foc)
{
    if (foc == NULL) return;

    foc->state        = FOC_STATE_IDLE;     // 停止状态
    foc->target_freq  = 0.0f;       // 目标频率
    foc->current_freq = 0.0f;
    foc->theta_elec   = 0.0f;
    foc->Vd           = 0.0f;
    foc->Vq           = 0.0f;
    foc->V_alpha      = 0.0f;
    foc->V_beta       = 0.0f;

    /* 电流环复位，避免下次启动时积分/参考残留 */
    foc->Id_ref       = 0.0f;
    foc->Iq_ref       = 0.0f;
    foc->Iq_ref_target = 0.0f;
    foc->Vd_int       = 0.0f;
    foc->Vq_int       = 0.0f;
    foc->voltage_test_en = 0U;
    foc->Vd_test      = 0.0f;
    foc->dc_test_en   = 0U;
    foc->sensorless_en = 0U;
    FOC_OBS_Init();

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
    float cos_theta = arm_cos_f32(foc->theta_elec);      // 计算电角度的余弦
    float sin_theta = arm_sin_f32(foc->theta_elec);     // 计算电角度的正弦

    foc->V_alpha = foc->Vd * cos_theta - foc->Vq * sin_theta;       // 计算 α 轴电压
    foc->V_beta  = foc->Vd * sin_theta + foc->Vq * cos_theta;       // 计算 β 轴电压
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
/*          死区补偿系数：电流方向，过零点线性过渡                     */
/*          返回 -1..+1；|i| >= th 时饱和到 ±1                        */
/* ================================================================== */
static float FOC_CompFactor(float i, float th)
{
    if (th <= 0.0f) return 0.0f;
    if (i >  th) return  1.0f;
    if (i < -th) return -1.0f;
    return i / th;
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

    /* 1.5 保存零序注入前的三相正弦电压（供 VOFA 调试对比） */
    foc->Va_sin = Va;            /* 在注入零序之前保存 */
    foc->Vb_sin = Vb;
    foc->Vc_sin = Vc;

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

    /* 5. 死区补偿 -------------------------------------------------------
     * 电流为正（逆器→电机）时，死区期间由下管二极管续流，输出电压偏低，
     * 需要把占空比补回来；电流为负时相反。量级 = t_dead / T。
     * 过零点用线性过渡（而不是硬切符号），否则小电流时补偿会自己抖起来。 */
    if (foc->dtc_en)
    {
        const float dtc = (float)FOC_DEADTIME_COUNTS /
                          (2.0f * foc->pwm_period_f);
        const float th  = FOC_DEADTIME_COMP_DB;

        foc->Ta += dtc * FOC_CompFactor(foc->Ia, th);
        foc->Tb += dtc * FOC_CompFactor(foc->Ib, th);
        foc->Tc += dtc * FOC_CompFactor(foc->Ic, th);
    }

    /* 补偿后重新限幅，保证 duty ∈ [0,1] */
    if (foc->Ta > 1.0f) foc->Ta = 1.0f;
    if (foc->Ta < 0.0f) foc->Ta = 0.0f;
    if (foc->Tb > 1.0f) foc->Tb = 1.0f;
    if (foc->Tb < 0.0f) foc->Tb = 0.0f;
    if (foc->Tc > 1.0f) foc->Tc = 1.0f;
    if (foc->Tc < 0.0f) foc->Tc = 0.0f;

    /* 6. 占空比 → TIM1 CCR 寄存器值
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

    /* ---- 硬件通道映射（实测结果，万用表线电压验证） --------------------
     *   TIM1_CH1 -> 输出 W 相
     *   TIM1_CH2 -> 输出 V 相
     *   TIM1_CH3 -> 输出 U 相
     *
     * 而电流传感器是 IU->U、IV->V，所以驱动的 A/B/C 和采样的 U/V/W
     * 必须对齐，否则 Id/Iq 是假的 → PI 拿假反馈必然失控。
     * 做法：A 相（Ta）写到 CH3，C 相（Tc）写到 CH1，B 相（Tb）保持 CH2。
     * 验证方法：VD 0.5 时 U 相应为最高电压，此时 Ia 应为正、Ib/Ic 为负。
     * ------------------------------------------------------------------ */
    TIM1->CCR1 = foc->CCR3;     /* CH1 -> W 相，装 C 相占空比 */
    TIM1->CCR2 = foc->CCR2;     /* CH2 -> V 相，装 B 相占空比 */
    TIM1->CCR3 = foc->CCR1;     /* CH3 -> U 相，装 A 相占空比 */
}

/* ================================================================== */
/*      强拖迭代（在 TIM1 更新中断 HAL 回调中调用，20kHz）             */
/*      数据流: 角度推进 → 逆Park → SVPWM → CCR 写入                  */
/* ================================================================== */
void FOC_ForcedRotation_ISR(FOC_Handle_t *foc)
{
    if (foc == NULL) return;

    /* ---- 静态直流矢量测试：三相占空比固定，不经过 SVPWM、不开关 ----
     * 没有死区、没有纹波、没有采样时序问题，是判断相序/极性的唯一无歧义手段。
     * 3 秒自动退出，避免长时间通直流发热。 */
    if (foc->dc_test_en)
    {
        FOC_CurrentSample_ISR(foc);   /* 只测量不控制 */

        /* 硬件级保护：DC 测试把直流直接加到绕组（相电阻只有 ~0.1Ω），
           电源限流不可靠，必须靠这里兑底。超过 1.5A 立即停机。 */
        if ((foc->Ia > FOC_DC_ABORT_A) || (foc->Ia < -FOC_DC_ABORT_A) ||
            (foc->Ib > FOC_DC_ABORT_A) || (foc->Ib < -FOC_DC_ABORT_A) ||
            (foc->Ic > FOC_DC_ABORT_A) || (foc->Ic < -FOC_DC_ABORT_A))
        {
            foc->dc_test_en = 0U;
            FOC_Stop(foc);
            return;
        }

        foc->Ta = foc->Ta_dc;
        foc->Tb = foc->Tb_dc;
        foc->Tc = foc->Tc_dc;

        foc->CCR1 = (uint16_t)((1.0f - foc->Ta) * foc->pwm_period_f);
        foc->CCR2 = (uint16_t)((1.0f - foc->Tb) * foc->pwm_period_f);
        foc->CCR3 = (uint16_t)((1.0f - foc->Tc) * foc->pwm_period_f);
        FOC_UpdatePWM(foc);

        foc->dc_ticks++;
        if (foc->dc_ticks > (3U * FOC_PWM_FREQ_HZ))
        {
            /* 超时必须把占空比恢复成 50%，否则 CCR 停在直流矢量上会持续通电 */
            foc->dc_test_en = 0U;
            FOC_Stop(foc);
        }
        return;
    }

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

    /* ---- 角度推进（I/f 开环积分） ---- */
    foc->omega_elec = FOC_TWO_PI * foc->current_freq;
    {
        float dt = (float)FOC_ISR_PERIOD_US * 1e-6f;
        foc->theta_if += foc->omega_elec * dt;
    }

    /* 归一化到 [0, 2π) */
    while (foc->theta_if >= FOC_TWO_PI)
        foc->theta_if -= FOC_TWO_PI;
    while (foc->theta_if < 0.0f)
        foc->theta_if += FOC_TWO_PI;

    /* ---- 选择控制用的电角度 ---- */
    if (foc->sensorless_en)
    {
        foc->theta_elec = FOC_OBS_GetTheta();
        foc->theta_if   = foc->theta_elec;   /* 保持开环参考同步，方便回退 */
    }
    else
    {
        foc->theta_elec = foc->theta_if;
    }

    /* ---- 电流环或开环电压指令 ---- */
    if (foc->current_loop_en)
    {
        FOC_CurrentLoop_ISR(foc);   /* 读 ADC → Clarke/Park → PI → Vd/Vq */
    }
    else if (foc->voltage_test_en)
    {
        /* 开环纯 d 轴电压：theta 固定为 0，Va=Vd、Vb=Vc=-Vd/2。
           用来直接观察三相电流关系，绕开 PI 验证传感器相序。 */
        FOC_CurrentSample_ISR(foc);   /* 只测量不控制，保证 Ia/Ib/Ic 可见 */
        foc->Vd = foc->Vd_test;
        foc->Vq = foc->Vq_test;
    }
    else
    {
        foc->Vd = 0.0f;
        foc->Vq = foc->voltage_setting;
    }

    /* ---- 逆 Park 变换: dq → αβ ---- */
    FOC_InvPark(foc);

    /* ---- 无感观测器：用本拍施加的电压和实测电流更新角度估计 ----
     * 频率前馈：I/f 期间把指令电角速度喂给 PLL，
     * 否则 PLL 从 ω̂=0 起步、带宽远小于实际频率，永远锁不上。
     * 切换成无感后前馈保持不变，由 PLL 积分器自己修正残余误差。 */
    FOC_OBS_SetOmegaFF(foc->omega_elec);
    FOC_OBS_Update(foc->V_alpha, foc->V_beta, foc->I_alpha, foc->I_beta);

    /* ---- I/f → 闭环的自动切换 -------------------------------------
     * 切换条件（三个都满足）：
     *   1. 指令电频率 > obs_switch_hz（反电势够大，观测器才可信）
     *   2. 观测器已锁定（反电势幅值够 + PLL 相位收敛）
     *   3. 观测角与 I/f 角相差 < 35°（防止锁定到 180° 反相点）
     * 切换后如果连续 100ms 失锁，自动退回 I/f。
     * -------------------------------------------------------------- */
    if ((foc->sensorless_auto != 0U) && (foc->obs_switch_hz > 0.0f))
    {
        static uint16_t lost_cnt = 0U;

        if (foc->sensorless_en == 0U)
        {
            float dth = FOC_OBS_GetTheta() - foc->theta_if;
            while (dth >  FOC_PI) { dth -= FOC_TWO_PI; }
            while (dth < -FOC_PI) { dth += FOC_TWO_PI; }

            if ((foc->current_freq >= foc->obs_switch_hz) &&
                (FOC_OBS_IsLocked() != 0U) &&
                (dth < 0.9f) && (dth > -0.9f) &&
                /* 双重保险：过零法测到的真实电频率必须和指令频率接近。
                 * 否则说明转子在打滑，根本没跟上 I/f，绝不能切闭环。 */
                (FOC_OBS_GetEmfHz() > (0.7f * foc->current_freq)) &&
                (FOC_OBS_GetEmfHz() < (1.3f * foc->current_freq)))
            {
                foc->sensorless_en = 1U;
                lost_cnt = 0U;
            }
        }
        else
        {
            if (FOC_OBS_IsLocked() == 0U)
            {
                if (lost_cnt < 0xFFFFU) { lost_cnt++; }
                if (lost_cnt > 4000U)          /* 100ms 失锁 -> 退回 I/f */
                {
                    foc->sensorless_en = 0U;
                    foc->theta_if = FOC_OBS_GetTheta();
                    lost_cnt = 0U;
                }
            }
            else
            {
                lost_cnt = 0U;
            }
        }
    }

    /* ---- SVPWM 调制: αβ → 马鞍波占空比 → CCR ---- */
    FOC_SVPWM(foc);

    /* ---- 写入 TIM1 硬件 ---- */
    FOC_UpdatePWM(foc);

    /* ---- 计数器 ---- */
    foc->loop_count++;
}