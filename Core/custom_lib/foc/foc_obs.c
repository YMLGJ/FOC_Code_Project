/**
 * @file    foc_obs.c
 * @brief   无感反电势观测器 + 锁相环（PLL）
 */

#include "foc_obs.h"
#include "arm_math.h"

#define OBS_TWO_PI          6.283185307f

/* 中断周期：与 foc.h 的 FOC_ISR_PERIOD_US 保持一致（20kHz -> 50us） */
#define OBS_TS              50.0e-6f

/* 电角速度上下限：5000 rad/s ≈ 800Hz 电频率，够用了 */
#define OBS_OMEGA_MAX       5000.0f

/* PLL 参数：自然频率 ~12Hz、阻尼 0.6
 *   Kp = 2·ζ·ωn = 80
 *   Ki = ωn²     = 6400
 * 之前 450/98000（ωn≈313 rad/s）在 20kHz 下完全失控：
 * 归一化误差 ±1 时 ω̂ 直接跳到 ±450 rad/s，而实际只有 ~29 rad/s。
 * 这也说明：归一化误差的 PLL 增益必须按“能表达的最大转速”来定，
 * 不能按理论带宽算。 */
#define OBS_PLL_KP          80.0f
#define OBS_PLL_KI          6400.0f

/* 反电势低通系数：20kHz 下 alpha=0.15 → 截止 ≈550Hz，
 * 对 50Hz 反电势的相位滞后只有 5°，可以不做相位补偿 */
#define OBS_EMF_LPF         0.15f

/* 锁定判定 */
#define OBS_LOCK_SIN_MAX    0.25f     /* |sin(θ-θ̂)| 小于此值算相位对齐 */
#define OBS_LOCK_CYCLES     2000U     /* 连续 2000 个周期(50ms)才认为锁定 */

/* ------------------------------------------------------------------ */
/* 相电阻：万用表实测 U-V 线间 0.12Ω → 相电阻 0.06Ω。
 * （早期用 VD 0.05/Id 反推得 0.0394Ω，那是因为当时电流标度偏大 1.52 倍；
 *   修正 FOC_CS_SENSOR_V_PER_A=0.1 之后，0.05V / 0.833A = 0.060Ω，与万用表一致。） */
#define OBS_R_DEFAULT       0.06f
#define OBS_L_DEFAULT       20.0e-6f

static float    s_R      = OBS_R_DEFAULT;   /* 相电阻 [Ω] */
static float    s_L      = OBS_L_DEFAULT;   /* 相电感 [H]（预留） */

static float    s_Ea     = 0.0f;
static float    s_Eb     = 0.0f;
static float    s_theta  = 0.0f;
static float    s_omega  = 0.0f;
static float    s_intg   = 0.0f;
static float    s_emf    = 0.0f;

/* 频率前馈 [rad/s]：由 I/f 的指令电角速度提供。
 * PLL 带宽只有 ~12Hz，而 50Hz 电频率 = 314 rad/s —— 从 ω̂=0 起步根本捕获不到，
 * 相位误差一直饱和在 ±1、积分正负抵消，ω̂ 永远爬不起来（实测就是 theta/omega 卡在 0）。
 * 加上前馈后，PLL 只在指令频率附近做微调，捕获范围变成“带宽”而不是“绝对频率”。 */
static float    s_omega_ff = 0.0f;

static uint16_t s_ok_cnt  = 0U;
static uint16_t s_bad_cnt = 0U;
static uint8_t  s_locked  = 0U;

/* ===== 反电势过零法测「真实」电频率 =====
 * 完全不依赖模型：只看 Eα 在静止坐标系里过零多少次。
 * 转子转一圈（电角度 360°）Eα 过两次零，所以 f = 过零数 / 2 / 时间。
 * 好处：就算 R 估错、就算 V 算错，只要转子真在转，这个数就是真的。 */
static uint32_t s_zc_cnt   = 0U;
static uint32_t s_zc_ticks = 0U;
static uint8_t  s_zc_last  = 0U;      /* 0 = 未建立, 1 = Eα>0, 2 = Eα<0 */
static float    s_emf_hz   = 0.0f;

#define OBS_ZC_WIN_TICKS  10000U      /* 500ms @ 20kHz -> 1Hz 分辨率 */
#define OBS_ZC_MIN_EMF    0.020f     /* 幅值小于 20mV 不计数（纯噪声） */

/* ------------------------------------------------------------------ */
void FOC_OBS_Init(void)
{
    FOC_OBS_Reset(0.0f);
    s_R = OBS_R_DEFAULT;
    s_L = OBS_L_DEFAULT;
}

void FOC_OBS_Reset(float theta0)
{
    s_Ea    = 0.0f;
    s_Eb    = 0.0f;
    s_theta = theta0;
    s_omega = 0.0f;
    s_intg  = 0.0f;
    s_emf   = 0.0f;
    s_ok_cnt  = 0U;
    s_bad_cnt = 0U;
    s_locked  = 0U;

    s_zc_cnt   = 0U;
    s_zc_ticks = 0U;
    s_zc_last  = 0U;
    s_emf_hz   = 0.0f;
}

void FOC_OBS_SetOmegaFF(float w)
{
    if (w >  OBS_OMEGA_MAX) w =  OBS_OMEGA_MAX;
    if (w < -OBS_OMEGA_MAX) w = -OBS_OMEGA_MAX;
    s_omega_ff = w;
}

void FOC_OBS_SetRL(float R, float L)
{
    if (R > 0.001f) { s_R = R; }
    if (L > 1.0e-6f) { s_L = L; }
}

/* ------------------------------------------------------------------ */
void FOC_OBS_Update(float V_a, float V_b, float I_a, float I_b)
{
    /* 1) 反电势（电压模型，忽略 L·di/dt） */
    const float Ea_raw = V_a - s_R * I_a;
    const float Eb_raw = V_b - s_R * I_b;

    /* 2) 低通，滤掉 PWM 开关噪声 */
    s_Ea += OBS_EMF_LPF * (Ea_raw - s_Ea);
    s_Eb += OBS_EMF_LPF * (Eb_raw - s_Eb);

    /* 3) 反电势幅值 */
    float emf = 0.0f;
    arm_sqrt_f32(s_Ea * s_Ea + s_Eb * s_Eb, &emf);
    s_emf = emf;

    /* 4) PLL 相位误差：ε = |E|·sin(θ - θ̂) */
    const float c  = arm_cos_f32(s_theta);
    const float sn = arm_sin_f32(s_theta);
    const float eps = -s_Ea * c - s_Eb * sn;

    /* 归一化后与转速无关，PLL 增益不需要随速度调整 */
    float eps_n = 0.0f;
    if (emf > FOC_OBS_EMF_MIN)
    {
        eps_n = eps / emf;
        if (eps_n >  1.0f) { eps_n =  1.0f; }
        if (eps_n < -1.0f) { eps_n = -1.0f; }
    }
    /* 反电势太小（转速太低）时 eps_n = 0，PLL 保持当前速度自由滑行 */

    s_intg += OBS_PLL_KI * eps_n * OBS_TS;
    if (s_intg >  600.0f) { s_intg =  600.0f; }
    if (s_intg < -600.0f) { s_intg = -600.0f; }

    /* ω̂ = 前馈 + 比例修正 + 积分修正 */
    s_omega = s_omega_ff + OBS_PLL_KP * eps_n + s_intg;
    if (s_omega >  OBS_OMEGA_MAX) { s_omega =  OBS_OMEGA_MAX; }
    if (s_omega < -OBS_OMEGA_MAX) { s_omega = -OBS_OMEGA_MAX; }

    s_theta += s_omega * OBS_TS;
    while (s_theta >= OBS_TWO_PI) { s_theta -= OBS_TWO_PI; }
    while (s_theta <  0.0f)       { s_theta += OBS_TWO_PI; }

    /* 4.5) 过零法测真实电频率（与模型无关，转子不转就是 0） */
    if ((emf > OBS_ZC_MIN_EMF) || (s_Eb > OBS_ZC_MIN_EMF) || (s_Eb < -OBS_ZC_MIN_EMF))
    {
        const uint8_t sign = (s_Ea > 0.0f) ? 1U : 2U;
        if ((s_zc_last != 0U) && (sign != s_zc_last))
        {
            s_zc_cnt++;
        }
        s_zc_last = sign;
    }
    else
    {
        s_zc_last = 0U;
    }

    if (++s_zc_ticks >= OBS_ZC_WIN_TICKS)
    {
        /* 500ms 窗口内的过零数 / 2 / 0.5s */
        s_emf_hz   = ((float)s_zc_cnt * 0.5f) / 0.5f;
        s_zc_cnt   = 0U;
        s_zc_ticks = 0U;
    }

    /* 5) 锁定判断
     * 关键修正：不能只看反电势「幅值」！
     * 反电势幅值 = |V - R·I|，当电流很大（或 R 估错）时，-R·I 这个直流分量
     * 自己就能超过阈值，此时转子根本没转，却会被误判成"已锁定"。
     * 实测踩坑：Id=18A 时 |V-R·I| = 1.1V，locked=1、自动切无感，直接炸电流。
     * 所以必须同时要求「过零法测到的真实电频率 > 2Hz」。 */
    const uint8_t emf_ok = ((emf > FOC_OBS_EMF_MIN) && (s_emf_hz > 5.0f)) ? 1U : 0U;
    const uint8_t phase_ok = ((eps_n < OBS_LOCK_SIN_MAX) &&
                              (eps_n > -OBS_LOCK_SIN_MAX)) ? 1U : 0U;

    if ((emf_ok != 0U) && (phase_ok != 0U))
    {
        if (s_ok_cnt < 60000U) { s_ok_cnt++; }
        s_bad_cnt = 0U;
    }
    else
    {
        if (s_bad_cnt < 60000U) { s_bad_cnt++; }
        s_ok_cnt = 0U;
    }

    s_locked = (s_ok_cnt >= OBS_LOCK_CYCLES) ? 1U : 0U;
}

/* ------------------------------------------------------------------ */
float FOC_OBS_GetTheta(void)  { return s_theta; }
float FOC_OBS_GetOmega(void)  { return s_omega; }
float FOC_OBS_GetEmf(void)    { return s_emf;   }
float FOC_OBS_GetEmfHz(void)  { return s_emf_hz; }
uint8_t FOC_OBS_IsLocked(void){ return s_locked; }
