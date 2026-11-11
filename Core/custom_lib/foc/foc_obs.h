/**
 * @file    foc_obs.h
 * @brief   无感反电势观测器 + 锁相环（PLL）—— 估算转子电角度和电角速度
 *
 * 原理（静止 αβ 坐标系，表贴式 PMSM）：
 *     Eα = Vα - R·Iα - L·dIα/dt
 *     Eβ = Vβ - R·Iβ - L·dIβ/dt
 *
 * 本机相电感只有 ~20µH，在 ≤100Hz 电频率下 L·di/dt 占比只有约 1.4%，
 * 直接忽略（省掉噪声最大的微分项）：
 *     Eα ≈ Vα - R·Iα
 *     Eβ ≈ Vβ - R·Iβ
 *
 * 再用低通滤掉开关噪声，最后用 PLL 从 (Eα,Eβ) 中提取角度：
 *     ε = -Eα·cosθ̂ - Eβ·sinθ̂ = |E|·sin(θ - θ̂)
 *     ω̂ = Kp·εn + ∫Ki·εn dt      （εn = ε/|E|，与转速无关）
 *     θ̂ = ∫ω̂ dt
 *
 * 为什么不用经典滑模观测器（SMO）：
 *     SMO 的离散稳定条件 Ts/L · k/φ < 2，而滑动条件要求 k > |E|max。
 *     本机 Ts/L = 1.25，两个条件在 k>1.4V 时无法同时满足，会抖振发散。
 */

#ifndef __FOC_OBS_H__
#define __FOC_OBS_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 反电势幅值低于此值时认为"转速太低，不可信"，PLL 停止修正、角度自由滑行 */
/* 反电势低于此值时认为"转速太低，观测器不可信"。
 * 实测：本电机 20Hz 电频率时观测器估出的反电势只有 ~0.10V
 * （原以为 0.69V）。若还按 0.30V 做门槛，locked 永远为 0。
 * 真正可靠的判据是"过零法测到的真实频率"，它不依赖任何模型参数，
 * 所以这里只留一个防噪声的下限。 */
#define FOC_OBS_EMF_MIN     0.05f

/* 复位计数器：放在 RAM 未用区域，软复位/看门狗复位后保留，硬上电清零。
 * 用来判断“板子是不是在反复复位”。 */
extern volatile uint32_t g_boot_count;

/** 初始化/复位观测器（角度、速度、锁定状态全部清零） */
void FOC_OBS_Init(void);

/** 复位观测器状态，并把估算角度预置为 theta0（用于 I/f → 闭环的平滑接管） */
void FOC_OBS_Reset(float theta0);

/** 设置电机相电阻 [Ω] 和相电感 [H]（电感当前只用于文档说明，未参与运算） */
void FOC_OBS_SetRL(float R, float L);

/** 设置 PLL 频率前馈 [rad/s]。
 *  I/f 运行期间必须把指令电角速度喂进来，否则 PLL 从 0 起步、
 *  带宽（~12Hz）远小于实际电频率（50Hz→314rad/s），永远捕获不到。 */
void FOC_OBS_SetOmegaFF(float w);

/**
 * @brief 观测器单步更新，在每个 PWM 周期（40kHz）调用一次
 * @param V_a 上一步实际施加的 α 轴电压 [V]
 * @param V_b 上一步实际施加的 β 轴电压 [V]
 * @param I_a 实测 α 轴电流 [A]
 * @param I_b 实测 β 轴电流 [A]
 */
void FOC_OBS_Update(float V_a, float V_b, float I_a, float I_b);

/** 估算电角度 [rad]，范围 [0, 2π) */
float FOC_OBS_GetTheta(void);

/** 估算电角速度 [rad/s] */
float FOC_OBS_GetOmega(void);

/** 估算反电势幅值 [V]，用来判断能否切换闭环 */
float FOC_OBS_GetEmf(void);

/** 反电势过零点测得的「真实」电频率 [Hz]。
 *  与任何电机模型无关：转子不转就是 0，真实转速多少就是多少。
 *  用来判定 I/f 开环是不是失步（指令 20Hz 但实际只有 3Hz 等）。 */
float FOC_OBS_GetEmfHz(void);

/** 观测器是否已锁定：1 = 反电势足够大且 PLL 相位误差收敛 */
uint8_t FOC_OBS_IsLocked(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_OBS_H__ */
