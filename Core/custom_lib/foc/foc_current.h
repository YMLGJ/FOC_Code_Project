/**
 * @file    foc_current.h
 * @brief   Phase current sensing + d/q current loop for the STM32G473 FOC board.
 *
 * Hardware mapping from FOC原理图 G4-V1 (verified from the schematic):
 *   AMP_IU (U2 output) -> PB0  -> ADC1_IN15
 *   AMP_IV (U4 output) -> PA5  -> ADC2_IN13
 *   AMP_IW (U5 output) -> PB12 -> ADC1_IN11
 *
 * The CC6920SO-20A outputs pass through a 1k/2k divider before reaching the MCU.
 */

#ifndef __FOC_CURRENT_H__
#define __FOC_CURRENT_H__

#include "foc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- ADC channel mapping ---- */
#define FOC_CS_ADC1_CH_IU       ADC_CHANNEL_15   /* PB0  */
#define FOC_CS_ADC1_CH_IW       ADC_CHANNEL_11   /* PB12 */
#define FOC_CS_ADC2_CH_IV       ADC_CHANNEL_13   /* PA5  */

#define FOC_CS_GPIO_IU_PORT     GPIOB
#define FOC_CS_GPIO_IU_PIN      GPIO_PIN_0
#define FOC_CS_GPIO_IV_PORT     GPIOA
#define FOC_CS_GPIO_IV_PIN      GPIO_PIN_5
#define FOC_CS_GPIO_IW_PORT     GPIOB
#define FOC_CS_GPIO_IW_PIN      GPIO_PIN_12

/* ---- Current sensor scaling ----
 * CC6920SO-20A：灵敏度 100 mV/A，再经 1k/2k 分压（×2/3）后进 ADC。
 *
 * FOC_CS_ADC_V_PER_A 会自动再乘一次分压系数，所以 FOC_CS_SENSOR_V_PER_A
 * 必须填"未分压"的原始灵敏度 0.1。
 * 旧值 0.066 其实是"已经分压后"的数，再乘 0.6667 变成 44mV/A，
 * 导致电流读数偏大 1.5 倍（实测：VD 0.05V 下读 1269mA，实际约 850mA）。
 */
#define FOC_CS_SENSOR_V_PER_A   0.1f
#define FOC_CS_DIVIDER          0.6666667f
#define FOC_CS_ADC_V_PER_A      (FOC_CS_SENSOR_V_PER_A * FOC_CS_DIVIDER)
#define FOC_ADC_VREF            3.3f
#define FOC_ADC_FULL_SCALE      4095.0f

/* Flip a sign if the measured current direction is opposite to the FOC
 * convention (positive = inverter -> motor).
 *
 * 实测定标（VD 0.3、theta=0、转子夹住）：
 *   命令 U 相最高 → 实测 Ia=-1.72 / Ib=+0.93 / Ic=+0.79
 *   大小关系 2:1:1 完全正确，只是符号全反 → 传感器方向装反。
 *   取反后：Ia=+1.72 / Ib=-0.93 / Ic=-0.79 ✓ 与期望一致。 */
#define FOC_CS_SIGN_IU          (-1.0f)
#define FOC_CS_SIGN_IV          (-1.0f)
#define FOC_CS_SIGN_IW          (-1.0f)   /* 未使用（第三相由 Ic=-(Ia+Ib) 推出） */

/* ---- Current loop defaults ----
 * Start with a small Iq and a high enough supply current limit.
 * The board current sensors are 20 A; keep the test limit well below that.
 */
#define FOC_CURRENT_LIMIT_A     5.0f   /* 相电阻只有 ~0.1Ω，小电流下死区完全压制控制权，
                                        * 测试工作点必须提到 3~5A 才有闭环能力 */
#define FOC_CURRENT_RAMP_A_S    2.0f
#define FOC_CURRENT_KP          0.05f  /* 从 0.1 降到 0.05，先求稳再求快 */
#define FOC_CURRENT_KI          50.0f  /* 从 100 降到 50，穿越频率降到 ~80Hz */

/* ---- 电流采样抗干扰 ----
 * ADC 由 TIM1 更新事件同步触发（每个 PWM 周期一次），采样点落在零矢量的
 * 安静时刻；再用一阶 IIR 低通抑制残余纹波。
 * alpha 越小越平滑、相位滞后越大：20kHz 采样下 alpha=0.25 等效截止≈800Hz。
 */
#define FOC_CS_FILTER_ALPHA     0.25f
/* 单相电流硬限幅：CC6920SO-20A 量程就是 ±20A，超出后传感器已经饱和，
 * 继续往里塞只会把 PI 踢飞，所以直接按量程钳位。 */
#define FOC_CS_CLAMP_A          20.0f
/* 零漂校准采样点数：20kHz 下 400 点 ≈ 20ms */
#define FOC_CS_OFFSET_SAMPLES   400U

/* 零漂合理范围：正常 ≈2068（VCC/2 经 1k/2k 分压）。超出这个范围说明
 * 传感器没供电 / AMP 引脚没接 / 分压电阻缺失，此时禁止使能电流环。 */
#define FOC_CS_OFFSET_MIN       1000U
#define FOC_CS_OFFSET_MAX       3000U

/** Initialize ADC1/ADC2 + DMA and calibrate the zero-current offsets.
 *  @return 0 on success, non-zero on failure. */
uint8_t FOC_Current_Init(void);

/** Read the three phase currents in amperes. */
void FOC_Current_Read(float *ia, float *ib, float *ic);

/** Diagnostic: raw ADC value. idx: 0=IU, 1=IV, 2=IW.
 *  Normal value at zero current is about VREF/2 (≈2068). */
uint16_t FOC_Current_GetRaw(uint8_t idx);

/** Diagnostic: zero-current offset captured at boot. idx: 0=IU, 1=IV, 2=IW. */
uint16_t FOC_Current_GetOffset(uint8_t idx);

/** 零漂校准步进：必须在 FOC_CS_OFFSET_SAMPLES 次 PWM 中断内被调用，
 *  建议放在 TIM1 更新中断回调的第一行。累计满后自动完成校准。 */
void FOC_Current_CalibStep(void);

/** 零漂校准是否结束：1 = 已结束（成功或失败）。 */
uint8_t FOC_Current_CalibDone(void);

/** 零漂校准是否成功：1 = 偏移合理，可以启用电流环。 */
uint8_t FOC_Current_CalibOk(void);

/** 重新开始零漂校准（DMA 已在跑，只是重新累计样本）。
 *  用于传感器比 MCU 晚上电（先插 USB 再合 24V）的场景，失败后重试。 */
void FOC_Current_RestartCalib(void);

/** 保护触发原因：0=无  1=过流(OCP)  2=ADC反馈冻结。电机突然停转时看它。 */
extern volatile uint8_t g_foc_trip_code;

/** 只测量不控制：读 ADC → Clarke → Park，把 Ia/Ib/Ic/Id/Iq 写回句柄。
 *  开环电压测试（VD 命令）时用，不跑 PI。 */
void FOC_CurrentSample_ISR(FOC_Handle_t *foc);

/** Run one d/q current-loop iteration. Call from the 20 kHz TIM1 ISR. */
void FOC_CurrentLoop_ISR(FOC_Handle_t *foc);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_CURRENT_H__ */
