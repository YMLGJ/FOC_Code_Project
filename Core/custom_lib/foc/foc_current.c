/**
 * @file    foc_current.c
 * @brief   Three-phase current sensing and d/q current loop.
 *
 * ADC mapping (from the schematic):
 *   IU -> PB0  -> ADC1_IN15
 *   IV -> PA5  -> ADC2_IN13
 *   IW -> PB12 -> ADC1_IN11
 *
 * ADC1 scans IU/IW with DMA, ADC2 samples IV with DMA. Both run continuously;
 * the 20 kHz FOC ISR reads the latest results.
 */

#include "foc_current.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_adc_ex.h"
#include "arm_math.h"
#include <string.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/* ADC / DMA handles and buffers                                       */
/* ------------------------------------------------------------------ */
static ADC_HandleTypeDef hadc1;
static ADC_HandleTypeDef hadc2;
static DMA_HandleTypeDef hdma_adc1;
static DMA_HandleTypeDef hdma_adc2;

/* 只采两相：IU 走 ADC1、IV 走 ADC2，两路由同一个 TIM1_TRGO 触发 → 严格同时采样。
 * 第三相 IW 不再单独转换：ADC1 扫两路时第二路会比第一路晚 1.42us，
 * 在 2.4A/us 的 di/dt 下就是 3.4A 的误差，三相之和不为零，Clarke/Park 全错。
 * 星形连接必然有 Ic = -(Ia + Ib)，直接算出来即可。 */
static volatile uint16_t adc1_raw[1] = {0U};
/* ADC2: rank1 = IV */
static volatile uint16_t adc2_raw[1] = {0U};

static uint16_t off_iu = 0U;
static uint16_t off_iv = 0U;
static uint16_t off_iw = 0U;
static uint8_t  current_adc_ok = 0U;

/* 保护触发原因：0=无  1=过流(OCP)  2=ADC反馈冻结
 * 电机突然停转时看这个变量就知道是谁把闭环关掉了。 */
volatile uint8_t g_foc_trip_code = 0U;

/* 三点中值：去掉开关噪声造成的单点尖峰，几乎不引入相位滞后 */
static uint16_t FOC_Median3(uint16_t a, uint16_t b, uint16_t c)
{
    if (a < b)
    {
        if (b < c) { return b; }
        return (a < c) ? c : a;
    }
    else
    {
        if (a < c) { return a; }
        return (b < c) ? c : b;
    }
}

/* ------------------------------------------------------------------ */
/* GPIO / DMA / ADC init                                               */
/* ------------------------------------------------------------------ */
static void FOC_Current_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;

    gpio.Pin = FOC_CS_GPIO_IV_PIN;
    HAL_GPIO_Init(FOC_CS_GPIO_IV_PORT, &gpio);

    gpio.Pin = FOC_CS_GPIO_IU_PIN | FOC_CS_GPIO_IW_PIN;
    HAL_GPIO_Init(FOC_CS_GPIO_IU_PORT, &gpio);
}

static uint8_t FOC_Current_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    /* G4 的 DMA 请求必须经 DMAMUX1 路由，缺这个时钟时 DMA 永远收不到请求，
       而且 HAL_DMA_Init / HAL_ADC_Start_DMA 都不会报错。 */
    __HAL_RCC_DMAMUX1_CLK_ENABLE();

    /* ADC1 -> DMA1 Channel 1 */
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
        return 1U;      /* DMA1 Channel1 初始化失败 */
    }
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    /* ADC2 -> DMA1 Channel 2 */
    hdma_adc2.Instance = DMA1_Channel2;
    hdma_adc2.Init.Request = DMA_REQUEST_ADC2;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode = DMA_CIRCULAR;
    hdma_adc2.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK)
    {
        return 2U;      /* DMA1 Channel2 初始化失败 */
    }
    __HAL_LINKDMA(&hadc2, DMA_Handle, hdma_adc2);

    return 0U;
}

static uint8_t FOC_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef ch = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.GainCompensation = 0U;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;  /* 只转 1 路，才能和 ADC2 严格同步 */
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;    /* 由 TIM1_TRGO 触发，每 PWM 周期一次 */
    hadc1.Init.NbrOfConversion = 1U;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.NbrOfDiscConversion = 1U;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.OversamplingMode = DISABLE;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        return 1U;
    }

    ch.Channel = FOC_CS_ADC1_CH_IU;
    ch.Rank = ADC_REGULAR_RANK_1;
    ch.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;  /* 加长采样窗口，配合 100nF 抑制开关尖峰 */
    ch.SingleDiff = ADC_SINGLE_ENDED;
    ch.OffsetNumber = ADC_OFFSET_NONE;
    ch.Offset = 0U;
    if (HAL_ADC_ConfigChannel(&hadc1, &ch) != HAL_OK)
    {
        return 2U;
    }

    /* 不再配置 IW：第三相由 Ic = -(Ia+Ib) 推出 */

    return 0U;
}

static uint8_t FOC_ADC2_Init(void)
{
    ADC_ChannelConfTypeDef ch = {0};

    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc2.Init.Resolution = ADC_RESOLUTION_12B;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.GainCompensation = 0U;
    hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc2.Init.LowPowerAutoWait = DISABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;    /* 由 TIM1_TRGO 触发，与 ADC1 同步 */
    hadc2.Init.NbrOfConversion = 1U;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.NbrOfDiscConversion = 1U;
    hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc2.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
    hadc2.Init.DMAContinuousRequests = ENABLE;
    hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc2.Init.OversamplingMode = DISABLE;

    if (HAL_ADC_Init(&hadc2) != HAL_OK)
    {
        return 1U;
    }

    ch.Channel = FOC_CS_ADC2_CH_IV;
    ch.Rank = ADC_REGULAR_RANK_1;
    ch.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;  /* 与 ADC1 保持一致 */
    ch.SingleDiff = ADC_SINGLE_ENDED;
    ch.OffsetNumber = ADC_OFFSET_NONE;
    ch.Offset = 0U;
    if (HAL_ADC_ConfigChannel(&hadc2, &ch) != HAL_OK)
    {
        return 2U;
    }

    return 0U;
}

/* ------------------------------------------------------------------ */
/* 零漂校准（非阻塞：由 20kHz 中断累计样本，不阻塞启动流程）            */
/*   cal_state: 0=未开始  1=采集中  2=完成                              */
/* ------------------------------------------------------------------ */
static volatile uint8_t  cal_state = 0U;
static uint32_t          cal_sum_u = 0U;
static uint32_t          cal_sum_v = 0U;
static uint32_t          cal_cnt   = 0U;

void FOC_Current_CalibStep(void)
{
    if (cal_state != 1U)
    {
        return;
    }

    cal_sum_u += adc1_raw[0];
    cal_sum_v += adc2_raw[0];
    cal_cnt++;

    if (cal_cnt >= FOC_CS_OFFSET_SAMPLES)
    {
        off_iu = (uint16_t)(cal_sum_u / cal_cnt);
        off_iv = (uint16_t)(cal_sum_v / cal_cnt);
        off_iw = 0U;                 /* IW 不再单独采样 */

        /* 合理性检查：CC6920 零电流输出 VCC/2=2.5V，经 1k/2k 分压后
           ≈ 1.667V，对应 ADC ≈ 2068 计数。
           如果读到接近 0（传感器没供电 / 引脚没接 / 没分压）或接近 4095，
           继续闭环会把 (raw-off) 当成几十安的电流，PI 直接饱和炸管。
           典型踩坑：先插 USB 上电（传感器还没电）→ 采到 off≈110 →
           再把 24V 合上 → raw=2089 → 假电流 36A。 */
        if ((off_iu < FOC_CS_OFFSET_MIN) || (off_iu > FOC_CS_OFFSET_MAX) ||
            (off_iv < FOC_CS_OFFSET_MIN) || (off_iv > FOC_CS_OFFSET_MAX))
        {
            current_adc_ok = 0U;
            cal_state      = 3U;        /* 3 = 校准失败（偏移不合理） */
        }
        else
        {
            current_adc_ok = 1U;
            cal_state      = 2U;        /* 2 = 校准成功 */
        }
    }
}

uint8_t FOC_Current_CalibDone(void)
{
    return (cal_state >= 2U) ? 1U : 0U;
}

uint8_t FOC_Current_CalibOk(void)
{
    return (cal_state == 2U) ? 1U : 0U;
}

void FOC_Current_RestartCalib(void)
{
    current_adc_ok = 0U;
    cal_sum_u = 0U;
    cal_sum_v = 0U;
    cal_cnt   = 0U;
    cal_state = 1U;      /* 重新开始采集 */
}

uint8_t FOC_Current_Init(void)
{
    RCC_PeriphCLKInitTypeDef pclk = {0};

    /* ADC12 clock = SYSCLK (170 MHz), then /4 inside the ADC = 42.5 MHz. */
    pclk.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    pclk.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK)
    {
        return 1U;
    }

    __HAL_RCC_ADC12_CLK_ENABLE();

    FOC_Current_GPIO_Init();
    if (FOC_Current_DMA_Init() != 0U)
    {
        return 8U;
    }

    if (FOC_ADC1_Init() != 0U)
    {
        return 2U;
    }
    if (FOC_ADC2_Init() != 0U)
    {
        return 3U;
    }

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
    {
        return 4U;
    }
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
    {
        return 5U;
    }

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_raw, 1U) != HAL_OK)
    {
        return 6U;
    }
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_raw, 1U) != HAL_OK)
    {
        return 7U;
    }

    /* 不再用 HAL_Delay 阻塞校准：这里只是打开采集开关，
       之后由 20kHz 中断里的 FOC_Current_CalibStep() 累计样本。
       这样启动流程不会因为 SysTick 异常而卡死，也不会拖慢 USB 枚举。 */
    cal_sum_u = 0U;
    cal_sum_v = 0U;
    cal_cnt   = 0U;
    cal_state = 1U;

    return 0U;
}

/* ------------------------------------------------------------------ */
/* Diagnostic accessors (send these to VOFA instead of printf)         */
/* ------------------------------------------------------------------ */
uint16_t FOC_Current_GetRaw(uint8_t idx)
{
    switch (idx)
    {
        case 0U: return adc1_raw[0];    /* IU */
        case 1U: return adc2_raw[0];    /* IV */
        case 2U: return 0U;             /* IW 不再单独采集（由 Ic=-(Ia+Ib) 推出） */
        default: return 0U;
    }
}

uint16_t FOC_Current_GetOffset(uint8_t idx)
{
    switch (idx)
    {
        case 0U: return off_iu;
        case 1U: return off_iv;
        case 2U: return off_iw;
        default: return 0U;
    }
}

/* ------------------------------------------------------------------ */
/* Current conversion                                                   */
/* ------------------------------------------------------------------ */
void FOC_Current_Read(float *ia, float *ib, float *ic)
{
    if ((ia == NULL) || (ib == NULL) || (ic == NULL))
    {
        return;
    }

    if (current_adc_ok == 0U)
    {
        *ia = 0.0f;
        *ib = 0.0f;
        *ic = 0.0f;
        return;
    }

    const float adc_to_amp =
        FOC_ADC_VREF / FOC_ADC_FULL_SCALE / FOC_CS_ADC_V_PER_A;

    /* 三点中值滤波：开关瞬间的 dv/dt 会在 ADC 引脚上打出一个不可能的单点
       （例如 raw=4700 对应传感器输出 5.68V，已经超过 5V 供电，物理上不存在）。
       中值滤波专门剔除这类单点尖峰，对这一层噪声最有效。 */
    static uint16_t ring_u[3] = {0U, 0U, 0U};
    static uint16_t ring_v[3] = {0U, 0U, 0U};
    static uint8_t  ring_idx  = 0U;

    ring_u[ring_idx] = adc1_raw[0];
    ring_v[ring_idx] = adc2_raw[0];
    ring_idx = (uint8_t)((ring_idx + 1U) % 3U);

    const uint16_t raw_u = FOC_Median3(ring_u[0], ring_u[1], ring_u[2]);
    const uint16_t raw_v = FOC_Median3(ring_v[0], ring_v[1], ring_v[2]);

    int32_t du = (int32_t)raw_u - (int32_t)off_iu;
    int32_t dv = (int32_t)raw_v - (int32_t)off_iv;

    float ra = FOC_CS_SIGN_IU * ((float)du * adc_to_amp);
    float rb = FOC_CS_SIGN_IV * ((float)dv * adc_to_amp);

    /* 硬限幅：超出传感器量程的尖峰不能进入 PI */
    if (ra >  FOC_CS_CLAMP_A) ra =  FOC_CS_CLAMP_A;
    if (ra < -FOC_CS_CLAMP_A) ra = -FOC_CS_CLAMP_A;
    if (rb >  FOC_CS_CLAMP_A) rb =  FOC_CS_CLAMP_A;
    if (rb < -FOC_CS_CLAMP_A) rb = -FOC_CS_CLAMP_A;

    /* 一阶 IIR 低通，抑制开关纹波 */
    static float   flt_a = 0.0f, flt_b = 0.0f;
    static uint8_t flt_ready = 0U;

    if (flt_ready == 0U)
    {
        flt_a = ra;
        flt_b = rb;
        flt_ready = 1U;
    }
    else
    {
        flt_a += FOC_CS_FILTER_ALPHA * (ra - flt_a);
        flt_b += FOC_CS_FILTER_ALPHA * (rb - flt_b);
    }

    /* 星形连接：第三相由前两相推出，天然满足 Ia+Ib+Ic = 0 */
    float rc = -(flt_a + flt_b);
    if (rc >  FOC_CS_CLAMP_A) rc =  FOC_CS_CLAMP_A;
    if (rc < -FOC_CS_CLAMP_A) rc = -FOC_CS_CLAMP_A;

    *ia = flt_a;
    *ib = flt_b;
    *ic = rc;
}

/* ------------------------------------------------------------------ */
/* d/q current loop                                                     */
/* ------------------------------------------------------------------ */
void FOC_CurrentSample_ISR(FOC_Handle_t *foc)
{
    if ((foc == NULL) || (current_adc_ok == 0U))
    {
        return;
    }

    float ia, ib, ic;
    FOC_Current_Read(&ia, &ib, &ic);

    foc->Ia = ia;
    foc->Ib = ib;
    foc->Ic = ic;

    /* Clarke */
    foc->I_alpha = ia;
    foc->I_beta  = (ia + 2.0f * ib) * FOC_ONE_OVER_SQRT3;

    /* Park */
    const float c = arm_cos_f32(foc->theta_elec);
    const float s = arm_sin_f32(foc->theta_elec);
    foc->Id =  foc->I_alpha * c + foc->I_beta * s;
    foc->Iq = -foc->I_alpha * s + foc->I_beta * c;
}


void FOC_CurrentLoop_ISR(FOC_Handle_t *foc)
{
    if ((foc == NULL) || (foc->current_loop_en == 0U) || (current_adc_ok == 0U))
    {
        return;
    }

    /* ---- 反馈失效看门狗 -------------------------------------------------
     * ADC/DMA 一旦不工作，三相原始值会完全冻结，电流永远为 0，PI 会一路
     * 积分到电压限幅，把电机当电阻灌。这里检测三相原始值连续 4000 次 ISR
     * （40kHz -> 100ms）一个都不变，就退出闭环。
     * ------------------------------------------------------------------ */
    static volatile uint16_t last_raw[2] = {0xFFFFU, 0xFFFFU};
    static uint16_t stuck_cnt = 0U;

    if ((adc1_raw[0] == last_raw[0]) &&
        (adc2_raw[0] == last_raw[1]))
    {
        if (stuck_cnt < 0xFFFFU)
        {
            stuck_cnt++;
        }
    }
    else
    {
        last_raw[0] = adc1_raw[0];
        last_raw[1] = adc2_raw[0];
        stuck_cnt   = 0U;
    }

    if (stuck_cnt > 4000U)
    {
        g_foc_trip_code      = 2U;      /* 2 = ADC 反馈冻结            */
        foc->current_loop_en = 0U;      /* 退出闭环                          */
        foc->voltage_setting = 0.0f;    /* 开环兜底也是 0V -> 三相 50% 占空比 */
        foc->Iq_ref          = 0.0f;
        foc->Iq_ref_target   = 0.0f;
        foc->Vd = 0.0f;
        foc->Vq = 0.0f;
        foc->Vd_int = 0.0f;             /* 清积分，防止下次进入时残余饱和      */
        foc->Vq_int = 0.0f;
        return;
    }

    FOC_CurrentSample_ISR(foc);

    /* ---- 过流 / 反馈异常保护 -------------------------------------------
     * Id 或 Iq 连续 10ms（40kHz 下 400 次）超过 1.5 倍限幅 -> 认为反馈异常
     * 或真的过流，停机。电机突然停转 = 触发了这条保护，
     * 具体原因看 g_foc_trip_code（1=过流 2=ADC反馈冻结）。
     * ------------------------------------------------------------------ */
    static uint16_t ovr_cnt = 0U;
    {
        const float i_lim = FOC_CURRENT_LIMIT_A * 1.5f;

        if ((foc->Id >  i_lim) || (foc->Id < -i_lim) ||
            (foc->Iq >  i_lim) || (foc->Iq < -i_lim))
        {
            if (ovr_cnt < 0xFFFFU)
            {
                ovr_cnt++;
            }
        }
        else
        {
            ovr_cnt = 0U;
        }

        if (ovr_cnt > 400U)     /* 400 x 25us = 10ms */
        {
            g_foc_trip_code      = 1U;  /* 1 = 过流保护触发 */
            foc->current_loop_en = 0U;
            foc->voltage_setting = 0.0f;
            foc->Iq_ref          = 0.0f;
            foc->Iq_ref_target   = 0.0f;
            foc->Vd = 0.0f;
            foc->Vq = 0.0f;
            foc->Vd_int = 0.0f;
            foc->Vq_int = 0.0f;
            return;
        }
    }

    /* Ramp Iq reference */
    const float dt = (float)FOC_ISR_PERIOD_US * 1e-6f;
    const float step = FOC_CURRENT_RAMP_A_S * dt;

    if (foc->Iq_ref < foc->Iq_ref_target)
    {
        foc->Iq_ref += step;
        if (foc->Iq_ref > foc->Iq_ref_target)
        {
            foc->Iq_ref = foc->Iq_ref_target;
        }
    }
    else if (foc->Iq_ref > foc->Iq_ref_target)
    {
        foc->Iq_ref -= step;
        if (foc->Iq_ref < foc->Iq_ref_target)
        {
            foc->Iq_ref = foc->Iq_ref_target;
        }
    }

    const float ed = foc->Id_ref - foc->Id;
    const float eq = foc->Iq_ref - foc->Iq;

    /* 线性调制区上限：Vbus/√3 * 0.90（24V -> 12.5V）。
       超过它 SVPWM 就会把 Ta/Tc 钳在 0/1（过调制），电流出现脉冲尖峰。 */
    const float vlim = foc->Vbus * FOC_ONE_OVER_SQRT3 * 0.90f;

    foc->Vd_int += foc->Ki_i * ed * dt;
    foc->Vq_int += foc->Ki_i * eq * dt;

    float vd = foc->Kp_i * ed + foc->Vd_int;
    float vq = foc->Kp_i * eq + foc->Vq_int;

    /* 矢量限幅 + 反算抗饱和：两轴一起缩放，保证合成矢量幅值不超限。
       原来把两轴各自钳到 13.2V，合成矢量最大能到 18.7V，直接过调制。 */
    float vmag = 0.0f;
    arm_sqrt_f32(vd * vd + vq * vq, &vmag);

    if (vmag > vlim)
    {
        const float k = vlim / vmag;
        vd *= k;
        vq *= k;
        foc->Vd_int *= k;
        foc->Vq_int *= k;
    }

    foc->Vd = vd;
    foc->Vq = vq;
}
