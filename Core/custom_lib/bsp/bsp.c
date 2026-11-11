#include "bsp.h"
#include <string.h>
#include <stdbool.h>
#include <tim.h>
#include "foc.h" 
#include "foc_current.h"
#include "foc_obs.h"

/* 电流采样初始化/校准的结果码，0 = 全部正常。
 * 通过 VOFA 通道送出（FOC_VOFA_RAW_DIAG = 3 时 I11）。 */
volatile uint8_t g_foc_init_code = 0xFFU;

/* 复位计数：放在链接脚本的 .noinit 段（复位不清零，但硬上电是随机值）。
 * 用魔数判断内容是否可信 —— 以前直接读裸 RAM，上电拿到一个随机数却小于 10000
 * 时会被当成"已经复位了 N 次"，白白误导判断。 */
typedef struct
{
    uint32_t magic;
    uint32_t boot;
} boot_rec_t;

#define BSP_BOOT_MAGIC   0xB007C0DEU

__attribute__((section(".noinit"), used))
volatile boot_rec_t g_boot_rec;

volatile uint32_t g_boot_count = 0U;

/* 1 = 上电后自动转起来（不推荐：会让"电机问题"和"USB 掉线"纠缠在一起）
 * 0 = 上电只做初始化，等串口命令 START / STOP 再动电机 */
#define BSP_AUTO_START   0

static void BSP_ApplyDq(float vd, float vq);

/* 升速斜率 [Hz/s]。I/f 开环从静止启动时太快会失步（转子跟不上），
 * 所以默认从原来的 3 Hz/s 降到 1 Hz/s，可用 RAMP 命令在线改。
 * START / SPEED 都统一用这个值，不再各自写死。 */
static float s_ramp_hz_s = 1.0f;


void BSP_AlignMotor(float id_a)
{
    FOC_Handle_t *foc = FOC_GetHandle();

    if (FOC_Current_CalibOk() == 0U)
    {
        printf("Refuse: ADC not ready\r\n");
        return;
    }

    if (id_a < 0.1f)  id_a = 0.1f;
    if (id_a > FOC_CURRENT_LIMIT_A) id_a = FOC_CURRENT_LIMIT_A;

    foc->current_loop_en = 1U;      /* 清掉上一次的过流停机 */

    /* 只加 d 轴电流，角度固定在 0：转子会被吸住不动。 */
    foc->state         = FOC_STATE_RUNNING;
    foc->theta_elec    = 0.0f;
    foc->current_freq  = 0.0f;
    foc->target_freq   = 0.0f;
    foc->Id_ref        = id_a;
    foc->Iq_ref        = 0.0f;
    foc->Iq_ref_target = 0.0f;
    foc->Vd_int        = 0.0f;
    foc->Vq_int        = 0.0f;
    g_foc_trip_code    = 0U;

    printf("ALIGN: Id=%dmA locked, expect Id=+%dmA Iq=0\r\n",
           (int)(id_a * 1000.0f), (int)(id_a * 1000.0f));
}


uint8_t BSP_StartMotor(float iq_a)
{
    FOC_Handle_t *foc = FOC_GetHandle();

    if (FOC_Current_CalibOk() == 0U)
    {
        /* 不要在这里动 target_freq：以前调用方不管成败都会再调 BSP_SetSpeed，
         * 结果 START 被拒了、目标频率却变了，看起来像"命令乱飘"。 */
        printf("Refuse: ADC not ready, init code %u\r\n", (unsigned)g_foc_init_code);
        return 0U;
    }

    /* 撤掉 d 轴电流，给 q 轴电流，从 0Hz 开始 I/f 旋转 */
    foc->Id_ref    = 0.0f;
    foc->ramp_rate = s_ramp_hz_s;
    FOC_StartCurrentMode(foc, 3.0f, iq_a);
    g_foc_trip_code = 0U;

    /* 观测器复位：角度预置为当前 I/f 角，方便后续无缝接管 */
    FOC_OBS_Reset(foc->theta_if);

    printf("Motor START, Iq=%dmA\r\n", (int)(iq_a * 1000.0f));
    return 1U;
}


/* 重新采集零漂：等 300ms 让 40kHz 中断采满 400 点，然后看结果。
 * 不阻塞电机运行以外的任何东西（此时电机本来就是停的）。 */
uint8_t BSP_Recalib(void)
{
    FOC_Handle_t *foc = FOC_GetHandle();

    FOC_Current_RestartCalib();

    for (uint8_t i = 0U; (i < 30U) && (FOC_Current_CalibDone() == 0U); i++)
    {
        delay_ms(10U);
    }

    if (FOC_Current_CalibOk() != 0U)
    {
        g_foc_init_code      = 0U;
        foc->current_loop_en = 1U;
        printf("CAL OK   off IU=%u IV=%u\r\n",
               (unsigned)FOC_Current_GetOffset(0U),
               (unsigned)FOC_Current_GetOffset(1U));
        return 1U;
    }

    g_foc_init_code      = 9U;
    foc->current_loop_en = 0U;
    printf("CAL FAIL off IU=%u IV=%u (need %u..%u)\r\n",
           (unsigned)FOC_Current_GetOffset(0U),
           (unsigned)FOC_Current_GetOffset(1U),
           (unsigned)FOC_CS_OFFSET_MIN, (unsigned)FOC_CS_OFFSET_MAX);
    return 0U;
}


void BSP_StopMotor(void)
{
    FOC_Stop(FOC_GetHandle());
    printf("Motor STOP\r\n");
}


void BSP_SetDtc(uint8_t en)
{
    FOC_GetHandle()->dtc_en = (en != 0U) ? 1U : 0U;
    printf("Dead-time comp %s\r\n", (en != 0U) ? "ON" : "OFF");
}


void BSP_SetSpeed(float hz)
{
    FOC_Handle_t *foc = FOC_GetHandle();

    if (hz < 0.5f)   { hz = 0.5f; }
    if (hz > 200.0f) { hz = 200.0f; }

    foc->target_freq = hz;
    foc->ramp_rate   = s_ramp_hz_s;        /* 用统一的斜坡速率，不再写死 3.0 */
    if (foc->state == FOC_STATE_RUNNING)
    {
        foc->state = FOC_STATE_RAMPING;
    }

    printf("SPEED -> %d mHz\r\n", (int)(hz * 1000.0f));
}


void BSP_SetSwitchHz(float hz)
{
    FOC_Handle_t *foc = FOC_GetHandle();

    if (hz < 0.0f)   { hz = 0.0f; }
    if (hz > 200.0f) { hz = 200.0f; }

    foc->obs_switch_hz = hz;
    printf("Sensorless switch at %d mHz\r\n", (int)(hz * 1000.0f));
}


void BSP_SetRamp(float hz_per_s)
{
    FOC_Handle_t *foc = FOC_GetHandle();

    if (hz_per_s < 0.05f) hz_per_s = 0.05f;
    if (hz_per_s > 20.0f) hz_per_s = 20.0f;

    foc->ramp_rate = hz_per_s;
    s_ramp_hz_s    = hz_per_s;
    printf("Ramp -> %d mHz/s\r\n", (int)(hz_per_s * 1000.0f));
}


void BSP_ForceSensorless(uint8_t en)
{
    FOC_Handle_t *foc = FOC_GetHandle();

    foc->sensorless_auto = 0U;             /* 手动模式优先 */

    if (en == 0U)
    {
        foc->sensorless_en = 0U;
        printf("Force IF mode\r\n");
        return;
    }

    if (FOC_OBS_IsLocked() == 0U)
    {
        printf("Refuse: observer not locked (EMF=%dmV)\r\n",
               (int)(FOC_OBS_GetEmf() * 1000.0f));
        return;
    }

    foc->sensorless_en = 1U;
    printf("Force sensorless mode\r\n");
}


void BSP_SetRL(float R, float L)
{
    FOC_OBS_SetRL(R, L);
    printf("Observer R=%d mohm, L=%d uH\r\n",
           (int)(R * 1000.0f), (int)(L * 1e6f));
}


void BSP_ApplyDc(float a, float b, float c)
{
    FOC_Handle_t *foc = FOC_GetHandle();

    if (FOC_Current_CalibOk() == 0U)
    {
        printf("Refuse: ADC not ready\r\n");
        return;
    }

    if (a < 0.0f) { a = 0.0f; }
    if (a > 1.0f) { a = 1.0f; }
    if (b < 0.0f) { b = 0.0f; }
    if (b > 1.0f) { b = 1.0f; }
    if (c < 0.0f) { c = 0.0f; }
    if (c > 1.0f) { c = 1.0f; }

    foc->current_loop_en = 0U;
    foc->voltage_test_en = 0U;
    foc->dc_test_en      = 1U;
    foc->dc_ticks        = 0U;
    foc->Ta_dc           = a;
    foc->Tb_dc           = b;
    foc->Tc_dc           = c;
    foc->state           = FOC_STATE_RUNNING;
    g_foc_trip_code      = 0U;

    printf("DC test: Ta=%d Tb=%d Tc=%d (3s auto stop)\r\n",
           (int)(a * 100.0f), (int)(b * 100.0f), (int)(c * 100.0f));
}


void BSP_ApplyVd(float vd)
{
    BSP_ApplyDq(vd, 0.0f);
}


void BSP_ApplyVq(float vq)
{
    BSP_ApplyDq(0.0f, vq);
}


static void BSP_ApplyDq(float vd, float vq)
{
    FOC_Handle_t *foc = FOC_GetHandle();

    if (FOC_Current_CalibOk() == 0U)
    {
        printf("Refuse: ADC not ready\r\n");
        return;
    }

    /* 绕开 PI：直接给固定 d/q 轴电压，theta 固定 0（不旋转） */
    foc->current_loop_en = 0U;
    foc->voltage_test_en = 1U;
    foc->dc_test_en      = 0U;
    foc->Vd_test         = vd;
    foc->Vq_test         = vq;
    foc->state           = FOC_STATE_RUNNING;
    foc->theta_elec      = 0.0f;
    foc->current_freq    = 0.0f;
    foc->target_freq     = 0.0f;
    g_foc_trip_code      = 0U;

    printf("VD/VQ test: Vd=%dmV Vq=%dmV\r\n",
           (int)(vd * 1000.0f), (int)(vq * 1000.0f));
}


void BSP_Init(void)
{
    {
        if (g_boot_rec.magic != BSP_BOOT_MAGIC)
        {
            g_boot_rec.boot  = 0U;          /* 硬上电：RAM 是随机值，重新计数 */
            g_boot_rec.magic = BSP_BOOT_MAGIC;
        }
        g_boot_rec.boot += 1U;
        g_boot_count = g_boot_rec.boot;
    }

    delay_init();        // 无参数，内部自动获取 SystemCoreClock
    delay_ms(1000);

    FOC_Handle_t *foc = FOC_GetHandle();

    /* ---- 电流采样初始化（ADC1/ADC2 + DMA） ---- */
    uint8_t current_ok = FOC_Current_Init();

    /* 零漂校准由 20kHz 中断推进（非阻塞）。
       传感器可能比 MCU 晚上电（先插 USB、后合 24V），这时采到的零漂是 0V，
       必须重来。失败就重试，最多 25 次 ≈ 2.5 秒，成功则清零错误码。
       注意：如果合 24V 的时间比这更晚，请先合母线再按复位。 */
    if (current_ok == 0U)
    {
        for (uint8_t retry = 0U; retry < 25U; retry++)
        {
            if (retry > 0U)
            {
                delay_ms(80U);
                FOC_Current_RestartCalib();
            }

            for (uint8_t i = 0U; (i < 6U) && (FOC_Current_CalibDone() == 0U); i++)
            {
                delay_ms(10U);
            }

            if (FOC_Current_CalibOk() != 0U)
            {
                break;
            }

            if (retry == 24U)
            {
                current_ok = 9U;   /* 9 = 2.5s 内都没采到合理零漂（传感器没电/没接） */
            }
        }
    }

    g_foc_init_code = current_ok;

    /* ---- FOC 初始化 ---- */
    FOC_Init(foc);
    foc->current_loop_en = (current_ok == 0U) ? 1U : 0U;

    /* 默认不转：state = IDLE 时 TIM1 中断里什么都不做，三相保持 50% 占空比。
       这样上电只有 USB + VOFA 在跑，"电机问题"和"USB 掉线"彻底分开。 */
    foc->state = FOC_STATE_IDLE;

    if (BSP_AUTO_START != 0)
    {
        if (current_ok == 0U)
        {
            BSP_AlignMotor(2.0f);
            delay_ms(500);
            BSP_StartMotor(3.0f);
        }
        else
        {
            /* 电流采样失败时的极低电压开环兜底，只适合短时间验证 */
            foc->voltage_min      = 0.2f;
            foc->theta_elec       = 0.0f;
            foc->current_freq     = 0.0f;
            foc->voltage_setting  = 0.2f;
            foc->state            = FOC_STATE_RUNNING;
            delay_ms(500);

            foc->ramp_rate = 0.5f;
            FOC_StartForcedRotation(foc, 2.0f, 0.3f);
        }
    }
    else if (current_ok == 0U)
    {
        float ia0, ib0, ic0;
        FOC_Current_Read(&ia0, &ib0, &ic0);
        printf("ADC zero: %dmA %dmA %dmA\r\n",
               (int)(ia0 * 1000.0f), (int)(ib0 * 1000.0f), (int)(ic0 * 1000.0f));
        printf("Ready. Send 'START 0.5' to run.\r\n");
    }

    printf("System Ready. Current loop %s (init code %u)\r\n",
           foc->current_loop_en ? "ON" : "OFF", (unsigned)current_ok);
}








