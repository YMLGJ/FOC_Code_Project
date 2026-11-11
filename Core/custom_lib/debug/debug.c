#include "bsp.h"
#include "debug.h"
#include "main.h"
#include "foc.h"
#include "foc_current.h"
#include "foc_obs.h"
#include <stdlib.h>
#include <string.h>
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;     /* 外部 USB 句柄声明 */
extern PCD_HandleTypeDef  hpcd_USB_FS;      /* USB 外设（用于断开/重连 D+） */


/*vofa数据帧*/
VOFA_Send_Handle_t VOFA_Handle = {
    .tail = {0x00, 0x00, 0x80, 0x7f},
};


/* 命令计数，用于确认 USB 收到的命令是否真的进了解析器 */
volatile uint16_t g_cmd_count = 0U;
/* 最近一条命令的 ID：1=DIAG 9=START 10=STOP 0=其他 */
volatile uint8_t  g_last_cmd  = 0U;

/* 主循环计数：每秒应增约 500 次。如果波形卡住时它还在涨，
 * 说明主循环活着（问题在 USB）；不涨则是主循环被中断饿死了。 */
volatile uint32_t g_main_loop_count = 0U;

/* ================= 文本调试通道 ================= */
volatile uint8_t g_txt_mode = 0U;

static char     s_txt_buf[200];
static uint16_t s_txt_len = 0U;

/* printf 的一个字符：缓存；遇到换行立刻尝试发出去。
 * 之所以“遇到换行就发”，是为了万一后面主循环卡死，已经输出的那几行还看得到。 */
void dbg_putchar(int ch)
{
    if (s_txt_len < sizeof(s_txt_buf))
    {
        s_txt_buf[s_txt_len++] = (char)ch;
    }

    if (ch == '\n')
    {
        USBD_CDC_HandleTypeDef *hcdc =
            (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

        if ((hcdc != NULL) && (hcdc->TxState == 0U) &&
            (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED))
        {
            cdc_vcp_data_tx((uint8_t *)s_txt_buf, s_txt_len);
            s_txt_len = 0U;
        }
    }
}

/* 把当前所有关键状态打印一遍（在 TXT 模式下看得见） */
static void Debug_StatusDump(void)
{
    FOC_Handle_t *foc = FOC_GetHandle();
    extern volatile uint32_t g_foc_isr_cycles;   /* stm32g4xx_it.c */
    extern volatile uint32_t g_isr_overrun;      /* stm32g4xx_it.c */
    extern volatile uint8_t  g_foc_init_code;    /* bsp.c */

    printf("\r\n=== STATUS ===\r\n");
    printf("boot=%u cmd=%u last=%u\r\n",
           (unsigned)g_boot_count, (unsigned)g_cmd_count, (unsigned)g_last_cmd);
    printf("init=%u calibOk=%u loopEn=%u state=%u\r\n",
           (unsigned)g_foc_init_code,
           (unsigned)FOC_Current_CalibOk(),
           (unsigned)foc->current_loop_en,
           (unsigned)foc->state);
    printf("offset IU=%u IV=%u need=%u..%u\r\n",
           (unsigned)FOC_Current_GetOffset(0U),
           (unsigned)FOC_Current_GetOffset(1U),
           (unsigned)FOC_CS_OFFSET_MIN, (unsigned)FOC_CS_OFFSET_MAX);
    printf("isr=%u overrun=%u mainloop=%u\r\n",
           (unsigned)g_foc_isr_cycles, (unsigned)g_isr_overrun,
           (unsigned)g_main_loop_count);
    {
        uint32_t fc = 0U, fpc = 0U, fcfsr = 0U, fhfsr = 0U, fsp = 0U, fst = 0U;
        Fault_GetRecord(&fc, &fpc, &fcfsr, &fhfsr, &fsp, &fst);
        printf("fault n=%u pc=%08X cfsr=%08X hfsr=%08X sp=%08X state=%u\r\n",
               (unsigned)fc, (unsigned)fpc, (unsigned)fcfsr,
               (unsigned)fhfsr, (unsigned)fsp, (unsigned)fst);
    }
    printf("trip=%u\r\n", (unsigned)g_foc_trip_code);
    printf("stack used=%u free=%u\r\n",
           (unsigned)g_stack_used, (unsigned)g_stack_free);
    {
        const uint32_t c = g_reset_cause;
        printf("reset n=%u csr=%08X  pin=%u bor=%u sft=%u iwdg=%u wwdg=%u obl=%u\r\n",
               (unsigned)g_reset_count, (unsigned)c,
               (unsigned)((c >> 26) & 1U),   /* PINRSTF  引脚/上电复位 */
               (unsigned)((c >> 27) & 1U),   /* BORRSTF  欠压复位     */
               (unsigned)((c >> 28) & 1U),   /* SFTRSTF  软件复位     */
               (unsigned)((c >> 29) & 1U),   /* IWDGRSTF 独立看门狗   */
               (unsigned)((c >> 30) & 1U),   /* WWDGRSTF 窗口看门狗   */
               (unsigned)((c >> 25) & 1U));  /* OBLRSTF  选项字节装载 */
    }
    printf("target=%dmHz freq=%dmHz IqRef=%dmA Iq=%dmA Id=%dmA\r\n",
           (int)(foc->target_freq * 1000.0f), (int)(foc->current_freq * 1000.0f),
           (int)(foc->Iq_ref * 1000.0f),
           (int)(foc->Iq * 1000.0f),
           (int)(foc->Id * 1000.0f));
    printf("obs locked=%u theta=%dmrad omega=%dmrad/s emf=%dmV emfHz=%dmHz sens=%u auto=%u sw=%dmHz\r\n",
           (unsigned)FOC_OBS_IsLocked(),
           (int)(FOC_OBS_GetTheta() * 1000.0f),
           (int)(FOC_OBS_GetOmega() * 1000.0f),
           (int)(FOC_OBS_GetEmf() * 1000.0f),
           (int)(FOC_OBS_GetEmfHz() * 1000.0f),
           (unsigned)foc->sensorless_en,
           (unsigned)foc->sensorless_auto,
           (int)(foc->obs_switch_hz * 1000.0f));
    /* 观测器输入分项：Eα = Vα - R·Iα。Vα 和 R·Iα 谁不对一眼就能看出来。 */
    printf("Vab=%dmV,%dmV  Iab=%dmA,%dmA  Vbus=%dmV  Va_sin=%dmV\r\n",
           (int)(foc->V_alpha * 1000.0f), (int)(foc->V_beta * 1000.0f),
           (int)(foc->I_alpha * 1000.0f), (int)(foc->I_beta * 1000.0f),
           (int)(foc->Vbus * 1000.0f), (int)(foc->Va_sin * 1000.0f));
    {
        /* 观测角与 I/f 控制角的角度差（归一化到 ±π），切换判据就看它 */
        float dth = FOC_OBS_GetTheta() - foc->theta_if;
        while (dth >   3.14159265f) { dth -= 6.28318530f; }
        while (dth <  -3.14159265f) { dth += 6.28318530f; }
        printf("dth=%dmrad (switch needs |dth|<900)\r\n", (int)(dth * 1000.0f));
    }
    printf("==============\r\n");
}

/**
 * @brief 解析 USB CDC 收到的一行 ASCII 命令（在 VOFA 的发送框里输入，回车结束）
 *        ALIGN          只加 d 轴电流 0.5A，把转子锁住不转（用于验证电流相序）
 *        VD 0.05        开环纯 d 轴电压 0.05V（绕开 PI，直接验证传感器相序）
 *        DTC 0 / DTC 1  关闭 / 打开死区补偿
 *        DC 1 0 0       静态直流矢量：U 上管常开，V/W 下管常开（3s 自动停）
 *        SPEED 10       运行中修改目标电频率（Hz）
 *        SW 15          无感自动切换阀值（电频率 Hz），0 = 禁止自动切
 *        SMO 1 / SMO 0  手动强制切到观测器角度 / 退回 I/f
 *        RL 0.10 20     设置观测器用的相电阻(Ω)、相电感(μH)
 *        CAL            重新采集电流传感器零漂（不用断电重启）
 *        START          启动电机，默认 Iq = 0.5A
 *        START 1.0      启动电机，Iq = 1.0A
 *        STOP           停止电机
 */
static void Debug_CommandTask(void)
{
    if ((g_usb_usart_rx_sta & 0x8000U) == 0U)
    {
        return;                        /* 还没有收到完整的一行 */
    }

    char    *line = (char *)g_usb_usart_rx_buffer;
    uint16_t len  = (uint16_t)(g_usb_usart_rx_sta & 0x3FFFU);

    if (len >= USB_USART_REC_LEN)
    {
        len = USB_USART_REC_LEN - 1U;
    }
    line[len] = '\0';
    if ((len > 0U) && (line[len - 1U] == '\r'))
    {
        line[len - 1U] = '\0';
    }

    if (g_cmd_count < 60000U) { g_cmd_count++; }

    /* 注意：这里不能把 g_last_cmd 清零！
     * 以前每条命令开头都清零、DIAG 又置 1，结果“查“上一条命令结果”的命令
     * （DIAG 7 / DIAG 10）自己把结果覆盖掉了 —— 永远只能看到 1。
     * 现在 g_last_cmd 只是一个 sticky 的“最近一条有意义的命令”。 */

    if (strncmp(line, "DIAG", 4U) == 0)
    {
        float v = 0.0f;
        const char *p = line + 4;
        while (*p == ' ')
        {
            p++;
        }
        if (*p != '\0')
        {
            v = (float)atof(p);
        }
        g_vofa_diag = (uint8_t)v;      /* 立即生效，无需重烧 */
        /* 故意不动 g_last_cmd，否则就看不到上一条 START 的结果了 */
    }
    else if (strncmp(line, "ALIGN", 5U) == 0)
    {
        float id = 2.0f;
        const char *p = line + 5;
        while (*p == ' ')
        {
            p++;
        }
        if (*p != '\0')
        {
            id = (float)atof(p);
        }
        BSP_AlignMotor(id);
    }
    else if (strncmp(line, "DTC", 3U) == 0)
    {
        float v = 1.0f;
        const char *p = line + 3;
        while (*p == ' ')
        {
            p++;
        }
        if (*p != '\0')
        {
            v = (float)atof(p);
        }
        BSP_SetDtc((v != 0.0f) ? 1U : 0U);
    }
    else if (strncmp(line, "DC", 2U) == 0)
    {
        char *p = line + 2;
        float a = strtof(p, &p);
        float b = strtof(p, &p);
        float c = strtof(p, &p);
        BSP_ApplyDc(a, b, c);
    }
    else if (strncmp(line, "VQ", 2U) == 0)
    {
        float v = 0.5f;
        const char *p = line + 2;
        while (*p == ' ')
        {
            p++;
        }
        if (*p != '\0')
        {
            v = (float)atof(p);
        }
        BSP_ApplyVq(v);
    }
    else if (strncmp(line, "VD", 2U) == 0)
    {
        float v = 0.05f;
        const char *p = line + 2;
        while (*p == ' ')
        {
            p++;
        }
        if (*p != '\0')
        {
            v = (float)atof(p);
        }
        BSP_ApplyVd(v);
    }
    else if (strncmp(line, "SPEED", 5U) == 0)
    {
        float v = 3.0f;
        const char *p = line + 5;
        while (*p == ' ')
        {
            p++;
        }
        if (*p != '\0')
        {
            v = (float)atof(p);
        }
        BSP_SetSpeed(v);
    }
    else if (strncmp(line, "SW", 2U) == 0)
    {
        float v = 15.0f;
        const char *p = line + 2;
        while (*p == ' ')
        {
            p++;
        }
        if (*p != '\0')
        {
            v = (float)atof(p);
        }
        BSP_SetSwitchHz(v);
    }
    else if (strncmp(line, "RAMP", 4U) == 0)
    {
        /* RAMP <Hz/s>：设置升速斜率。I/f 从静止启动建议 0.3~1 Hz/s，
         * 太快（默认 3 Hz/s）转子跟不上会失步。 */
        float v = 1.0f;
        const char *p = line + 4;
        while (*p == ' ')
        {
            p++;
        }
        if (*p != '\0')
        {
            v = (float)atof(p);
        }
        BSP_SetRamp(v);
    }
    else if (strncmp(line, "RL", 2U) == 0)
    {
        char *p = line + 2;
        float R = strtof(p, &p);
        float L = strtof(p, &p);
        BSP_SetRL(R, L * 1e-6f);
    }
    else if (strncmp(line, "SMO", 3U) == 0)
    {
        float v = 1.0f;
        const char *p = line + 3;
        while (*p == ' ')
        {
            p++;
        }
        if (*p != '\0')
        {
            v = (float)atof(p);
        }
        BSP_ForceSensorless((v != 0.0f) ? 1U : 0U);
    }
    else if (strncmp(line, "START", 5U) == 0)
    {
        /* START <Iq电流A> [目标电频率Hz]
         *   例：START         → Iq=0.5A，频率 3Hz
         *       START 3       → Iq=3A，频率 3Hz
         *       START 3 10    → Iq=3A，频率 10Hz   */
        char  *p  = line + 5;
        float  iq = 0.5f;
        float  hz = 0.0f;          /* 0 = 用 BSP_StartMotor 里的默认 3Hz */

        while (*p == ' ') { p++; }
        if (*p != '\0')
        {
            iq = strtof(p, &p);
            while (*p == ' ') { p++; }
            if (*p != '\0')
            {
                hz = strtof(p, &p);
            }
        }

        if (BSP_StartMotor(iq) != 0U)
        {
            g_last_cmd = 9U;       /* 9 = START 成功 */

            if (hz > 0.0f)
            {
                BSP_SetSpeed(hz);  /* 启动后再把目标频率改掉 */
            }
        }
        else
        {
            g_last_cmd = 11U;      /* 11 = START 被拒绝（零漂校准未完成） */
            printf("START refused: run CAL after powering the bus\r\n");
        }
    }
    else if (strncmp(line, "CAL", 3U) == 0)
    {
        /* 重新采集零漂。适用于“先插 USB、后合母线”导致校准时传感器没供电 */
        if (BSP_Recalib() != 0U)
        {
            g_last_cmd = 12U;      /* 12 = CAL 成功 */
        }
        else
        {
            g_last_cmd = 13U;      /* 13 = CAL 失败 */
        }
    }
    else if (strncmp(line, "TXT", 3U) == 0)
    {
        /* TXT 1 = 停二进制流，printf 改走 USB（用串口助手看文本）
         * TXT 0 = 回到 VOFA 二进制模式 */
        float v = 1.0f;
        const char *p = line + 3;
        while (*p == ' ') { p++; }
        if (*p != '\0') { v = (float)atof(p); }

        g_txt_mode = (v != 0.0f) ? 1U : 0U;
        if (g_txt_mode != 0U)
        {
            Debug_StatusDump();
        }
    }
    else if (strncmp(line, "STATUS", 6U) == 0)
    {
        Debug_StatusDump();
    }
    else if (strncmp(line, "STOP", 4U) == 0)
    {
        BSP_StopMotor();
        g_last_cmd = 10U;
    }
    else if (len > 0U)
    {
        printf("Unknown cmd: %s\r\n", line);
    }

    g_usb_usart_rx_sta = 0U;           /* 清标志，准备接收下一行 */
}


void Debug_Task(void)
{
    g_main_loop_count++;

    Debug_CommandTask();

    stack_update();      /* 低频刷新栈水位（带缓存，中断里只读变量） */

    /* ===== USB 掉线自动恢复 =====
     * 电机大电流冲击后 USB PHY 有时会卡死：Windows 里设备变成 Error 状态，
     * 靠 MCU 复位也救不回来（必须拔插线或断电）。
     *
     * 这里用两步救：
     *   1) HAL_PCD_DevDisconnect() 拉低 D+ 上拉 → 主机看到"拔出"
     *   2) 200ms 后 MX_USB_Device_Init() → 重新接入，主机重新枚举
     * 这样连 Windows 缓存的 Error 状态也能被冲掉。 */
    {
        static uint8_t  s_was_cfg  = 0U;
        static uint16_t s_lost     = 0U;
        static uint8_t  s_reinit   = 0U;     /* 0=空闲 1=已断开 2=等时间到 */
        static uint32_t s_disc_t0  = 0U;

        if (s_reinit == 1U)
        {
            /* 刚断开 D+，记时间 */
            s_reinit  = 2U;
            s_disc_t0 = HAL_GetTick();
        }
        else if (s_reinit == 2U)
        {
            if ((HAL_GetTick() - s_disc_t0) > 200U)
            {
                MX_USB_Device_Init();        /* 重新挂上并枚举 */
                s_reinit = 0U;
            }
        }
        else if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
        {
            s_was_cfg = 1U;
            s_lost    = 0U;
        }
        else if (s_was_cfg != 0U)
        {
            if (++s_lost > 1500U)            /* 500Hz -> 3 秒 */
            {
                s_lost    = 0U;
                s_was_cfg = 0U;
                USBD_Stop(&hUsbDeviceFS);
                HAL_PCD_DevDisconnect(&hpcd_USB_FS);   /* 主机看到拔出 */
                s_reinit = 1U;
            }
        }
    }

    static VOFA_Send_Handle_t tx_snapshot;
    static uint32_t s_tx_busy_since = 0U;
    static uint8_t  s_tx_busy       = 0U;

    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

    if (hcdc == NULL)
    {
        return;
    }

    if (g_txt_mode != 0U)
    {
        /* 文本模式：不发二进制帧，把 printf 缓存一次性发出去 */
        if ((s_txt_len > 0U) &&
            (hcdc->TxState == 0U) &&
            (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED))
        {
            cdc_vcp_data_tx((uint8_t *)s_txt_buf, s_txt_len);
            s_txt_len = 0U;
        }
        return;
    }

    if (hcdc->TxState == 0U)
    {
        s_tx_busy = 0U;
        memcpy(&tx_snapshot, (const void*)&VOFA_Handle, sizeof(tx_snapshot));
        cdc_vcp_data_tx((uint8_t*)&tx_snapshot, sizeof(tx_snapshot));
    }
    else
    {
        /* 发送卡死看门狗：
         * 如果主机一段时间不取数据（拔线、端口被别的软件抢占、VOFA 卡住），
         * hcdc->TxState 会永远停在 1，此后数据流永久停摆 —— 现象就是
         * "波形突然冻住，但板子好像还活着"。超过 500ms 没进展就强制释放。 */
        const uint32_t now = HAL_GetTick();
        if (s_tx_busy == 0U)
        {
            s_tx_busy       = 1U;
            s_tx_busy_since = now;
        }
        else if ((now - s_tx_busy_since) > 500U)
        {
            hcdc->TxState   = 0U;
            s_tx_busy       = 0U;
        }
    }
}


