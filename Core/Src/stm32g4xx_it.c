/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "foc.h"
#include "debug.h"
#include "foc_current.h"
#include "foc_obs.h"
#include "sys_m4.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* VOFA 11/12/13 通道的用途：
 * 0 = 正常输出 Ia/Ib/Ic
 * 1 = 输出 ADC 原始值(IU/IV/IW)，用来确认采样是否活着
 *     零电流应≈2068 且抖动 <±15
 * 2 = 通道11 输出中断耗时(CPU周期)，4250 = 25us 满载；通道12/13 输出 IU/IV 原始值
 * 3 = 通道11 输出电流采样初始化错误码(0=正常)，通道12 输出 IU 零漂，
 *     通道13 输出保护触发原因(0=无 1=过流 2=ADC反馈冻结)
 * 4 = 通道11 观测器角θ̂，通道12 实际控制角，通道13 估算电角速度（无感调试）
 * 5 = 反电势幅值 / 估算ω̂ / 指令ω
 * 6 = 状态机state / 零漂校准OK / 电流环使能
 * 7 = 复位计数 / 命令计数 / 最近命令ID
 * 8 = 硬件故障次数 / 出错PC高16位 / 出错PC低16位（PC = I12*65536 + I13）
 * 9 = CFSR / HFSR / 故障时的FOC状态
 * 调试完成后请改回 0。 */
#define FOC_VOFA_RAW_DIAG   0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* 中断最大耗时（CPU 周期，170MHz）。40kHz 下一个周期只有 4250 周期，
   超过 3000（≈17.6us）就该优化了，否则 USB 和主循环会被饿死。 */
volatile uint32_t g_foc_isr_cycles = 0U;

/* VOFA 诊断通道模式，可用 DIAG 命令在线修改（无需重烧） */
volatile uint8_t g_vofa_diag = FOC_VOFA_RAW_DIAG;

/* 中断超时计数：本次中断耗时 >= 4150 周期（≈24.4us，接近 40kHz 的 25us 周期）时 +1。
 * 一旦这个数在涨，主循环就会被饿死 —— 表现就是“电机转起来了，但波形冻住、
 * 命令无响应、LED 不闪”。此时由中断自己接管闪灯，用 5Hz（正常是主循环 2Hz）
 * 告诉外面：CPU 没死，是中断吃满了 CPU。 */
volatile uint32_t g_isr_overrun = 0U;

/* ====================== 硬件异常现场记录 ======================
 * 以前 HardFault_Handler 是 while(1)，一旦触发 → CPU 停机 → USB 掉线、
 * 板子"装死"、复位计数也不涨，完全查不出来。
 * 现在：把故障现场存到 .noinit（复位不清零）里，然后主动软复位，
 * USB 会重新枚举，我们就可以用 DIAG 8 / DIAG 9 把现场读出来。 */
#define FAULT_MAGIC   0x4641554CU          /* 'FAUL' */

typedef struct
{
    uint32_t magic;
    uint32_t count;        /* 累计故障次数 */
    uint32_t pc;           /* 出错指令地址 */
    uint32_t lr;           /* 出错时的返回地址 */
    uint32_t psr;
    uint32_t cfsr;         /* 可配置故障状态寄存器 */
    uint32_t hfsr;         /* 硬故障状态寄存器 */
    uint32_t bfar;         /* 总线故障地址 */
    uint32_t mmfar;        /* 存储器故障地址 */
    uint32_t exc_return;
    uint32_t isr_cycles;   /* 出错时 FOC 中断的最长耗时 */
    uint32_t foc_state;
    uint32_t tick;
    uint32_t sp;           /* 出错时的栈指针（低于 _sstack 则为栈溢出） */
} fault_rec_t;

__attribute__((section(".noinit"), used))
volatile fault_rec_t g_fault_rec;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* 硬件异常公共处理：记录现场后软复位，让 USB 重新枚举 */
void Fault_Handler_C(uint32_t *sp, uint32_t exc_return) __attribute__((used, noreturn));
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  硬件异常公共处理
 * @param  sp        异常压栈的 8 字（r0 r1 r2 r3 r12 lr pc xpsr）
 * @param  exc_return 进入异常时的 LR
 * @note   保存现场 → 软复位。复位后 USB 重新枚举，用 DIAG 8/9 读现场。
 */
void Fault_Handler_C(uint32_t *sp, uint32_t exc_return)
{
    FOC_Handle_t *foc = FOC_GetHandle();
    uint32_t *msp_now;

    __asm volatile ("mov %0, sp" : "=r"(msp_now));

    g_fault_rec.count     += 1U;
    g_fault_rec.pc         = sp[6];
    g_fault_rec.lr         = sp[5];
    g_fault_rec.psr        = sp[7];
    g_fault_rec.exc_return = exc_return;
    g_fault_rec.cfsr       = SCB->CFSR;
    g_fault_rec.hfsr       = SCB->HFSR;
    g_fault_rec.bfar       = SCB->BFAR;
    g_fault_rec.mmfar      = SCB->MMFAR;
    g_fault_rec.isr_cycles = g_foc_isr_cycles;
    g_fault_rec.foc_state  = (foc != NULL) ? (uint32_t)foc->state : 0xFFFFFFFFU;
    g_fault_rec.tick       = HAL_GetTick();
    g_fault_rec.sp         = (uint32_t)msp_now;
    g_fault_rec.magic      = FAULT_MAGIC;   /* 最后写，保证前面都已落盘 */

    NVIC_SystemReset();

    while (1)
    {
    }
}

/**
 * @brief  读取上一次硬件异常现场（供 Debug_StatusDump 用）
 * @note   没发生过故障时 count = 0，其余全 0
 */
void Fault_GetRecord(uint32_t *count, uint32_t *pc, uint32_t *cfsr,
                     uint32_t *hfsr, uint32_t *sp, uint32_t *state)
{
    if (g_fault_rec.magic == FAULT_MAGIC)
    {
        if (count != NULL) { *count = g_fault_rec.count; }
        if (pc    != NULL) { *pc    = g_fault_rec.pc;    }
        if (cfsr  != NULL) { *cfsr  = g_fault_rec.cfsr;  }
        if (hfsr  != NULL) { *hfsr  = g_fault_rec.hfsr;  }
        if (sp    != NULL) { *sp    = g_fault_rec.sp;    }
        if (state != NULL) { *state = g_fault_rec.foc_state; }
    }
    else
    {
        if (count != NULL) { *count = 0U; }
        if (pc    != NULL) { *pc    = 0U; }
        if (cfsr  != NULL) { *cfsr  = 0U; }
        if (hfsr  != NULL) { *hfsr  = 0U; }
        if (sp    != NULL) { *sp    = 0U; }
        if (state != NULL) { *state = 0U; }
    }
}

/**
 * @brief  TIM 周期溢出回调（HAL 弱函数，在此重写）
 * @note   由 HAL_TIM_IRQHandler 在检测到更新事件时自动调用
 *         → 执行频率: 40kHz（PWM 由 20kHz 提高到 40kHz 以降低纹波）
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        const uint32_t isr_t0 = DWT->CYCCNT;

        /* 启动阶段先推进零漂校准（不阻塞主流程），累计满 400 点自动结束 */
        FOC_Current_CalibStep();

        FOC_Handle_t *foc = FOC_GetHandle();

        if (foc != NULL)
        {
            /* 只有运行态才跑控制；但 VOFA 数据无论什么状态都要更新，
               否则电机停着时改 DIAG 会完全看不到变化。 */
            if (foc->state != FOC_STATE_IDLE)
            {
                /* 强拖迭代：角度前进 → 逆Park → SVPWM → 更新PWM */
                FOC_ForcedRotation_ISR(foc);
            }

            extern VOFA_Send_Handle_t VOFA_Handle;
            VOFA_Handle.fdata[0]  = foc->theta_elec;     /* 电角度          */
            VOFA_Handle.fdata[1]  = foc->V_alpha;        /* Vα 电压指令     */
            VOFA_Handle.fdata[2]  = foc->V_beta;         /* Vβ 电压指令     */
            VOFA_Handle.fdata[3]  = foc->Ta;             /* A相占空比       */
            VOFA_Handle.fdata[4]  = foc->Tb;             /* B相占空比       */
            VOFA_Handle.fdata[5]  = foc->Tc;             /* C相占空比       */
            VOFA_Handle.fdata[6]  = foc->current_freq;   /* 当前电频率      */
            VOFA_Handle.fdata[7]  = foc->target_freq;    /* 目标电频率      */
            VOFA_Handle.fdata[8]  = foc->Va_sin;         /* A相正弦电压(注入前) */
            VOFA_Handle.fdata[9]  = foc->Vb_sin;         /* B相正弦电压(注入前) */
            VOFA_Handle.fdata[10] = foc->Vc_sin;         /* C相正弦电压(注入前) */

            switch (g_vofa_diag)
            {
            case 1:
                VOFA_Handle.fdata[11] = (float)FOC_Current_GetRaw(0U);
                VOFA_Handle.fdata[12] = (float)FOC_Current_GetRaw(1U);
                VOFA_Handle.fdata[13] = (float)FOC_Current_GetRaw(2U);
                break;

            case 2:
                VOFA_Handle.fdata[11] = (float)g_foc_isr_cycles;   /* 4250 = 25us 满载 */
                VOFA_Handle.fdata[12] = (float)FOC_Current_GetRaw(0U);
                VOFA_Handle.fdata[13] = (float)FOC_Current_GetRaw(1U);
                break;

            case 3:
            {
                extern volatile uint8_t g_foc_init_code;   /* 定义在 bsp.c */
                extern volatile uint8_t g_foc_trip_code;   /* 定义在 foc_current.c */
                VOFA_Handle.fdata[11] = (float)g_foc_init_code;
                VOFA_Handle.fdata[12] = (float)FOC_Current_GetOffset(0U);
                VOFA_Handle.fdata[13] = (float)g_foc_trip_code;
                break;
            }

            case 4:
                VOFA_Handle.fdata[11] = FOC_OBS_GetTheta();        /* 观测角 θ̂ [rad] */
                VOFA_Handle.fdata[12] = foc->theta_elec;           /* 控制角 [rad]   */
                VOFA_Handle.fdata[13] = FOC_OBS_GetOmega();        /* 估算 ω̂ [rad/s] */
                break;

            case 5:
                VOFA_Handle.fdata[11] = FOC_OBS_GetEmf();          /* 反电势幅值 [V] */
                VOFA_Handle.fdata[12] = FOC_OBS_GetOmega();        /* 估算 ω̂ [rad/s] */
                VOFA_Handle.fdata[13] = FOC_TWO_PI * foc->current_freq;  /* 指令 ω */
                break;

            case 6:
                VOFA_Handle.fdata[11] = (float)foc->state;              /* 0=IDLE 1=RAMPING 2=RUNNING 3=FAULT */
                VOFA_Handle.fdata[12] = (float)FOC_Current_CalibOk();   /* 1=零漂校准OK，START 才允许 */
                VOFA_Handle.fdata[13] = (float)foc->current_loop_en;    /* 1=电流环使能 */
                break;

            case 7:
                VOFA_Handle.fdata[11] = (float)g_boot_count;   /* 复位计数：一直变大 = 板子在复位 */
                VOFA_Handle.fdata[12] = (float)g_cmd_count;    /* 收到的命令行总数 */
                VOFA_Handle.fdata[13] = (float)g_last_cmd;     /* 1=DIAG 9=START 10=STOP 0=其他 */
                break;

            case 8:
                /* 故障次数 / 出错PC高16位 / 出错PC低16位
                 * 实际 PC = I12 * 65536 + I13
                 * 例：I12=2049(0x801)、I13=43981(0xABCD) → PC = 0x0801ABCD */
                if (g_fault_rec.magic == FAULT_MAGIC)
                {
                    VOFA_Handle.fdata[11] = (float)g_fault_rec.count;
                    VOFA_Handle.fdata[12] = (float)((g_fault_rec.pc >> 16) & 0xFFFFU);
                    VOFA_Handle.fdata[13] = (float)(g_fault_rec.pc & 0xFFFFU);
                }
                else
                {
                    VOFA_Handle.fdata[11] = 0.0f;
                    VOFA_Handle.fdata[12] = 0.0f;
                    VOFA_Handle.fdata[13] = 0.0f;
                }
                break;

            case 9:
                /* CFSR / HFSR / 出错时的 FOC 状态 */
                if (g_fault_rec.magic == FAULT_MAGIC)
                {
                    VOFA_Handle.fdata[11] = (float)g_fault_rec.cfsr;
                    VOFA_Handle.fdata[12] = (float)g_fault_rec.hfsr;
                    VOFA_Handle.fdata[13] = (float)g_fault_rec.foc_state;
                }
                else
                {
                    VOFA_Handle.fdata[11] = 0.0f;
                    VOFA_Handle.fdata[12] = 0.0f;
                    VOFA_Handle.fdata[13] = 0.0f;
                }
                break;

            case 10:
                /* 一次性看清"刚才那条命令发生了什么"（g_last_cmd 现在是 sticky 的，
                 * DIAG 自己不会覆盖它）：
                 *   I11 = 最近命令ID（9=START成功 11=START被拒 12=CAL成功 13=CAL失败）
                 *   I12 = 初始化/校准码（0=正常 8=DMA失败 9=零漂不合理）
                 *   I13 = 状态机state（0=IDLE 1=RAMPING 2=RUNNING 3=FAULT） */
                {
                    extern volatile uint8_t g_foc_init_code;   /* 定义在 bsp.c */
                    VOFA_Handle.fdata[11] = (float)g_last_cmd;
                    VOFA_Handle.fdata[12] = (float)g_foc_init_code;
                    VOFA_Handle.fdata[13] = (float)foc->state;
                }
                break;

            case 11:
                /* 栈水位：
                 *   I11 = 已用栈字节数
                 *   I12 = 剩余栈字节数（小于 300 就该担忧了）
                 *   I13 = 中断最长耗时（CPU周期，4250 = 25us 满载）
                 * 注意：这两个值是 Debug_Task 缓存过来的，中断里只读变量。 */
                VOFA_Handle.fdata[11] = (float)g_stack_used;
                VOFA_Handle.fdata[12] = (float)g_stack_free;
                VOFA_Handle.fdata[13] = (float)g_foc_isr_cycles;
                break;

            case 12:
                /* 主循环是否还活着：
                 *   I11 = 中断超时次数（>0 且持续增长 = 中断吃满了 CPU）
                 *   I12 = 中断最长耗时（CPU周期，4250 = 25us 满载）
                 *   I13 = 主循环计数（每秒应增约 500；不涨 = 主循环被饿死） */
                VOFA_Handle.fdata[11] = (float)(g_isr_overrun % 100000U);
                VOFA_Handle.fdata[12] = (float)g_foc_isr_cycles;
                VOFA_Handle.fdata[13] = (float)(g_main_loop_count % 100000U);
                break;

            default:
                VOFA_Handle.fdata[11] = foc->Ia;
                VOFA_Handle.fdata[12] = foc->Ib;
                VOFA_Handle.fdata[13] = foc->Ic;
                break;
            }

            VOFA_Handle.fdata[14] = foc->Id;              /* d 轴电流 [A]     */
            VOFA_Handle.fdata[15] = foc->Iq;              /* q 轴电流 [A]     */
        }

        /* 记录中断本体最长耗时 */
        const uint32_t isr_cyc = DWT->CYCCNT - isr_t0;
        if (isr_cyc > g_foc_isr_cycles)
        {
            g_foc_isr_cycles = isr_cyc;
        }

        /* 中断超时：吃掉周期预算的 97% 以上，主循环就没机会跑了。
         * 预算 = FOC_ISR_PERIOD_US * 170（20kHz -> 8500 周期）。
         * 以前这里写死 4150（那是 40kHz 的旧值），降到 20kHz 后会误报。 */
        const uint32_t over_budget = ((uint32_t)FOC_ISR_PERIOD_US * 170U) * 24U / 25U;
        if (isr_cyc > over_budget)
        {
            g_isr_overrun++;

            /* 主循环已被饿死，由中断代跑 ~5Hz 心跳（正常是主循环的 2Hz） */
            if ((g_isr_overrun & 0x0FFFU) == 0U)
            {
                HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
            }
        }
    }
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern TIM_HandleTypeDef htim1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  * @note  NMI 也可能是隐形的黑洞（CSS 时钟失效、奇偶错等）。同样记录+复位。
  */
__attribute__((naked, noreturn)) void NMI_Handler(void)
{
  __asm volatile
  (
    "tst   lr, #4          \n"
    "ite   eq              \n"
    "mrseq r0, msp         \n"
    "mrsne r0, psp         \n"
    "mov   r1, lr          \n"
    "b     Fault_Handler_C \n"
  );
}

/**
  * @brief This function handles Hard fault interrupt.
  * @note  不再 while(1) 死等：记录现场后软复位，USB 可重新枚举。
  */
__attribute__((naked, noreturn)) void HardFault_Handler(void)
{
  __asm volatile
  (
    "tst   lr, #4          \n"
    "ite   eq              \n"
    "mrseq r0, msp         \n"
    "mrsne r0, psp         \n"
    "mov   r1, lr          \n"
    "b     Fault_Handler_C \n"
  );
}

/**
  * @brief This function handles Memory management fault.
  */
__attribute__((naked, noreturn)) void MemManage_Handler(void)
{
  __asm volatile
  (
    "tst   lr, #4          \n"
    "ite   eq              \n"
    "mrseq r0, msp         \n"
    "mrsne r0, psp         \n"
    "mov   r1, lr          \n"
    "b     Fault_Handler_C \n"
  );
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
__attribute__((naked, noreturn)) void BusFault_Handler(void)
{
  __asm volatile
  (
    "tst   lr, #4          \n"
    "ite   eq              \n"
    "mrseq r0, msp         \n"
    "mrsne r0, psp         \n"
    "mov   r1, lr          \n"
    "b     Fault_Handler_C \n"
  );
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
__attribute__((naked, noreturn)) void UsageFault_Handler(void)
{
  __asm volatile
  (
    "tst   lr, #4          \n"
    "ite   eq              \n"
    "mrseq r0, msp         \n"
    "mrsne r0, psp         \n"
    "mov   r1, lr          \n"
    "b     Fault_Handler_C \n"
  );
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USB low priority interrupt remap.
  */
void USB_LP_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_IRQn 0 */

  /* USER CODE END USB_LP_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_IRQn 1 */

  /* USER CODE END USB_LP_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
