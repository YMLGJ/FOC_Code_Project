#include "sys_m4.h"
#include "debug.h"
#include <stdio.h>

/* ===================== 栈用量监测 ===================== */

#define STACK_PATTERN     0x5A5A5A5AU
#define STACK_SCAN_BYTES  8192U

/* 由链接脚本提供：栈顶、栈大小 */
extern uint32_t _estack;
extern uint32_t _Min_Stack_Size;

/* 监测窗口：栈顶往下 8KB。栈是从栈顶往下长的，所以只能看这里，
 * 从栈底（_sstack）往上扫是看不到实际用量的。 */
static uint32_t *stack_win_base(void)
{
    return (uint32_t *)((uint32_t)&_estack - STACK_SCAN_BYTES);
}

uint32_t stack_total_bytes(void)
{
    return STACK_SCAN_BYTES;
}

void stack_paint(void)
{
    uint32_t *p = stack_win_base();
    uint32_t *sp;

    __asm volatile ("mov %0, sp" : "=r"(sp));

    /* 只涂当前 SP 以下（尚未被使用）的部分，留 128 字（512B）余量 */
    while (p < (sp - 128))
    {
        *p++ = STACK_PATTERN;
    }
}

uint32_t stack_used_bytes(void)
{
    const uint32_t *p     = stack_win_base();
    const uint32_t  words = STACK_SCAN_BYTES / 4U;
    uint32_t        i     = 0U;

    while ((i < words) && (p[i] == STACK_PATTERN))
    {
        i++;
    }

    return STACK_SCAN_BYTES - (i * 4U);
}

/* 缓存版：只在 Debug_Task（500Hz）里刷新，中断里只读全局量 */
volatile uint32_t g_stack_used = 0U;
volatile uint32_t g_stack_free = 0U;

void stack_update(void)
{
    const uint32_t used  = stack_used_bytes();
    const uint32_t total = stack_total_bytes();

    g_stack_used = used;
    g_stack_free = (used < total) ? (total - used) : 0U;
}

/* ===================== 复位原因记录 ===================== */

#define RESET_MAGIC   0x52535443U      /* 'RSTC' */

__attribute__((section(".noinit"), used))
volatile uint32_t g_reset_rec[2];      /* [0]=magic [1]=count */

volatile uint32_t g_reset_cause = 0U;
volatile uint32_t g_reset_count = 0U;

void BSP_RecordResetCause(void)
{
    const uint32_t csr = RCC->CSR;

    if (g_reset_rec[0] != RESET_MAGIC)
    {
        g_reset_rec[0] = RESET_MAGIC;
        g_reset_rec[1] = 0U;
    }

    g_reset_rec[1] += 1U;

    g_reset_cause = csr;
    g_reset_count = g_reset_rec[1];

    __HAL_RCC_CLEAR_RESET_FLAGS();      /* 清标志，下次只反映新的复位 */
}


void soft_reset(void)
{
    NVIC_SystemReset();
}

/***************************串口映射到printf****************************/

#include <usart.h>

#if defined(__ARMCC_VERSION)       /* Keil ARMCC */
  #pragma import(__use_no_semihosting)
  struct __FILE { int handle; };
  FILE __stdout;
  int _ttywrch(int ch) { ch = ch; return ch; }
  void _sys_exit(int x) { x = x; }
  char *_sys_command_string(char *cmd, int len) { return NULL; }
#endif

int fputc(int ch, FILE *f)
{
    if (g_txt_mode != 0U) { dbg_putchar(ch); return ch; }   /* TXT 模式走 USB */
    while ((USART1->ISR & 0X40) == 0);              
    USART1->TDR = (uint8_t)ch;
    return ch;
}

int __io_putchar(int ch)
{
    /* printf 默认走 USART1(PB6)，用 USB 是看不到的。
       发过 TXT 1 之后就改走 USB CDC，用串口助手能直接看文本。 */
    if (g_txt_mode != 0U) { dbg_putchar(ch); return ch; }
    while ((USART1->ISR & 0X40) == 0);
    USART1->TDR = (uint8_t)ch;
    return ch;
}
/*******************************************************/


/* ======================== DWT 延时（不占用 SysTick） ======================== */

/**
 * @brief  初始化 DWT 周期计数器用于延时
 * @note   必须在 SystemClock_Config() 之后调用
 *         不与 HAL 的 SysTick 冲突，无需中断
 */
void delay_init(void)
{
    /* 使能 DWT 外设（由调试寄存器控制） */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* 清零并启动 DWT 周期计数器 */
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief  微秒级延时（基于 DWT 周期计数器）
 * @param  us : 延时微秒数
 * @note   最大延时受 uint32_t 限制：
 *         170MHz 时约 25.3 秒，远大于实际需求
 */
void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);

    while ((DWT->CYCCNT - start) < ticks)
    {
        /* 空循环等待，利用 uint32_t 回卷特性自然处理溢出 */
    }
}

/**
 * @brief  毫秒级延时
 * @param  ms : 延时毫秒数
 */
void delay_ms(uint32_t ms)
{
    uint32_t i;
    for (i = 0; i < ms; i++)
    {
        delay_us(1000);
    }
}









