#include "sys_m4.h"
#include <stdio.h>


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
    while ((USART1->ISR & 0X40) == 0);              
    USART1->TDR = (uint8_t)ch;
    return ch;
}

int __io_putchar(int ch)
{
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









