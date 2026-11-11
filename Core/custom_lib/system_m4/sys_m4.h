#ifndef _SYS_M4_H
#define _SYS_M4_H

#include <stdint.h>
#include <main.h>

void soft_reset(void);              /* 软复位 */

void delay_init(void);              /* DWT延时初始化（ */
void delay_us(uint32_t us);         /* 微秒级延时 */
void delay_ms(uint32_t ms);         /* 毫秒级延时 */

/* 栈用量监测：启动时用固定图样涂满空闲栈区，之后扫描就能得到
 * 真正的"栈最深用了多少字节"。全程只读运行时，不占 CPU。 */
void     stack_paint(void);         /* 在 main 早期调用一次，必须在深调用之前 */
uint32_t stack_used_bytes(void);    /* 已用栈字节数（扫描栈顶下方 8KB） */
uint32_t stack_total_bytes(void);   /* 监测窗口大小 *//* 低频刷新（在 Debug_Task 里调）：把扫描结果缓存到下面两个全局量，
 * 这样 40kHz 中断里只读变量，不用去扫 1K 个字的栈。 */
void stack_update(void);
extern volatile uint32_t g_stack_used;   /* 已用字节 */
extern volatile uint32_t g_stack_free;   /* 剩余字节 */

/* ===== 复位原因记录 =====
 * 开机第一件事读 RCC->CSR 并存入 .noinit，status 就能知道
 * "到底是谁把板子打回复位的"：上电引脚、欠压(BOR)、软件、看门狗。 */
void BSP_RecordResetCause(void);
extern volatile uint32_t g_reset_cause;      /* 最近一次的 RCC->CSR 原始值 */
extern volatile uint32_t g_reset_count;      /* 累计复位次数 */

#endif

