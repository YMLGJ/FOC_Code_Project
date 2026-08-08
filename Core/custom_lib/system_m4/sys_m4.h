#ifndef _SYS_M4_H
#define _SYS_M4_H

#include <stdint.h>
#include <main.h>

void soft_reset(void);              /* 软复位 */

void delay_init(void);              /* DWT延时初始化（ */
void delay_us(uint32_t us);         /* 微秒级延时 */
void delay_ms(uint32_t ms);         /* 毫秒级延时 */

#endif

