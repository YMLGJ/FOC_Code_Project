


#ifndef _SYS_M4_H
#define _SYS_M4_H

#include <stdint.h>
#include <main.h>

void soft_reset(void);              /* 软复位 */

void delay_init(uint16_t sysclk);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);


#endif

