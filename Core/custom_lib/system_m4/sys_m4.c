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


/***************************delay****************************/


/**
 * @brief     初始化延迟函数
 * @param     sysclk: 系统时钟频率, 即CPU频率(HCLK),等于系统时钟主频，单位MHz
 * @retval    无
 */  

static uint32_t sys_us = 0;

void delay_init(uint16_t sysclk)
{
    uint32_t reload = sysclk*1000;                              /* 1ms触发一次中断 */
    sys_us          = sysclk;

    SysTick->CTRL   |= SysTick_CTRL_TICKINT_Msk;                /* 开启systick中断 */
	SysTick->LOAD   = reload; 					                /* 装载值 */
	SysTick->CTRL   |= SysTick_CTRL_ENABLE_Msk;                 /* 启动systick定时器 */
}

void delay_us(uint32_t us)
{
    uint32_t ticks;
	uint32_t told,tnow,tcnt=0;
	uint32_t reload=SysTick->LOAD;				 	 
	ticks=us*sys_us; 						    
	told=SysTick->VAL;        				    
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	
			else tcnt+=reload-tnow+told;	    
 			told=tnow;
			if(tcnt>=ticks)break;			
		}  
	};
}

void delay_ms(uint32_t ms)
{
    uint32_t i;
	for(i=0;i<ms;i++) delay_us(1000);
}









