#include "bsp.h"
#include <string.h>
#include <stdbool.h>
#include <tim.h>


void BSP_Init(void)
{
    delay_init(170);
    delay_ms(1000);
	
    //MX_USB_Device_Init();

    HAL_Delay(1000);
    printf("BSP_Init: OK\r\n");
}








