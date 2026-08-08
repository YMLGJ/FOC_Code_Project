#include "bsp.h"
#include <string.h>
#include <stdbool.h>
#include <tim.h>
#include "foc.h" 


void BSP_Init(void)
{
    delay_init();        // 无参数，内部自动获取 SystemCoreClock
    delay_ms(1000);

    //MX_USB_Device_Init();  // 已在 main.c 中调用，这里不需要
    /* ---- FOC 初始化 ---- */
    FOC_Init(FOC_GetHandle());

    /* ---- 启动强拖: 15Hz, 2.5V ---- */
    FOC_StartForcedRotation(FOC_GetHandle(), 15.0f, 2.5f);
    delay_ms(1000);
    printf("System Ready. Motor starting...\r\n");
    printf("BSP_Init: OK\r\n");
}








