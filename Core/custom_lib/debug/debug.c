#include "bsp.h"
#include "debug.h"
#include <stdlib.h>
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;     /* 外部 USB 句柄声明 */


/*vofa数据帧*/
VOFA_Send_Handle_t VOFA_Handle = {
    .tail = {0x00, 0x00, 0x80, 0x7f},
};


void Debug_Task(void)
{
    
    //VOFA_Handle.fdata[0] = 0;
    //VOFA_Handle.fdata[1] = 1;
    //VOFA_Handle.fdata[2] = 2;
    //VOFA_Handle.fdata[3] = 3;
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    if (hcdc != NULL && hcdc->TxState == 0)
    {
        cdc_vcp_data_tx((uint8_t*)&VOFA_Handle, sizeof(VOFA_Handle));
    }
    __disable_irq();
    cdc_vcp_data_tx((uint8_t*)&VOFA_Handle, sizeof(VOFA_Handle));
    __enable_irq();
}


