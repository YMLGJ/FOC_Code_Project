#ifndef _DEBUG_TASK_H
#define _DEBUG_TASK_H


/*vofa的通道数量*/
#define VOFA_CH_COUNT               16

#define VOFA_RECV_CH_COUNT               4

/*发送结构体*/
typedef struct{
    float fdata[VOFA_CH_COUNT];
    const unsigned char tail[4];          /*尾帧 0x00, 0x00, 0x80, 0x7f*/ 
}VOFA_Send_Handle_t;

/*接收结构体*/
typedef struct{
    float fdata[VOFA_RECV_CH_COUNT];
}VOFA_Recv_Handle_t;



void Debug_Task(void);

/* VOFA 诊断通道模式，可用 DIAG <n> 命令在线切换（无需重新烧录）
 *   0 = Ia / Ib / Ic
 *   1 = ADC 原始值 IU / IV / IW
 *   2 = 中断耗时(CPU周期) / IU原始 / IV原始
 *   3 = 初始化码 / IU零漂 / 保护码
 *   4 = 观测角θ̂ / 实际控制角 / 估算ω̂
 *   5 = 反电势幅值 / 估算ω̂ / 指令ω
 *   6 = 状态机state / 零漂校准OK / 电流环使能
 *   7 = 复位计数 / 命令计数 / 最近命令ID
 *   8 = 故障次数 / 出错PC高16位 / 出错PC低16位（PC = I12*65536 + I13）
 *   9 = CFSR / HFSR / 故障时的FOC状态                       */
extern volatile uint8_t g_vofa_diag;

/* 诊断计数器：判断“命令有没有收到”、“板子有没有在复位” */
extern volatile uint16_t g_cmd_count;
/* 1=DIAG 9=START成功 10=STOP 11=START被拒 12=CAL成功 13=CAL失败 0=其他 */
extern volatile uint8_t  g_last_cmd;
extern volatile uint32_t g_boot_count;
extern volatile uint32_t g_main_loop_count;

/* ===== 文本调试通道 =====
 * 默认 printf 走 USART1(PB6)，USB 上看不到 —— 这就是排查一首盲的原因。
 * 发 “TXT 1” 后：停掉 VOFA 二进制流，printf 改为直接发到 USB CDC，
 * 用任意串口助手（或者 VOFA 接收区）就能看到可读文本。
 * 发 “TXT 0” 回到二进制 VOFA 模式。 */
extern volatile uint8_t g_txt_mode;
void dbg_putchar(int ch);        /* printf 重定向目标，由 sys_m4.c 调用 */

/* 读取上一次硬件异常现场（实现在 stm32g4xx_it.c） */
void Fault_GetRecord(uint32_t *count, uint32_t *pc, uint32_t *cfsr,
                     uint32_t *hfsr, uint32_t *sp, uint32_t *state);

#endif


