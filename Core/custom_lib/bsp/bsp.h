#ifndef _BSP_H
#define _BSP_H


#include "sys_m4.h"

#include "usb_device.h"


void BSP_Init(void);

/** 预对齐：只加 d 轴电流（0.5A）把转子锁在 theta=0，角度不旋转。
 *  用来验证电流相序：正确时 VOFA 上应看到 Id≈+0.5A、Iq≈0。 */
void BSP_AlignMotor(float id_a);

/** 启动电机：撤掉 Id、按斜坡给 Iq，从 0Hz 开始 I/f 旋转。
 *  iq_a = 目标 q 轴电流 [A]，会被 FOC_CURRENT_LIMIT_A 限幅。
 *  @retval 1 = 已启动；0 = 被拒绝（零漂校准未完成） */
uint8_t BSP_StartMotor(float iq_a);

/** 停止电机并把 PWM 恢复为 50% 占空比 */
void BSP_StopMotor(void);

/** 开环纯 d 轴电压测试：绕开电流环，theta 固定 0，直接施加 Vd。
 *  预测：Va=Vd、Vb=Vc=-Vd/2  → Ia≈Vd/R，Ib≈Ic≈-Ia/2。
 *  这是验证三相电流传感器相序最直接的实验。 */
void BSP_ApplyVd(float vd);

/** 开环纯 q 轴电压测试（theta=0 时 B、C 两相不等，可区分正/反相序） */
void BSP_ApplyVq(float vq);

/** 开关死区补偿（验驱动极性时要关掉，避免补偿污染测量） */
void BSP_SetDtc(uint8_t en);

/** 静态直流矢量测试：三相占空比固定（0~1），不经过 SVPWM、不开关。
 *  没有任何死区/纹波/采样时序干扰，是判断相序与极性的唯一无歧义手段。
 *  3 秒后自动停止。
 *  例：DC 1 0 0  → U 相上管常开，V/W 下管常开，电流只走 U→V/W
 *     预期 Ia=+I，Ib=Ic=-I/2 */
void BSP_ApplyDc(float a, float b, float c);

/** 运行中修改目标电频率 [Hz]（I/f 模式下的转速指令） */
void BSP_SetSpeed(float hz);

/** 设置无感自动切换阀值（电频率 Hz），0 = 禁止自动切换 */
void BSP_SetSwitchHz(float hz);

/** 设置角频率斜坡速率 [Hz/s]。
 *  I/f 开环从静止启动时，斜坡太快（>1 Hz/s）转子跟不上就会失步，
 *  表现为“电机在转但转速远低于指令、反电势很小”。 */
void BSP_SetRamp(float hz_per_s);
/** 手动强制切换：1 = 切到观测器角度（需已锁定），0 = 回 I/f */
void BSP_ForceSensorless(uint8_t en);

/** 设置观测器使用的相电阻[Ω]、相电感[H] */
void BSP_SetRL(float R, float L);

/** 重新采集电流传感器零漂（不需要断电重启）。
 *  用于"先插 USB、后合母线"导致校准时传感器没供电的场合。
 *  @retval 1 = 校准成功，可以 START；0 = 仍失败（传感器没电/没接） */
uint8_t BSP_Recalib(void);

#endif

