# FOC 项目当前状态（已实测验证）

> 最后更新：与 `build/Debug/FOC_Code_Project.elf` 同步
> 所有数据均来自实际硬件测量，未经验证的项会标注 ⚠️

---

## 1. 已实测确认的硬件参数

| 项目 | 值 | 验证方式 |
|---|---|---|
| MCU | STM32G473CBT6 (Rev Y, Device ID 0x469) | SWD 读取 |
| 主频 | 170 MHz (HSE 25 MHz + PLL) | — |
| PWM | 20 kHz，中心对齐，TIM1，ARR = 4248，RCR = 1 | 示波器/寄存器 |
| 死区 | 40 counts ≈ 235 ns | 寄存器 |
| 相电阻 R | **0.06 Ω**（U-V 线间 0.12 Ω） | 万用表 |
| 极对数 | 7（12N14P） | 型号 |
| 电流传感器 | CC6920SO-20A，100 mV/A，1k/2k 分压 | 手册 + 实测 |
| 零漂 | IU ≈ 2081，IV ≈ 2090（理论 2068） | `STATUS` offset |
| 母线 | 24 V，限流 5 A | 实验电源 |

### PWM 通道与物理相对映（万用表验证，重要）

```
TIM1_CH1 (PA8)  →  W 相
TIM1_CH2 (PA9)  →  V 相
TIM1_CH3 (PA10) →  U 相
```

`FOC_UpdatePWM()` 里做了交叉映射：

```c
TIM1->CCR1 = foc->CCR3;   /* CH1 <- W */
TIM1->CCR2 = foc->CCR2;   /* CH2 <- V */
TIM1->CCR3 = foc->CCR1;   /* CH3 <- U */
```

**不要"修正"成直连，否则 Id/Iq 全是假的。**

### ADC 采样

| 通道 | 引脚 | ADC | 说明 |
|---|---|---|---|
| IU | PB0 | ADC1_IN15 | |
| IV | PA5 | ADC2_IN13 | |
| IW | PB12 | ADC1_IN11 | **不采样**，由 `Ic = -(Ia+Ib)` 推出 |

- 触发源：TIM1_TRGO (UPDATE)，与 PWM 同步
- 采样时间：47.5 cycles
- DMA：DMA1_Channel1/2 + **DMAMUX1**（两个时钟都要开！）

---

## 2. 当前功能状态

| 功能 | 状态 | 实测数据 |
|---|---|---|
| 三相桥 / 栅极驱动 | ✅ | — |
| ADC/DMA 同步采样 | ✅ | 零电流 2081±15 |
| Clarke/Park/InvPark/SVPWM | ✅ | `Id` 在 60 Hz 时可到 4 mA |
| 死区补偿 | ✅ | 线性过零过渡 |
| 电流环 PI | ✅ | `Iq` 跟踪 2000 mA，误差 <2% |
| I/f 开环运行 | ✅ | 20/50 Hz 均稳定 |
| **反电势过零测速** | ✅ | 与指令频率吻合（20Hz→20000mHz，50Hz→50000mHz） |
| **观测器 PLL** | ✅ | `ω̂` 误差 <3%（50Hz 时 305~323 rad/s，理论 314） |
| **手动无感切换 `SMO 1`** | ✅ | 切后电机正常运行；`trip=0` |
| 自动切换 I/f → 无感 | ❌ | 卡在 `\|dth\| < 0.9 rad` 门槛（见下） |
| 速度环 | ❌ | 未实现 |
| 带载测试 | ❌ | 未做 |

### 无感切换验证结果（`SMO 1`）

切到观测器角度后，电机从 50 Hz 电频率平滑加速到 **158 Hz（1106 RPM）**，
`emf` 随转速增长到 600 mV，`Iq` 继续跟踪 2 A，**无过流、无故障**。

> **不断加速是正确的物理行为**：当前只有电流环（恒力矩），没有速度环。
> 空载下恒力矩 → 一直加速，直到母线电压限制。

---

## 3. 串口命令参考

**连接方式**：USB CDC (COM4) 或 USART1 (PB6/PB7, 115200 8N1)。
命令以 `\r\n` 结尾，UTF-8 编码。

### 3.1 调试通道

| 命令 | 作用 |
|---|---|
| `TXT 1` | **停止 VOFA 二进制流，printf 改走 USB 纯文本**（配合任意串口助手） |
| `TXT 0` | 恢复 VOFA JustFloat 二进制模式 |
| `STATUS` | 打印完整状态快照（见 3.3） |
| `DIAG <n>` | 切换 VOFA 通道 11/12/13 的含义，n = 0~12 |

> `printf` 默认走 **USART1 (PB6)**。USB 上看不到任何文本，除非先发 `TXT 1`。

### 3.2 电机控制

| 命令 | 作用 |
|---|---|
| `START` / `START <Iq_A>` / `START <Iq_A> <Hz>` | 启动，默认 0.5 A、3 Hz |
| `STOP` | 停止，三相回 50% 占空比 |
| `SPEED <Hz>` | 运行中改目标电频率（0.5~200 Hz） |
| `RAMP <Hz/s>` | 升速斜率，默认 1.0（I/f 启动建议 0.3~1） |
| `CAL` | **重采电流零漂**（上电顺序反了用它，不用断电） |
| `ALIGN [Id_A]` | 只加 d 轴电流锁转子，验证相序 |
| `DC <a> <b> <c>` | 静态直流矢量（3 s 自动停，1.5 A 保护） |
| `VD <V>` / `VQ <V>` | 纯电压开环测试（绕开 PI） |
| `DTC <0\|1>` | 开关死区补偿 |

### 3.3 `STATUS` 输出示例

```
=== STATUS ===
boot=1 cmd=1 last=0                                  ← 复位计数 / 命令计数 / 最近命令ID
init=0 calibOk=1 loopEn=1 state=0                    ← 初始化码 / 校准OK / 电流环使能 / 状态机
offset IU=2081 IV=2090 need=1000..3000               ← 零漂实测值
isr=456 overrun=0 mainloop=30799                     ← 中断耗时(周期) / 超时次数 / 主循环计数
fault n=0 pc=00000000 cfsr=... hfsr=... sp=... state=0   ← 硬件异常记录
trip=0                                               ← 保护码：0无 1过流 2ADC冻结
stack used=936 free=7256                             ← 栈水位
reset n=1 csr=14000000  pin=1 bor=0 sft=1 ...        ← 复位原因（RCC->CSR 解码）
target=20000mHz freq=20000mHz IqRef=2000mA Iq=2029mA Id=39mA
obs locked=0 theta=5880mrad omega=1194mrad/s emf=123mV emfHz=20000mHz sens=0 auto=1 sw=15000mHz
Vab=-12mV,195mV  Iab=705mA,1935mA  Vbus=24000mV  Va_sin=-12mV
dth=1170mrad (switch needs |dth|<900)                ← 观测角与 I/f 角之差
==============
```

### 3.4 关键字段解码

**`g_last_cmd`（sticky，`DIAG` 不覆盖）**

| 值 | 含义 |
|---|---|
| 9 | START 成功 |
| 10 | STOP |
| 11 | START 被拒绝（校准未完成） |
| 12 | CAL 成功 |
| 13 | CAL 失败 |

**`state`**

| 值 | 含义 |
|---|---|
| 0 | IDLE |
| 1 | RAMPING |
| 2 | RUNNING |
| 3 | FAULT |

**`trip`**

| 值 | 含义 |
|---|---|
| 0 | 无 |
| 1 | 过流（Id/Iq 超 1.5×限幅持续 10 ms） |
| 2 | ADC 反馈冻结（原始值 100 ms 不变） |

**`reset` 位**（`RCC->CSR`，位 25~31）

| 位 | 名称 | 含义 |
|---|---|---|
| 26 | PINRSTF | NRST 引脚 / 上电 |
| 27 | BORRSTF | **欠压** |
| 28 | SFTRSTF | 软件复位 |
| 29/30 | IWDG/WWDG | 看门狗 |
| 25 | OBLRSTF | 选项字节装载 |

### 3.5 `DIAG` 模式（VOFA 通道 11/12/13）

| n | 通道 11 / 12 / 13 |
|---|---|
| 0 | Ia / Ib / Ic |
| 1 | ADC 原始 IU / IV / IW |
| 2 | 中断耗时(周期) / IU / IV |
| 3 | init_code / off_iu / trip_code |
| 4 | 观测角 θ̂ / 控制角 / ω̂ |
| 5 | emf / ω̂ / 指令 ω |
| 6 | state / calibOk / loopEn |
| 7 | boot_count / cmd_count / last_cmd |
| 8 | fault_count / fault_pc 高16位 / 低16位 |
| 9 | CFSR / HFSR / 故障时 state |
| 10 | last_cmd / init_code / state |
| 11 | 栈已用 / 栈剩余 / 中断耗时 |
| 12 | 中断超时次数 / 中断耗时 / 主循环计数 |

固定通道：0=θ, 1=Vα, 2=Vβ, 3=Ta, 4=Tb, 5=Tc, 6=current_freq, 7=target_freq, 8~10=Va/Vb/Vc_sin, 14=Id, 15=Iq

### 3.6 观测器参数

| 命令 | 作用 |
|---|---|
| `RL <R_Ω> <L_µH>` | 设置观测器相电阻/电感，**实测 R = 0.06** |
| `SW <Hz>` | 无感自动切换阈值，默认 15 Hz，0 = 禁止 |
| `SMO <0\|1>` | 手动强制切到观测器角度 / 退回 I/f |

---

## 4. 构建与烧录

### 4.1 构建

用 VS Code 的 CMake Tools 构建，或：

```powershell
cmake --build build/Debug
```

构建后需要生成 `.bin`（**每次改代码都必须重新生成，否则会烧录旧固件**）：

```powershell
& "C:\Users\YMLGJ\AppData\Local\stm32cube\bundles\gnu-tools-for-stm32\14.3.1+st.2\bin\arm-none-eabi-objcopy.exe" `
  -O binary "E:\FOC\FOC_Code_Project\build\Debug\FOC_Code_Project.elf" `
            "E:\FOC\FOC_Code_Project\build\Debug\FOC_Code_Project.bin"
```

### 4.2 烧录

```powershell
& "C:\Users\YMLGJ\AppData\Local\stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe" `
  -c port=SWD mode=normal freq=100 `
  -e all --skipErase `
  -w "E:\FOC\FOC_Code_Project\build\Debug\FOC_Code_Project.bin" 0x08000000 -v -rst
```

### 4.3 串口抓取脚本

`E:\FOC\cmd.ps1`：

```powershell
& E:\FOC\cmd.ps1 -Cmds "TXT 1","STATUS" -Wait 2000
& E:\FOC\cmd.ps1 -Cmds "TXT 1","RAMP 2","START 2 50","@22000","STATUS" -Wait 1200
```

- `@<毫秒>` = 纯等待，不发命令
- 脚本**故意不调用 `Close()`**，避免 USB 设备掉线时卡死终端
- 首次运行需要 `Set-ExecutionPolicy -Scope Process Bypass`

---

## 5. ⚠️ 环境陷阱（每次都会踩）

| 陷阱 | 现象 | 规避 |
|---|---|---|
| **SWD 频率过高** | `ST-LINK error (DEV_CONNECT_ERR)` | **必须用 `freq=100`**（800 会失败） |
| **SWD 连接会复位 MCU** | 注入的 RAM 数据/运行状态丢失 | 不能靠 SWD 做交互调试 |
| **改了代码没重新 objcopy** | 烧进去的是旧固件，调试结论全错 | 每次 build 后必跑 objcopy |
| **ST-Link 3.3V 与 buck 并联** | `PINRSTF` 复位（NRST 被拉低） | 二者只能接一个 |
| **ST-Link 3.3V 断开后** | 母线一断 MCU 就没电，SWD/USB 全失 | 调试时母线必须开着 |
| **VOFA 占用 COM4** | PowerShell 打不开串口 | 先断开 VOFA 的串口 |
| **串口是独占资源** | 一个 COM 口只能被一个程序打开 | 同上 |
| **双击 VOFA 文本框看不到 printf** | 文本走 USART1 | 先发 `TXT 1` |

---

## 6. 已知问题与下一步

### 6.1 自动切换被 `dth` 门槛卡住 ❌

实测 I/f 模式下 `dth = θ̂ - theta_if`：

| 工况 | `dth` | 说明 |
|---|---|---|
| 50 Hz / 2.0 A | 1.17~1.21 rad (68°) | 稳定 |
| 50 Hz / 0.5 A | 2.5~2.6 rad (145°) | 稳定 |

**随电流变化 → 这是真实负载角，不是坐标偏差。** 因此 `|dth| < 0.9 rad` 的硬门槛永远过不去。

**建议改法（角度跟踪式交接）**：

```c
if (locked && current_freq >= obs_switch_hz) {
    foc->theta_if += 0.00005f * dth;      /* ~1 s 时间常数拖向观测角 */
}
if (fabsf(dth) < 0.15f) { sensorless_en = 1; }   /* 拖到位再切，无冲击 */
```

### 6.2 没有速度环 ❌

切到无感后恒 `Iq` → 空载持续加速（实测 50 Hz → 158 Hz）。
需要外环：

```c
Iq_ref = Kp_s * (freq_target - emfHz) + Ki_s * integral;
```

`emfHz`（过零法）可直接作为**模型无关的真实转速反馈**。

### 6.3 其它待办

- [ ] 观测器加入 `L·di/dt` 项（当前忽略，EMF 信噪比偏低）
- [ ] `Vbus` 目前是硬编码 24.0，建议加 ADC 采样
- [ ] 高速下 `locked` 会掉 0（`eps_n < 0.25` 判据偏严）
- [ ] 带载测试 / 温度保护
- [ ] `README.md` 的硬件表（ADC 一项仍写"待配置"）需要更新
