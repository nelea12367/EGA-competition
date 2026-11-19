# 交接文档（给下一个 Agent）

> ⚠ **2026-11-11 更新**：CAN 部分已经改好了——固件不再写死 FDCAN3，改成上电自动扫描
> FDCAN1/2/3 并用灯报告结果。原因和判读见 **`README_CAN排查_最新.md`**。
> 本文第 3 节"FDCAN3 是电调那一路"的判断是错的，第 5 节的排查步骤已被那份文档取代。

> 你接手的是一个 **STM32H723 麦轮底盘**项目。目标只有一条：**遥控摇杆 → 四个麦轮转起来**。
> 当前状态：**遥控 ✅ / 灯 ✅ / CAN ❌（唯一阻塞项）**。
> 本文档只写"你需要的、已验证的事实"和"下一步怎么做"，不写过程。

---

## 0. 一句话背景

用户手上是一台达妙 **DM-MC02**（STM32H723VGT6）麦轮车，遥控 **FS-i6X + FS-iA10B**，电机 **4×M3508 + 4×C620**（电调灯闪 1/2/3/4，ID 已设好，24V 已供电）。
原始工程 `DM_MC02_Test.zip` **开机就 HardFault，不能用**，现在的工程是在它基础上重写的。

---

## 1. 马上能用的三样东西

```powershell
cd dm_mc02_chassis

# 编译
powershell -ExecutionPolicy Bypass -File .\build.ps1

# 烧录（PWLINK2/DAP-Link + OpenOCD，自动软复位）
powershell -ExecutionPolicy Bypass -File .\flash.ps1

# 或者直接烧已经编好的
powershell -ExecutionPolicy Bypass -File .\flash.ps1 -Hex firmware\DM_MC02_Chassis.hex
```

**工具链**：`C:\ST\STM32CubeIDE_2.2.0`（gcc 14.3 + OpenOCD，脚本会自动找路径）
**调试器**：PWLINK2 = CMSIS-DAP（VID 0D28 / PID 0204）→ **STM32CubeProgrammer 不认，只能走 OpenOCD**
**注意**：所有 `.ps1` 必须是**纯 ASCII**（Windows PowerShell 5.1 无 BOM 时按 ANSI 读，中文会语法报错）

---

## 2. 已验证的硬件事实（⚠ 不要重新猜，这些都是实测的）

| 项目 | 事实 |
|---|---|
| MCU | STM32H723VGT6，1MB Flash，AXI SRAM @0x24000000（栈顶 0x24050000） |
| 遥控输入 | **UART5，PD2 = RX**，`100000 8E2`，**SBUS**（25 字节帧，0x0F 帧头） |
| 遥控协议选择 | 遥控器菜单 `RX Setup → Output mode` 必须是 **`PPM and SBUS`**（选 iBUS 时串口脚不出数据） |
| 遥控通道 | CH1~CH4 = 摇杆；**CH7 = SWA、CH8 = SWB**（两段开关）；中位≈1024，两端 240/1807 |
| 状态灯 | WS2812，**SPI6：PA5=SCK、PA7=MOSI**，2.148MHz，**GRB 顺序** |
| CAN 通道 | 官方例程：FDCAN1=PD0/PD1、**FDCAN2=PB5/PB6**、FDCAN3=PD12/PD13(AF2) |
| 板上 CAN 座 | 丝印 `CAN1`/`CAN2`，**哪个对应哪个 FDCAN 尚未最终确认**（见第 5 节） |
| 电机 | 4×M3508 + C620，反馈 0x201~0x204（若 ID 非 1~4 则在 0x205~0x208） |
| 官方资料 | `https://gitee.com/kit-miao/dm-mc02`（原理图、说明书、**例程/CtrBoard-H7_CAN**） |

---

## 3. 现在的运行状态（最后一次实测）

### ✅ 遥控：完全正常
```
frame_counter = 499 且持续增长
rc_ok = 1
CH1~CH4 = 967/964/1020/1024（实时跟随摇杆）
CH7 拨动 = 240 <-> 1807（灯会跟着变）
```

### ✅ 灯：驱动正常
`CHASSIS_LED_TEST=1` 时循环显示 红→绿→蓝→白→灭，**实测能按顺序变**，说明 SPI 时序与颜色顺序都对。

### ❌ CAN：通了但没数据（唯一问题）
三通道同时发 200 帧的实测结果：

| 通道 | TXBTO（成功发出） | TEC（错误计数） | 判读 |
|---|---|---|---|
| FDCAN1 (PD0/PD1) | 0 | **128** | 有收发器、发出去了，**没人 ACK** |
| FDCAN2 (PB5/PB6) | 0 | **128** | 同上 |
| FDCAN3 (PD12/PD13) | 0 | 0 | **压根没发**（这路可能没接收发器） |

补充：FDCAN3 内部回环自测**能收到自己发的帧**（说明外设本身没问题，是这路外部不通）。
`PD12` 电平 = 高（总线空闲，没被压死）。

**结论：电调的 CAN 没有真正接到 FDCAN1/FDCAN2 这两条总线上。**

---

## 4. 关键坑（现象 → 根因 → 修复位置）

| # | 现象 | 根因 | 现在在哪修的 |
|---|---|---|---|
| 1 | 原 demo 灯亮但不动 | **开机 HardFault**（PC 停在 HardFault_Handler，CFSR=UNDEFINSTR） | 弃用原 demo，重写应用层 |
| 2 | 遥控一帧收不到 | 协议错 / ORE 溢出 / 中断收包不可靠 | `rc_input.c`：固定 SBUS + `OVRDIS` + **主循环轮询** |
| 3 | 收一会儿就死 | UART **ORE 溢出**后 RXNE 卡住 | `rc_input.c` 的 `rc_uart_config()` 开 `UART_ADVFEATURE_RXOVERRUNDISABLE_INIT` |
| 4 | 灯能发但不亮/颜色不跟 | HAL SPI 状态卡死 + 颜色顺序 + 逻辑条件 | `status_led.c`（SPI + 失败重初始化）、`main.c` |
| 5 | 灯"拨了没反应" | 有两个原因：(a) 烧的是 `CHASSIS_DIAG_MODE=1` 的诊断固件（不跑灯）；(b) 灯逻辑里加了 `frame_valid` 判断，丢一帧就跳 | 见第 7 节约束 |
| 6 | CAN 读寄存器结论反复 | **寄存器偏移记错**：`TXBC=0xC0`、`TXBRP=0xCC`、`TXBTO=0xD8`、`RXF0S=0xA4`（0xA4 不是 TXBC！） | 已更正 |
| 7 | 烧完程序不跑 | `reset run` 常留成 halt | `flash.ps1`：写 AIRCR=0x05FA0004 软复位两次 |
| 8 | `.ps1` 报语法错 | PS 5.1 按 ANSI 读无 BOM 文件 | 所有脚本保持纯 ASCII |

---

## 5. 下一步：CAN（**唯一任务**）

### 5.1 建议的排查顺序（从最省时间开始）

**实验 A｜最小系统直连**
拿**一块** C620，CAN 线**直接**接到板子 `CAN2` 座，只接 `CAN_H / CAN_L / GND` 三根（**不经集线板**），24V 仍从集线板取。
然后读 FDCAN2 的 `TEC`：

- `TEC=0` 且收到 0x201 → **通了**，问题是集线板（它可能只分了电源、没通 CAN）
- `TEC=128` → 板子这个座到收发器之间有问题，换 `CAN1` 座再试

**实验 B｜如果 A 两座都不行**
量电压（上电、黑表笔接 GND）：
- `CAN_H`、`CAN_L` 对 GND 都应是 **≈2.5V**（空闲隐性）；0V 或 5V = 短路/收发器坏
- 断电量 `CAN_H`-`CAN_L`：应 **≈60Ω**（两端各 120Ω 并联；∞ = 没有终端电阻）

**实验 C｜拿官方例程当对照**
下载 `例程/CtrBoard-H7_CAN`，**先单独烧它**，确认能收到 0x201~0x204。
如果它也收不到 → 硬件/接线问题；如果能收到 → 把它的 `fdcan.c` 整段搬过来（我们的 `Can_Init` 可能还有隐藏差异）。

### 5.2 判读表（读寄存器用）

| 现象 | 含义 |
|---|---|
| `TXBTO` 在涨 | 帧真的发出去了 |
| `TEC` 涨到 248 | 发出去了但没人 ACK（线/电阻/电调没上电） |
| `TXBTO=0` 且 `TEC=0` | 压根没发（引脚没接收发器 / 控制器没进入正常态） |
| `RXF0S` > 0 但计数不涨 | 收到了但没读（中断/轮询问题） |

### 5.3 CAN 通了之后

1. `CHASSIS_ID_PROBE_ON_BOOT=1` → 烧录 → 灯闪 N 下 = 该电调是 N 号（验证 ID）
2. `CHASSIS_SELFTEST_ON_BOOT=1` → 烧录 → 1→2→3→4 号电机依次点动（验证轮序/转向）
3. 位置不对改 `chassis.c` 的 `wheel_linear[]` 下标；转向不对改 `CHASSIS_MOTOR_REVERSED_MASK`
4. 实测几何参数填进 `chassis_config.h`（轮半径/半轴距/半轮距）

---

## 6. 代码地图

```
Core/Inc/chassis_config.h   ★所有参数都在这里（几何/限速/PID/通道/功能开关）
Core/Inc/chassis.h         底盘对外接口（Chassis_SetVelocity 等，给 3 号用）
Core/Src/chassis.c         麦轮解算 + 速度环 + 安全状态机 + ID探测/单轮自检
Core/Src/rc_input.c        SBUS 轮询接收（关键：OVRDIS + 主循环读 RDR）
Core/Src/status_led.c      WS2812 SPI 驱动（不要动，见约束）
Core/Src/motor_3508.c      M3508/C620 驱动 + 掉线判断
Core/Src/can.c             FDCAN 收发封装（Can_Init / Can_Send）
Core/Src/fdcan.c           FDCAN3 初始化（PD12/PD13/AF2）★CAN 问题大概率在这
Core/Src/main.c            主循环：轮询RC + 轮询CAN + 灯 + CAN总线自动恢复
Core/Src/diag.c            诊断模式（引脚扫描/串口抓包/三通道CAN测试）
firmware/*.hex             编好的固件
```

### 6.1 功能开关（`chassis_config.h`）

| 宏 | 现在 | 作用 |
|---|---|---|
| `CHASSIS_SWA_LED_TEST` | 1 | SWA 一档灭、另一档亮（白） |
| `CHASSIS_LED_TEST` | 0 | 1 = 红绿蓝白循环（灯自测，会覆盖 SWA 逻辑） |
| `CHASSIS_REQUIRE_ARM` | 0 | 1 = 必须拨 SWA 才出力 |
| `CHASSIS_SELFTEST_ON_BOOT` | 0 | 单轮点动自检 |
| `CHASSIS_ID_PROBE_ON_BOOT` | 0 | 电机 ID 探测（灯闪 N 下） |
| `CHASSIS_DIAG_MODE` | 0 | 诊断模式（**不跑灯**） |

---

## 7. 硬约束（用户明确要求过）

1. **不要动灯的逻辑和驱动**。灯已经能工作（颜色循环验证过），之前的反复折腾已经让用户很不满。
2. 如果必须烧 `CHASSIS_DIAG_MODE=1` 的诊断固件，**提前告知用户"灯会不工作"**，测完立刻烧回应用固件。
3. 改参数只改 `chassis_config.h`，不要散落在代码里。
4. 烧录后如果程序不跑（灯不亮、变量不动），先做**软复位**（`flash.ps1` 已内置）。

---

## 8. 用户侧已知信息

- 遥控器：FS-i6X（输出模式 `PPM and SBUS`），接收机 FS-iA10B（灯常亮 = 已对码）
- 电调：C620 ×4，**灯闪 1/2/3/4 = ID 1~4 已设好**，24V 从集线板供电
- 集线板：XT30 进、四路出，四块电调插在上面（**怀疑它没把 CAN 传下去**）
- 用户会用"灯"来判断状态，所以灯的语义要稳定

---

## 9. 相关文档

- `README_交接文档.md`：同一份内容，叙事版（含更多过程细节）
- `README_上手说明.md`：最初写的使用说明（部分内容已过时，以本文档为准）
