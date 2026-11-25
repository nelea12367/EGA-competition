# EGA-competition · DM-MC02 麦轮底盘 + 机械臂固件

STM32H723VGT6 / STM32 HAL / FDCAN 1 Mbps

四麦轮底盘 + 四自由度机械臂（DM430 步进大臂、DS3218 小臂、C610+M2006 腕部、舵机爪子）。

---

## 1. 快速开始

```powershell
# 编译（默认输出 build\DM_MC02_Chassis.hex）
powershell -ExecutionPolicy Bypass -File .\build.ps1 -OutName DM_MC02_Vehicle

# 烧录（PWLINK2 / CMSIS-DAP 或 ST-Link）
powershell -ExecutionPolicy Bypass -File .\flash.ps1 -Hex .\build\DM_MC02_Vehicle.hex
```

- 工具链：STM32CubeIDE 自带的 `arm-none-eabi-gcc`（脚本会自动找，也可设 `GCC_BIN`）。
- 所有 `.ps1` 必须是纯 ASCII：Windows PowerShell 5.1 无 BOM 时按 ANSI 读，中文会语法报错。

### 1.1 烧录方法 B：STM32CubeProgrammer CLI（本机实测最可靠）

`flash.ps1` 走的是 OpenOCD。**如果 OpenOCD 打不开 ST-Link**（报 `open failed` /
`Infinite eval recursion`），用 CubeIDE 自带的命令行工具，一条命令搞定：

```powershell
$cli = "E:\CubeIDE\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe"

# 先确认能连上（会打印 ST-LINK SN / 电压 / 芯片型号）
& $cli -c port=SWD freq=1000

# 烧录 + 校验 + 复位
& $cli -c port=SWD freq=1000 -w build\DM_MC02_Vehicle.hex -v -rst
```

看到 `Download verified successfully` + `Software reset is performed` 就是成功。

> ⚠ **烧录前必须先关掉 STM32CubeIDE**：它会独占 ST-Link，此时 OpenOCD 和
> CubeProgrammer 都会报打不开设备（`open failed`）。

---

## 2. 硬件

| 项目 | 型号 / 参数 |
|---|---|
| 主控 | 达妙 DM-MC02，STM32H723VGT6，1MB Flash，AXI SRAM @0x24000000 |
| 遥控 | FS-i6X + FS-iA10B，**SBUS**（100k / 8E2），`RX Setup → Output mode` 必须是 `PPM and SBUS` |
| 底盘电机 | 4× M3508 + C620，反馈 `0x201~0x204`，控制帧 `0x200` |
| 大臂 | DM430 步进，`PA02` = TIM2_CH3 脉冲（开漏）、`PA00` = 方向（开漏） |
| 小臂 | DS3218MG，`PE09` = TIM1_CH1，50 Hz 脉宽 |
| 腕部 | C610 + M2006，ID 5，反馈 `0x205`，控制帧 `0x1FF` |
| 爪子 | 舵机，`PE13` = TIM1_CH3，50 Hz 脉宽 |
| 状态灯 | WS2812B（GRB），`PA07`，DWT bit-bang |
| CAN | FDCAN1 `PD0/PD1`（默认），FDCAN2 `PB5/PB6`，FDCAN3 `PD12/PD13` |

> 舵机必须**外部 5–6 V 供电并与主板共地**，板载三针座的 5V 带不动舵机。

### 2.1 硬件白名单（这套车上实测跑通过的组合）

| 部位 | 允许使用 | 实测状态 |
|---|---|---|
| 主控 | 达妙 DM-MC02 / STM32H723VGT6（HSE 24 MHz → 264 MHz） | ✅ 底盘、机械臂、CAN 全通 |
| 调试器 | **ST-Link**（本次用 V2J37S7，SWD 4 线）/ PWLINK2（CMSIS-DAP） | ✅ 可烧可读 |
| 烧录工具 | STM32CubeProgrammer CLI（CubeIDE 自带） | ✅ 唯一在本机稳定可用的路径 |
| 遥控 | FS-i6X + FS-iA10B，**SBUS 100k/8E2**，`RX Setup → PPM and SBUS` | ✅ 通道 1~10 全可读 |
| 底盘 | 4 × M3508 + 4 × C620，1 Mbps，ID 1~4，`0x200` / `0x201~0x204` | ✅ 四轮可驱动 |
| 腕部 | C610 + M2006，**ID 5**，`0x1FF` / `0x205` | ✅ 位置环可用 |
| 大臂 | 42 步进 + DM430，`PA02`=脉冲 / `PA00`=方向（共阳，PUL+/DIR+ 接板载 5V） | ✅ 有保持力矩、可走 |
| 小臂 / 爪 | DS3218MG（`PE09`）/ 爪舵机（`PE13`），50 Hz | ⚠ 信号已验证（脉宽 1500→2500 µs 实测在变），**必须外接 5–6 V 且共地** |
| 电源 | TB48S 6S；舵机 5–6 V 走外接降压板 | ✅ 底盘/腕部 24 V 正常 |
| 状态灯 | WS2812B（GRB），`PA07`，DWT bit-bang | ✅ |
| 电池架 | 四螺丝刚性固定在底盘主梁（规则要求） | — |

**不要用 / 不要做**

| 禁止 | 原因 |
|---|---|
| 24 V 接到 `PUL+ / DIR+ / ENA` | 直接烧 DM430 |
| 用板载 3.3 V 或 GPIO 给舵机供电 | 带不动，还会把 MCU 拉复位 |
| 舵机信号不共地（只接信号线） | 舵机会“松”、完全不动（本次踩过） |
| 同一个 C610 同时插 PWM 口和 CAN 口 | 手册明确会失控 |
| 非组委会分发的遥控器上场比赛 | 规则判罚 |
| 钢板 / 钢管做结构 | 规则禁止 |
| 指望软件控制 DM430 使能 | `ENA` 物理悬空，`VEHICLE_STEPPER_USE_ENA = 0` |

---

## 3. 操作说明

### 3.1 模式

| 开关 | 位置 | 行为 |
|---|---|---|
| **SWB** (CH8) | 上 | 底盘模式：两杆控制麦轮，机械臂就地抱死 |
| **SWB** (CH8) | 下 | 机械臂模式：四轮电流强制归零 |

### 3.2 机械臂模式下的通道

| 通道 | 输入 | 作用 |
|---|---|---|
| CH3 | 左杆上下 | 大臂（步进，速度控制） |
| CH4 | 左杆左右 | 小臂（舵机，脉宽斜率） |
| CH1 | 右杆左右 | 腕部（C610 位置环） |
| CH2 | 右杆上下 | 爪子（手动微调，优先级高于 SWC） |
| SWC (CH9) | 上 / 中 / 下 | 爪子 张开 / 保持 / 合拢 |
| SWA (CH7) | 上 / 下 | 动作码 bit0 |
| SWD (CH10) | 上 / 下 | 动作码 bit1 |

### 3.3 四个预设动作（小臂 × 腕部）

| 码 | SWA | SWD | 小臂 | 腕部 |
|---|---|---|---|---|
| 0 | 上 | 上 | A | A |
| 1 | 下 | 上 | B | A |
| 2 | 上 | 下 | A | B |
| 3 | 下 | 下 | B | B |

- 拨到某个组合并稳定 200 ms 后，小臂和腕部一起走到对应位置并保持。
- **大臂和爪子不参与预设**，只跟摇杆。
- 推 CH1 / CH4 超过死区 → 立刻退出预设转手动；把 SWA/SWD 拨到别的组合再拨回来可重新执行。
- 目标值在 `Core/Inc/vehicle_config.h` 的 `VEHICLE_POSE_TABLE`，**当前是占位值，必须实测替换**。

### 3.4 零点与安全

- 大臂、腕部的零点在**第一次进入机械臂模式时自动建立**，不需要人工回中。
- 掉遥控（>120 ms 无有效帧）：所有输出归零，但**零点保留**。
- 上电前 800 ms 自动校准摇杆中点（只校准没被推动的 CH1~CH4）。

---

## 4. 状态灯

| 灯 | 含义 |
|---|---|
| 红慢闪 (200 ms) | 遥控丢失 |
| 白慢闪 (400 ms) | 等摇杆回中（底盘模式） |
| 紫红快闪 (100 ms) | 电机掉线 / 腕部 C610 不在线 |
| 橙常亮 | 未使能（等回中或未 armed） |
| 绿慢闪 (500 ms) | 就绪 |

---

## 5. 目录结构

```
Core/Inc/chassis_config.h   底盘全部参数（几何、限速、PID、通道、自检开关）
Core/Inc/vehicle_config.h   ★机械臂全部参数（通道、预设姿态、限速、PID）
Core/Src/chassis.c          麦轮解算 + 速度环 + 安全状态机
Core/Src/vehicle_control.c  ★模式路由：底盘 ↔ 机械臂、动作码、预设、手动优先
Core/Src/arm_stepper.c      DM430 步进（软停、保力矩、行程保护）
Core/Src/arm_2006.c         C610/2006 位置环（PD + 死区 + 电流斜率）
Core/Src/servo_pair.c       两路舵机 PWM + 斜率控制
Core/Src/rc_input.c         SBUS 接收 + 自动校准中点 + 一阶低通
Core/Src/status_led.c       WS2812 bit-bang 驱动
Core/Src/can.c              FDCAN 收发封装
tests/host/                 host 端单元测试（gcc 编译，run_tests.ps1）
```

---

## 6. 文档

| 文档 | 内容 |
|---|---|
| `README_机械臂控制说明.md` | ★机械臂唯一事实来源：通道、状态机、故障对照、SWD 调试速查 |
| `README_整车控制上手.md` | 整车接线与调试顺序 |
| `README_交接文档.md` | 底盘历史（CAN/遥控/灯踩过的坑） |
| `README_机械臂抖动排查.md` | 抖动/下坠整定记录 |
| `README_CAN排查_最新.md` | CAN 通道排查 |
| `README_LED修复记录.md` | WS2812 驱动修复记录 |

---

## 7. 测试

```powershell
powershell -ExecutionPolicy Bypass -File .\tests\host\run_tests.ps1
```

覆盖：麦轮解算、关节路由、步进引脚、安全配置、腕部 CAN 独立性。

---

## 8. 版本

| 版本 | 内容 |
|---|---|
| v10 | 大臂 350→700 Hz（加速 8000 Hz/s）；腕部位置死区 0.008→0.002、速度反馈低通、0x1FF 提至 500 Hz 且**发送失败自动重试** |
| v9 | 摇杆中点自动校准；动作码每位锁存；死区重铺满量程；删掉"三杆回中"门槛（大臂自动建零）；掉遥控保留零点；0x200 从 1 kHz 降到 100 Hz |
| v8 | 四模式改为小臂×腕部 2×2；大臂退出预设；爪子摇杆优先于 SWC |
| v7 | 首次实现 SWA+SWD 四位动作码 |
| v6 | 修正步进限速 1000→350 Hz、舵机脉宽 500/2500→1100/1900 µs |

---

## 9. 高频故障与排查（血泪清单）

> 这一节是 2026-11-25 一整晚现场调试的结论，按“现象 → 根因 → 动作”写。
> 机械臂各轴的详细对照表见 `README_机械臂控制说明.md` 第 6 节。

### 9.1 先看灯，灯是第一个诊断口

| 现象 | 根因 | 动作 |
|---|---|---|
| **灯常亮一色、完全不闪** | **固件卡死**（WS2812 会锁存最后一帧，不是灯坏） | 断电重启；反复出现就用 SWD 读 PC/CFSR（见 9.3） |
| 红慢闪 | 收不到遥控 | 查发射机开机、接收机常亮、输出模式 SBUS |
| 紫红快闪 | 电机掉线 / 腕部 C610 不在线 | 查 24 V、CAN 线、电调 ID |
| 橙常亮 | 未使能（等摇杆回中 / 未 armed） | 摇杆回中 200 ms |
| 绿慢闪 | 就绪 | — |

### 9.2 底盘能开、机械臂完全没反应（踩过最久的一个）

- **根因**：旧版把 **SWA(CH7) 当成机械臂总使能**（`enabled = link_ok && CH7==UP`），
  而底盘不检查 SWA（`CHASSIS_REQUIRE_ARM = 0`）。于是 SWA 不在 UP 位时：
  底盘照跑，机械臂四个轴被 `ForceStop()` 全灭——连不依赖 CAN 的舵机也不动。
- **现在**：v10 已删除独立总使能，只按 SWB 分模式。若还遇到，先确认
  `vehicle_control.c` 里 `enabled` 只看 `Rc_IsSignalValid()`。

### 9.3 烧录 / 调试器

| 现象 | 根因 | 动作 |
|---|---|---|
| `open failed` | **STM32CubeIDE 正开着**，独占 ST-Link | 关掉 IDE（含后台进程）再烧 |
| OpenOCD `Infinite eval recursion` | 本机 OpenOCD 与 `interface/stlink.cfg` 不兼容 | 用第 1.1 节的 CubeProgrammer CLI，或 `.embeddedskills/openocd/dm_stlink.cfg` |
| 烧完程序不跑 | 复位残留 halt | `-rst`，或写 `AIRCR = 0x05FA0004` 软复位 |

**不改代码读实时状态**（hotplug 不复位，可在跑的时候读）：

```powershell
$cli -c port=SWD freq=1000 mode=hotplug -r32 0x24000684 8
```

常用地址：`g_fw_version`、`vehicle s_state`（5 字节：mode/enabled/stepper_ref/wrist_ref/wrist_online）、
`chassis s_state`、`arm_stepper s_state`、`arm_2006 s_state`、`s_data`（遥控）。
地址用 `arm-none-eabi-nm -S build\DM_MC02_Vehicle.elf` 查。

### 9.4 执行器

| 现象 | 根因 | 动作 |
|---|---|---|
| **小臂、爪子同时不动，且“松”** | 舵机**地和主板没共地**（只接了信号线） | 降压板输出负极 → 主板任意 GND |
| 小臂/爪子单个不动 | 信号线不在 `PE09` / `PE13` | 量该脚对 GND：1500 µs@50 Hz ≈ **0.25 V** 平均 |
| 大臂**很硬**但推杆不走 | 使能和力矩都在，问题在**脉冲/方向线** | 量 `PA02` 推 CH3 时应有平均电压变化；`PA00` 推上/推下应 5 V/0 V |
| 大臂**很软** | 驱动器脱机（`ENA` 悬空默认脱机） | `ENA+`→5 V、`ENA−`→GND 永久使能 |
| 大臂只能往下、往上丢步 | 力矩不够 | 调大 DM430 电流拨码 / 改细分 / 降 `VEHICLE_STEPPER_MAX_HZ` |
| 腕部没反应、底盘正常 | **`0x200` 每圈都发把 1 Mbps 总线占满**（TX FIFO 只有 8 格），`0x1FF` 被挤掉 | v9 起停车帧改 100 Hz；不要在主循环里无节流发帧 |

### 9.5 零点与预设

- **v7 及更早**要求“三根杆同时回中 500 ms”才建零点，没做这个动作 → **CH3 永久失效**，
  表现为“摇杆完全没用”。**v9 起改为进入机械臂模式自动建零**，不需要人工回中。
- 掉遥控（>120 ms）只停输出、**保留零点**，恢复后不用重新回中。
- 四个预设动作只作用于**小臂 + 腕部**，大臂和爪子只跟摇杆。
- 拨了 SWA/SWD 没反应 → `VEHICLE_POSE_TABLE` 还是占位值，必须实测替换。

### 9.6 通道号对不上时（最快的一次性判定）

```powershell
powershell -ExecutionPolicy Bypass -File .\build.ps1 -Defines "-DVEHICLE_CHANNEL_PROBE=1" -OutName DM_MC02_ChannelProbe
```

烧进去后灯变成“数闪烁”：红灯闪 N 下 = 第 N 路推到正向底、绿灯闪 N 下 = 反向底、
暗蓝常亮 = 全在中位。一根一根推，记下通道号，再对照 `vehicle_config.h` 改。

