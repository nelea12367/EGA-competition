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
| v9 | 摇杆中点自动校准；动作码每位锁存；死区重铺满量程；删掉"三杆回中"门槛（大臂自动建零）；掉遥控保留零点；0x200 从 1 kHz 降到 100 Hz |
| v8 | 四模式改为小臂×腕部 2×2；大臂退出预设；爪子摇杆优先于 SWC |
| v7 | 首次实现 SWA+SWD 四位动作码 |
| v6 | 修正步进限速 1000→350 Hz、舵机脉宽 500/2500→1100/1900 µs |
