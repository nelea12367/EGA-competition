# DM-MC02 整车控制上手说明

本固件在原有四麦轮底盘基础上增加：DM430 步进大臂、ID 5 C610/2006 腕部、DS3218MG 小臂舵机、爪子舵机。**首次调试必须让车轮悬空，机械臂附近不得站人。**

## 1. 默认信号分配

| 功能 | DM-MC02 信号 | 外接设备 |
|---|---|---|
| 大臂步进脉冲 | PA02 / TIM2_CH3（开漏） | PA02 三针口信号脚 -> DM430 `PUL-` |
| 大臂方向 | PA00（开漏） | PA00 三针口信号脚 -> DM430 `DIR-` |
| DM430 信号正端 | 两个三针口的板载 5V（PC15 使能） | PA02 三针口 5V -> `PUL+`；PA00 三针口 5V -> `DIR+` |
| 大臂使能 | PE14 | DM430 `ENA+`、`ENA-` 暂时悬空 |
| 小臂 PWM | PE09 / TIM1_CH1 | 右上角第二组 PWM 三针座的信号脚（DS3218MG） |
| 爪子 PWM | PE13 / TIM1_CH3 | 右上角第一组 PWM 三针座的信号脚（爪子舵机） |
| 腕部 | CAN ID 5 | C610 + 2006，控制帧 `0x1FF` |

舵机 5--6V 供电地必须与 STM32 GND 共地；**舵机电源不得由 STM32 的 3.3V 或 GPIO 供给。**

## 2. 临时 DM430 直连

当前固件采用板载 5V 共阳极、STM32 开漏下拉方案：

```text
PA02 三针口 5V -> DM430 PUL+
PA02 三针口信号 -> DM430 PUL-
PA00 三针口 5V -> DM430 DIR+
PA00 三针口信号 -> DM430 DIR-
ENA+、ENA-  -> 悬空
```

PC15 默认按高电平使能板载 5V；上电后必须先用万用表确认 PWM 三针座 5V 对 GND 读数为约 5V。若没有 5V，将 `VEHICLE_BOARD_5V_ENABLE_ACTIVE_HIGH` 改为 `0` 后重新编译，切勿试接 24V 到 PUL+/DIR+。DM430 的电机线接 A+/A-/B+/B-；供电范围 12--32V。先按步进电机铭牌设置 DM430 工作电流和细分，首测使用较低电流。固件将 PUL 频率暂限为 1kHz。

6N137 可在后续需要额外隔离时再接入；恢复时需将 `VEHICLE_STEPPER_DIRECT_DM430` 设为 `0`，并按模块端子图重新接线。

## 3. 遥控逻辑

`Core/Inc/vehicle_config.h` 是唯一需要按实际遥控器和机构修改的配置文件。

| 条件 | 行为 |
|---|---|
| CH7(SWA) 未到使能档 | 所有车轮电流归零、步进停止、2006 电流归零 |
| CH8(SWB) 上档 | 高速底盘模式：底盘按 `CHASSIS_FAST_SCALE` 倍速运行 |
| CH8(SWB) 下档 | 底盘强制停止，进入机械臂模式 |
| 机械臂档 + 使能 + 三个机械臂摇杆回中 500ms | 将人工摆好的收臂姿态记为软件零点 |
| 软件零点成功后 | CH3 控大臂、CH4 控小臂（PWM 舵机）、CH1 控腕部（C610/2006）、CH9 控爪子张/停/合 |

无原点开关时，断电、堵转、丢步或手动掰动机械臂后，必须重新人工摆回统一收臂姿态，再进机械臂模式完成 500ms 参考确认。

## 4. 上电顺序

1. 不接 DM430/C610/舵机，只烧录并测 PE09/PE13 的 50Hz PWM、PA02 的低频脉冲。
2. 单独接 DM430，机械臂脱开负载或留足空间，低电流验证 CH3 两个方向。
3. 单独接两路舵机，先把 `vehicle_config.h` 的脉宽窗口调到小臂实测安全范围。
4. 只接 ID 5 的 C610/2006，确认收到 `0x205` 反馈后再做腕部小电流测试。
5. 车轮悬空时验证底盘 ID 1--4 与腕部 ID 5 同总线工作。
6. 全部通过后才允许低速落地测试。

## 5. 构建

```powershell
powershell -ExecutionPolicy Bypass -File .\build.ps1 `
  -ToolchainBin 'D:\stm32cubeide\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin' `
  -OutName DM_MC02_Vehicle
```

产物为 `build\DM_MC02_Vehicle.hex`。构建成功不表示接线与机构方向已经安全；必须按第 4 节逐项台架验证。
