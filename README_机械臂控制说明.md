# 机械臂控制说明（v8）

> 这一页是机械臂部分的**唯一事实来源**。改代码前先看这里，改完记得同步更新。
> 底盘参数仍看 `Core/Inc/chassis_config.h`，机械臂参数全部在 `Core/Inc/vehicle_config.h`。

---

## 1. 硬件对应关系

| 关节 | 执行器 | 信号 | 代码位置 |
|---|---|---|---|
| 大臂 | DM430 步进 | `PA02` = TIM2_CH3 脉冲（开漏）、`PA00` = 方向（开漏） | `arm_stepper.c` |
| 小臂 | DS3218MG 舵机 | `PE09` = TIM1_CH1，50Hz 脉宽 | `servo_pair.c` |
| 腕部 | C610 + M2006 | CAN `0x1FF` 发、`0x205` 收，ID 5 | `arm_2006.c` |
| 爪子 | 舵机 | `PE13` = TIM1_CH3，50Hz 脉宽 | `servo_pair.c` |

- DM430 的 `PUL+`/`DIR+` 接板载 5V（`PC15` 使能），`PUL-`/`DIR-` 接信号脚。
- **两个舵机的 5–6V 电源必须外部供给，并且要和 STM32 共地。** 板子只出信号。
- `ENA` 目前悬空，软件不控制使能（`VEHICLE_STEPPER_USE_ENA = 0`）。

---

## 2. 遥控通道

| 通道 | 开关/摇杆 | 作用 |
|---|---|---|
| CH1 | 右杆左右 | 腕部 |
| CH2 | 右杆上下 | 爪子（SWC 在"保持"档时才生效） |
| CH3 | 左杆上下 | 大臂 |
| CH4 | 左杆左右 | 小臂 |
| CH7 | SWA | 动作码 bit0 |
| CH8 | SWB | **上 = 底盘模式，下 = 机械臂模式** |
| CH9 | SWC | 上 = 爪子张开，中 = 保持，下 = 合拢 |
| CH10 | SWD | 动作码 bit1 |

- 接收机必须是 `PPM and SBUS`（100k / 8E2）。
- 归一化：`(raw - 992) / 820`，死区 ±0.08，过了死区重新铺满 0~1（不会跳变）。
- 如果某个开关方向反了，改 `vehicle_config.h` 里那一路的 `*_LEVEL`。

---

## 3. 模式

```
SWB 上  → 底盘模式
          三个摇杆控制底盘；SWA/SWD 切换大臂、小臂、腕部预设
SWB 下  → 机械臂模式
          四轮电流强制归零；四关节只由上面的摇杆通道手动控制
```

- **没有独立的总使能开关**。SWB 上就是安全态。
- 掉遥控（>120ms 无有效帧）：所有输出立即归零，但**零点保留**，重新连上不用回中。

---

## 4. 四个预设动作（大臂 × 小臂 × 腕部）

| 码 | SWA | SWD | 大臂 | 小臂 | 腕部 |
|---|---|---|---|---|---|
| 0 | 上 | 上 | 预设 0 | A | A |
| 1 | 下 | 上 | 预设 0 | B | A |
| 2 | 上 | 下 | 预设 0 | A | B |
| 3 | 下 | 下 | 预设 0 | B | B |

- 预设仅在**底盘模式**下解释：切入该模式时当前组合只作基线，之后 SWA/SWD 拨到任一新组合时，**大臂、小臂和腕部**立即开始前往对应位置；底盘驾驶不中断。
- **爪子不参加预设**，始终只跟 SWC/CH2。机械臂模式完全忽略 SWA/SWD，CH3/CH4/CH1/CH2 只作手动控制，因此不会与预设冲突。
- 每个 bit 独立锁存：开关经过中间位时保持上一次的值，不会误触发别的组合。
- 上电第一次读到的开关位置只当基线，不会触发动作。
- 目标值在 `vehicle_config.h` 的 `VEHICLE_POSE_TABLE`：`{大臂脉冲, 小臂脉宽us, 腕部弧度}`，**当前是占位值**。

---

## 5. 零点（重要，以前这里是最大的坑）

| 关节 | 零点怎么来 | 需不需要人工回中 |
|---|---|---|
| 大臂 | 第一次收到有效遥控时，以当前位置为 0 | **不需要** |
| 腕部 | C610 在线后，以当前角度为 0 | **不需要** |

- 大臂的零点用于 `±VEHICLE_STEPPER_MAX_PULSE`（±12000 脉冲）的行程保护，也是预设大臂脉冲位置的相对原点。
- 零点只在**上电**时建立；模式切换、掉遥控都不会清掉。
- v7 及以前要求"三根杆同时回中 500ms"才建零，没做过这个动作会导致 **CH3 永久失效**——这就是"摇杆完全没用"的根因，v8 已删除。

---

## 6. 故障对照表

| 现象 | 最可能的原因 | 怎么查 |
|---|---|---|
| **所有关节都不动** | 遥控失效 | 看灯：红慢闪 = 没遥控。检查发射机开机、接收机灯常亮、输出模式 SBUS |
| 大臂不动 | ① 脉冲/方向没接对 ② DM430 没上电/使能 ③ 电流拨码太小 | 量 `PA02` 应有脉冲（推 CH3 时约 2.5V 平均）；量 `PA00` 推上/推下应 5V/0V 变化 |
| **大臂只能往下、不能往上** | **力矩不够**（往下有重力帮忙） | 把 DM430 工作电流拨码按电机铭牌调大；或把细分调大；或降 `VEHICLE_STEPPER_MAX_HZ` |
| 大臂一推就"啪"掉一截 | 丢步 | 降 `VEHICLE_STEPPER_MAX_HZ`（350 → 250 → 150 逐档试） |
| 小臂不动 | ① 舵机电源没接 ② 信号线不在 PE09 | 量 `PE09` 对 GND 应约 **0.25V**（1500µs/50Hz×3.3V） |
| 爪子不动 | ① 舵机电源没接 ② 信号线不在 PE13 | 量 `PE13` 对 GND 应约 **0.24V**（1425µs） |
| **小臂和爪子同时都不动** | 几乎一定是**舵机 5–6V 电源**问题（两路共用 TIM1） | 量舵机电源对 GND；确认和主板共地 |
| 腕部不动 | ① C610 没上线 ② ID 不是 5 | 灯紫红快闪 = 腕部掉线；量 CAN_H/CAN_L 2.5V、60Ω |
| 腕部一直抖 | 位置环增益大 / 齿隙 | 降 `VEHICLE_WRIST_KP`、加大 `VEHICLE_WRIST_POS_DEADBAND` |
| 爪子方向反了 | `VEHICLE_GRIPPER_OPEN_LEVEL` 弄反 | 上/下档对调 |
| 四个模式看起来没反应 | 目标值还是占位值 | 改 `VEHICLE_POSE_TABLE` |

---

## 7. 用 SWD 读实时状态（不用改代码）

```powershell
# 1) 从 elf 拿地址
arm-none-eabi-nm -S build\DM_MC02_Vehicle.elf | findstr "g_fw_version s_state s_data s_action"

# 2) 用 CubeProgrammer 读（hotplug 不会复位，可以在跑的时候看）
STM32_Programmer_CLI.exe -c port=SWD freq=500 mode=HOTPLUG -r32 0x24000684 8
```

| 变量 | 含义 |
|---|---|
| `g_fw_version` | 固件版本号，每次烧录递增 |
| `vehicle s_state` | 5 字节：`mode, enabled, stepper_ref, wrist_ref, wrist_online` |
| `chassis s_state` | `rc_ok, motors_ok, armed, wait_center, ..., probe_count, missing_mask` |
| `arm_stepper s_state` | `position_pulse(4), command_hz(4), reference_valid, running, direction_positive` |
| `arm_2006 s_state` | `feedback_seen, reference_valid, temp, angle_rad, target_rad, velocity, current` |
| `servo s_state` | `lower_arm_command_us, gripper_deg, lower_arm_us, gripper_us` |
| `rc s_data` | `analog[16](64B), raw[16](32B), switch_pos[16](16B), ...`，`switch_pos` 后面是 `channel_count, protocol, frame_valid, failsafe` |
| `s_action_code` | 当前生效的动作码 0~3 |

> `switch_pos` 的值：`0 = 上`、`1 = 中`、`2 = 下`。

---

## 8. 通道自检固件（怀疑通道号对不上时用）

```powershell
powershell -ExecutionPolicy Bypass -File .\build.ps1 -Defines "-DVEHICLE_CHANNEL_PROBE=1" -OutName DM_MC02_ChannelProbe
```

烧进去以后灯不再显示状态，改成"数闪烁"：

- **红灯闪 N 下** = 第 N 路正在正向推到底
- **绿灯闪 N 下** = 第 N 路正在反向推到底
- **暗蓝常亮** = 所有通道都在中位

用法：一根一根来，把某个摇杆推到顶保持住，数闪几下，就知道它对应 CH 几；把 SWA/SWD/SWC 各拨一次同理。记完一张表，对照第 2 节改 `vehicle_config.h` 的通道号。

---

## 9. 版本记录

| 版本 | 改动 |
|---|---|
| v5 | 底盘 + 机械臂基础版（SWA 当总使能，三杆回中建零） |
| v6 | 修了两处与 README 结论冲突的配置（步进 1000→350Hz、舵机 500/2500→1100/1900µs） |
| v6b | SWA+SWD 四位动作码（含大臂）、爪子加 CH2 摇杆、去掉总使能 |
| v7 | 四模式改成小臂×腕部 2×2；大臂退出预设；爪子摇杆优先于 SWC |
| **v8** | **删除"三杆回中"门槛（大臂/腕部自动建零）；动作码每位独立锁存；死区重铺满量程；掉遥控保留零点；0x200 从 1kHz 降到 100Hz；灯态改用真实链路状态；新增通道自检固件** |
