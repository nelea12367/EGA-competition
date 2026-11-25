# 自己动手：编译 / 烧录 / 调试 / 改参数 / 推代码

> 工程目录：`C:\Users\gaozh\Documents\EGA-competition`
> 所有 `.ps1` 必须是纯 ASCII（Windows PowerShell 5.1 按 ANSI 读，中文会语法报错）。

---

## 1. 打开终端

`Win + X` → **终端**，然后：

```powershell
cd C:\Users\gaozh\Documents\EGA-competition
```

---

## 2. 编译

```powershell
powershell -ExecutionPolicy Bypass -File .\build.ps1 -OutName DM_MC02_Vehicle
```

- 产物：`build\DM_MC02_Vehicle.hex`
- 看到 `OK -> build\DM_MC02_Vehicle.elf / .hex / .bin` 就是成功
- `[WARN]` 不影响使用，只有 `[FAIL]` / `error:` 才是失败
- 提示找不到 `arm-none-eabi-gcc` 时，手动指定工具链：

```powershell
powershell -ExecutionPolicy Bypass -File .\build.ps1 -OutName DM_MC02_Vehicle `
  -ToolchainBin "C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin"
```

## 3. 跑测试（改过代码就建议跑一次）

```powershell
powershell -ExecutionPolicy Bypass -File .\tests\host\run_tests.ps1
```

要看到 5 个 `PASS`。任何一个 FAIL 就说明改坏了某个约束（关节路由、步进引脚、安全配置、腕部独立性）。

---

## 4. 烧录

### A. ST-Link（你现在插的那个）

```powershell
$cp = "C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe"
& $cp -c port=SWD freq=500 -e all -w .\build\DM_MC02_Vehicle.hex -v -rst
```

最后出现 `Download verified successfully` + `MCU Reset` 就是成功。

### B. PWLINK2 / DAP-Link（CMSIS-DAP）

```powershell
powershell -ExecutionPolicy Bypass -File .\flash.ps1 -Hex .\build\DM_MC02_Vehicle.hex
```

### 报 `No debug probe detected` 怎么办

1. 拔掉重插探针（这个 ST-Link 接触不太好，经常第一次连不上）
2. 换一个 USB 口，别插扩展坞
3. 关掉 STM32CubeIDE —— 它会独占探针
4. 设备管理器确认能看到：
   - ST-Link：`STM32 STLink`（VID_0483&PID_3748）
   - PWLINK2：`USB Composite Device`（VID_0D28&PID_0204）
5. 直接重跑命令，CubeProgrammer 经常第二次就成功

---

## 5. 确认烧进去的是哪一版

```powershell
$nm = "C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin\arm-none-eabi-nm.exe"
& $nm -S .\build\DM_MC02_Vehicle.elf | findstr "g_fw_version"
& $cp -c port=SWD freq=500 mode=HOTPLUG -r32 0x24000008 4
```

期望 `0x0000000A`（v10）。

> `mode=HOTPLUG` = **不复位**，看的是运行中的值；不加这个参数会复位板子。

---

## 6. 不用万用表看实时状态

```powershell
& $nm -S .\build\DM_MC02_Vehicle.elf | findstr "s_state s_data s_action"
```

| 变量 | 长度 | 含义 |
|---|---|---|
| `g_fw_version` | 4 | 固件版本 |
| `vehicle s_state` | 5 | `mode, enabled, stepper_ref, wrist_ref, wrist_online` |
| `chassis s_state` | 64 | `rc_ok, motors_ok, armed, wait_center, ..., missing_mask` |
| `arm_stepper s_state` | 12 | `position_pulse(4), command_hz(4), ref, running, dir` |
| `arm_2006 s_state` | 24 | `feedback_seen, ref, temp, angle, target, velocity, current` |
| `servo s_state` | 12 | `lower_arm_command_us, gripper_deg, lower_arm_us, gripper_us` |
| `rc s_data` | 124 | `analog[16](64B), raw[16](32B), switch_pos[16](16B), channel_count, protocol, frame_valid, failsafe, last_frame_ms, frame_counter` |

读法：

```powershell
& $cp -c port=SWD freq=500 mode=HOTPLUG -r32 0x240006e4 8
```

（地址每次编译都会变，用 `nm` 查当前值。）

---

## 7. 改参数：只改 `Core/Inc/vehicle_config.h`

| 需求 | 改哪个宏 | 说明 |
|---|---|---|
| **大臂太慢** | `VEHICLE_STEPPER_MAX_HZ` | 现在 700。1/8 细分 ≈158°/s，1/16 细分 ≈79°/s |
| 大臂一推就"掉一截" | 同上，往下调 | 600 → 500 → 400 逐档退；或调大 DM430 工作电流拨码 |
| 大臂起停太猛 | `VEHICLE_STEPPER_ACCEL_HZ_S` / `DECEL` | 现在 8000 / 4000 |
| **腕部抖 / 卡顿** | `VEHICLE_WRIST_KP`、`POS_DEADBAND`、`KD` | KP 小一点更稳；DEADBAND 现在 0.002 |
| 腕部扛不住重力往下掉 | `VEHICLE_WRIST_GRAVITY_FF` | 试 ±150，正负看方向 |
| 腕部发烫/顶死 | `VEHICLE_WRIST_CURRENT_MAX` | 现在 2500 ≈ 3A |
| **四个模式没反应** | `VEHICLE_POSE_TABLE` | 每行 `{小臂脉宽us, 腕部弧度}`，现在是占位值 |
| 小臂/爪子方向反了 | `VEHICLE_GRIPPER_OPEN_LEVEL` / `CLOSE_LEVEL` | 对调即可 |
| 通道不对 | `VEHICLE_CH_*` | 用通道自检固件确认后改 |
| 摇杆死区太大/太小 | `VEHICLE_RC_DEADBAND` | 现在 0.08 |

改完必须**重新编译 + 烧录**才生效。

---

## 8. 推 GitHub

```powershell
cd C:\Users\gaozh\Documents\EGA-competition
git add -A
git commit -m "说明这次改了什么"
git push
```

- 报 `TLS connect error` / `schannel` → 用 `git config --global http.sslBackend openssl`
- 弹窗要账号密码 → 用户名填 GitHub 用户名，**密码填 Personal Access Token**（不是登录密码）
- 看不下去改了什么 → `git status` / `git diff`

---

## 9. 烧完必须确认的三件事

1. 灯不再白常亮 → 应该是**绿慢闪**（就绪）
2. `g_fw_version` 是本次的版本号
3. `mode=1` 时 `stepper_ref=1`、`wrist_ref=1`（零点已自动建立）

---

## 10. 常见故障对照

| 现象 | 先查什么 |
|---|---|
| 灯红慢闪 | 遥控丢了：发射机开机、接收机灯常亮、`Output mode = PPM and SBUS` |
| 灯白慢闪 | 等摇杆回中（底盘模式） |
| 灯紫红快闪 | 电机掉线 / 腕部 C610 不在线（查 CAN、ID 5） |
| 灯橙常亮 | 未使能 / 未 armed |
| 大臂不动 | `reference_valid` 是否为 1；PA02 有没有脉冲；DM430 24V 和使能 |
| 大臂只能往下 | 力矩不够（往下有重力帮忙），调大 DM430 电流拨码 |
| 小臂/爪子不动 | 舵机 5–6V 外部供电和共地；PE09/PE13 是否有波形 |
| 腕部不动 | C610 是否在线；CAN_H/CAN_L 2.5V、60Ω 终端电阻 |
| 轮子不动 | `motors_ok`、`missing_mask`；CAN 通道对不对 |
