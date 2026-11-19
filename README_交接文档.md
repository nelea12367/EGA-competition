# 麦轮底盘固件 · 交接文档

> ⚠ **2026-11-11 更正**：本文第 3.4 节"CAN 座接的是 FDCAN3"这个结论是**错的**。
> FDCAN3 的 TEC=0 只说明"它没发出去"（那条路上没有收发器），不说明它是电调那一路。
> 详见 **`README_CAN排查_最新.md`**，以那份为准。

> 日期：2026-11-10
> 工程：`dm_mc02_chassis/`（基于用户提供的 `DM_MC02_Test.zip` 改写）
> 硬件：达妙 DM-MC02（STM32H723VGT6） + FS-i6X + FS-iA10B + 4×M3508/C620

---

## 1. 一句话现状

**遥控链路已经完全打通**（SBUS，494+ 帧持续接收，摇杆/开关实时跟随）；
**灯已经修好**（驱动正常，SWA 可以控制亮灭）；
**CAN 还差最后一步**：已经定位到板子的 CAN 座接的是 **FDCAN3**（不是原例程假设的 FDCAN2），收发硬件回环自测通过，但 HAL 初始化在这个工程里没把发送缓冲区配上（`TXBC=0`），帧发不出去。

---

## 2. 已验证的硬件事实（重要，别再猜）

### 2.1 主控与外设

| 项目 | 事实 | 来源 |
|---|---|---|
| 主控 | STM32H723VGT6，1MB Flash，AXI SRAM 320KB | 板子丝印 / 官方资料 |
| 调试器 | **PWLINK2（CMSIS-DAP）**，VID_0D28 PID_0204 | USB 枚举 |
| CAN 座实际通道 | **FDCAN3：PD12=RX，PD13=TX（AF2）** | 三通道扫描实测：只有 FDCAN3 的 TEC=0 |
| CAN1/FDCAN1 | PD0=RX，PD1=TX | 官方例程 .ioc |
| CAN2/FDCAN2 | PB5=RX，PB6=TX | 官方例程 .ioc |
| 遥控输入 | UART5，PD2=RX，100000 8E2（SBUS） | 实测抓包 |
| 状态灯 | WS2812，SPI6：PA5=SCK、PA7=MOSI，2.148MHz，GRB 顺序 | 实测（红绿蓝循环测试通过） |
| 电机 | 4×M3508 + **C620**，电调灯闪 1/2/3/4 = ID 1~4 已设好 | 用户观察 |

> 官方资料库：`https://gitee.com/kit-miao/dm-mc02`（原理图、说明书、例程都在里面）

### 2.2 遥控器（FS-i6X + FS-iA10B）实测通道表

| 通道 | 用途 | 实测 |
|---|---|---|
| CH1 | 右摇杆左右 | 240~1807 变化 |
| CH2 | 右摇杆上下 | 240~1807 变化 |
| CH3 | 左摇杆上下 | 240~1807 变化 |
| CH4 | 左摇杆左右 | 240~1807 变化 |
| CH5/CH6 | 空/旋钮 | 恒定（不动） |
| **CH7** | **SWA（两段开关）** | 240 ↔ 1807 |
| CH8 | SWB | 240 ↔ 1807 |
| CH9/CH10 | SWC/SWD | 240 ↔ 1807 |

- 数值范围：中位 ≈1024，两端 240 / 1807
- **遥控器 `RX Setup → Output mode` 必须选 `PPM and SBUS`**（选 `PPM and iBUS` 时串口脚不出数据）

---

## 3. 遇到过并已解决的坑（按重要性）

### 3.1 原例程 `DM_MC02_Test` 一上电就 HardFault
- 现象：PC 停在 `HardFault_Handler`，CFSR=`UNDEFINSTR`，出错 PC 在 RAM
- 结论：**原例程在这块板子上根本跑不起来**，用户之前"灯亮但什么都不动"就是这个原因
- 处理：不用原例程，重写应用层

### 3.2 遥控收不到（折磨最久）
按顺序踩了三个坑：

1. **协议不对**：原例程只认 SBUS，而接收机可能被切成 iBUS/PWM
   → 现在固定 SBUS（100k/8E2/不反相），这是实测正确的组合
2. **串口溢出（ORE）**：一旦置位，后面的数据全堵死，表现为"收一会儿就死"
   → 解决办法：`UART_ADVFEATURE_RXOVERRUNDISABLE_INIT` + 开 `OVRDIS`
3. **中断收包不可靠** → 改成**主循环轮询** `RDR`（这个组合实测稳定）

> 抓包验证方法：把 UART 配成 100k 8E2，轮询读 `RDR`，前几个字节应该是 `0F 00 ...`（SBUS 帧头）

### 3.3 灯"能发不亮/颜色不跟"
- 现象：SPI 发送返回成功（计数在涨），但灯颜色不变
- 真正原因有两个：
  1. **HAL SPI 状态卡死**（一次失败后永久 BUSY）→ 加 DeInit/再 Init 兜底
  2. **颜色顺序**：这块板实测按 GRB 发是对的（按 RGB 发会把红送到绿灯通道）
- 现在：`status_led.c` 是 SPI 版 + 不做缓存（每 50ms 重发一帧，33 字节 = 123us）
- 验证方法：`CHASSIS_LED_TEST = 1` 时循环显示 红→绿→蓝→白→灭，实测能按顺序变

### 3.4 CAN 一直不通
- 原例程用 **FDCAN2（PB5/PB6）**，但板子的 CAN 座接的是 **FDCAN3（PD12/PD13）**
  → 三通道同时发帧扫描：FDCAN1/FDCAN2 的 TEC 涨到 248，**只有 FDCAN3 的 TEC=0**
- 改到 FDCAN3 后，**内部回环自测通过**（发 3 帧收到 3 帧，ID=0x201 正确）
- **最新状态（今晚最后一次测量）**：
  | 寄存器 | 值 | 含义 |
  |---|---|---|
  | CCCR | 0x1000 | 已启动（INIT=0） |
  | TXBC | 0x080020CC | TX FIFO 8 个缓冲已配好 |
  | **TXBRP** | **0x000000FF** | **8 帧全部"排队中"** |
  | **TXBTO** | **0x00000000** | **一帧都没真正发出去** |
  | ECR | 0x0000 | 没有任何错误（TEC=0） |
  | RXF0S | 0 | 没收到任何帧 |
  | PD12（CAN RX 脚） | 高电平 | 总线空闲，没被压死 |
- 结论：**控制器收下了发送请求，但永远不启动发送**，而且不报任何错误 → 高度怀疑
  **FDCAN3 的消息 RAM 基址/偏移没配对**（官方例程用 `MessageRAMOffset=0x812`，已照改，
  但现象不变，说明还有别的差异）

> ⚠ **寄存器偏移别搞错**（我今晚踩的坑）：这块芯片的 CMSIS 头里
> `RXF0S = 0xA4`、`TXBC = 0xC0`、`TXBRP = 0xCC`、`TXBTO = 0xD8`。
> 按 0xA4 去读 TXBC 会读到 RXF0S（永远是 0），会得出"发送缓冲没配上"的错误结论。

---

## 4. 代码结构（相对原例程的改动）

```
dm_mc02_chassis/
├─ Core/Inc/
│  ├─ chassis_config.h     ★新增：所有参数（几何/限速/PID/通道映射/功能开关）
│  ├─ chassis.h/.c         ★新增：麦轮解算 + 速度环 + 安全状态机 + ID探测/自检
│  ├─ rc_input.h/.c        ★新增：SBUS 轮询接收（替代 fxi6x.c）
│  ├─ status_led.h/.c      ★改写：SPI 驱动 + 无缓存 + 颜色表
│  ├─ diag.h/.c            ★新增：诊断模式（引脚扫描/串口抓包/回环自测）
│  ├─ motor_3508.h/.c      ○改：加掉线时间戳、温度、Motor3508_IsOnline()
│  ├─ dji_motor.h          ○改：结构体加 last_rx_ms / temperature
│  ├─ fdcan.h/.c           ○改：加 FDCAN3 初始化（PD12/PD13/AF2）
│  └─ main.c               ★重写：主循环 + 轮询收 RC + 轮询收 CAN + 灯逻辑
├─ firmware/               ★编好的 hex（直接烧）
└─ build.ps1               ★命令行编译脚本（用 CubeIDE 自带 gcc）
```

**删除的文件**：`fxi6x.c/h`（SBUS 解析旧实现）、`control_mixer.c/h`（旧的归一化解算）

### 4.1 `chassis_config.h` 关键参数

```c
CHASSIS_WHEEL_RADIUS      0.0762f   // 麦轮半径（⚠ 实测覆盖）
CHASSIS_HALF_WHEELBASE    0.250f    // 半轴距
CHASSIS_HALF_TRACK        0.250f    // 半轮距
CHASSIS_MOTOR_REVERSED_MASK 0x00    // 哪只电机装反了就置对应 bit
CHASSIS_MAX_VX/VY/WZ      1.5/1.0/3.0
CHASSIS_CURRENT_LIMIT     6000
CHASSIS_SPEED_KP/KI       600/400
RC_CH_FORWARD/STRAFE/ROTATE  1/0/3  // CH2/CH1/CH4
RC_CH_ARM                 6         // CH7 = SWA
RC_CH_MODE                7         // CH8 = SWB
RC_SIGN_FORWARD/STRAFE/ROTATE  +1/-1/-1
CHASSIS_REQUIRE_ARM       0         // 1 = 必须拨 SWA 才出力
CHASSIS_SWA_LED_TEST      1         // 1 = SWA 控灯亮灭（当前）
CHASSIS_LED_TEST          0         // 1 = 红绿蓝白循环自测
CHASSIS_SELFTEST_ON_BOOT  0         // 1 = 单轮点动自检（测轮序）
CHASSIS_ID_PROBE_ON_BOOT  0         // 1 = 电机 ID 探测（灯闪 N 下 = N 号）
CHASSIS_DIAG_MODE         0         // 1 = 诊断模式
```

---

## 5. 编译与烧录

### 5.1 命令行编译（推荐，已验证）

```powershell
cd dm_mc02_chassis
powershell -ExecutionPolicy Bypass -File .\build.ps1
# 产物：build\DM_MC02_Chassis.elf / .hex / .bin
```

工具链：`C:\ST\STM32CubeIDE_2.2.0\...\gnu-tools-for-stm32.14.3.rel1\...\bin`
（脚本会自动找；也可设环境变量 `GCC_BIN`）

### 5.2 烧录（用 OpenOCD + PWLINK2，已验证）

```powershell
$ocd="C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.openocd.win32_2.4.500.202604080855\tools\bin\openocd.exe"
$sc="C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.debug.openocd_2.3.400.202606220929\resources\openocd\st_scripts"

# 先软复位（清 lockup）
& $ocd -s $sc -f interface/cmsis-dap.cfg -c "transport select swd" -c "adapter speed 500" `
       -c "set AP_NUM 0" -f target/stm32h7x.cfg -c "init" -c "halt" `
       -c "mww 0xE000ED0C 0x05FA0004" -c "shutdown"

# 烧录
& $ocd -s $sc -f interface/cmsis-dap.cfg -c "transport select swd" -c "adapter speed 500" `
       -c "set AP_NUM 0" -f target/stm32h7x.cfg -c "init" -c "halt" `
       -c "program <路径>\DM_MC02_Chassis.hex verify" -c "shutdown"

# 再复位一次让它跑起来
& $ocd ... -c "init" -c "halt" -c "mww 0xE000ED0C 0x05FA0004" -c "shutdown"
```

> **注意**：`program ... reset run` 在这块板子上经常留成 halt 状态，必须用
> **写 AIRCR（0xE000ED0C = 0x05FA0004）软复位** 才能真正跑起来。

### 5.3 读内部变量（调试用）

```powershell
# 1) 先从 elf 拿符号地址
arm-none-eabi-nm build\DM_MC02_Chassis.elf | findstr "s_data s_state g_can_rx_count"

# 2) 用 OpenOCD 读
& $ocd ... -c "init" -c "halt" -c "mdw 0x24000xxx 8" -c "shutdown"
```

---

## 6. 明天/下一步 TODO（按优先级）

### P0｜CAN 打通（唯一阻塞项）
**建议直接用以官方例程当底**：
1. 下载 `https://gitee.com/kit-miao/dm-mc02` → `例程/CtrBoard-H7_CAN`
2. 先单独验证它能收到电调反馈（应该能看到 0x201~0x204）
3. 对比它和本工程的 `MX_FDCAN3_Init` / `Can_Init` 差异，重点看：
   - `MessageRAMOffset`（官方：FDCAN1=0、FDCAN2=0x406、FDCAN3=0x812）
   - `TxFifoQueueElmtsNbr` / `TxBuffersNbr`
   - 是否调用了 `HAL_FDCAN_ActivateNotification`
4. 电流方向：本工程 `Can_Send` 用 `HAL_FDCAN_AddMessageToTxFifoQ`

**关键现象对照表**（明天自查）：
| 现象 | 含义 |
|---|---|
| `TXBTO` 在涨 | 帧发出去了 → 看 RX |
| `TEC` 涨到 248 | 发出去了但没人 ACK（线/终端电阻/电调没上电） |
| `TEC=0` 且 `TXBTO=0` | **压根没发**（消息 RAM 没配好）← 现在的状态 |
| `RXF0S` 有值但计数不涨 | 收到 FIFO 里了但没读（中断/轮询问题） |

### P1｜轮序标定（CAN 通了立刻做）
1. `CHASSIS_SELFTEST_ON_BOOT = 1` → 烧录 → 1→2→3→4 号电机依次点动
2. 记录哪个 ID 在哪个位置、正电流转向
3. 位置不对 → 改 `chassis.c` 里 `wheel_linear[]` 的下标映射
4. 转向不对 → 改 `CHASSIS_MOTOR_REVERSED_MASK`（bit0=1号）

### P2｜参数标定
- 轮半径：地上滚 10 圈量距离，`R = L/(10·2π)`
- 半轴距/半轮距：实测
- 速度环 Kp/Ki：台架阶跃响应调

### P3｜安全功能验证
- 关遥控器 → 电机 0 电流（已验证逻辑，需实测）
- 拔一根 CAN → 四轮停
- 摇杆不回中不能起步

---

## 7. 当前固件行为（烧进去是什么样）

| 功能 | 行为 |
|---|---|
| 上电 | 灯红→绿→蓝闪三下 |
| 遥控 | SBUS 自动收，掉线 >120ms 判失联 |
| SWA（CH7） | 一档 = 灯灭，另一档 = 灯亮（白） |
| 电机 | **不出力**（因为四轮没在线） |
| 摇杆 | 数据在解算，但 `motors_ok=0` 所以不驱动 |

---

## 8. 已知风险 / 注意事项

1. **WS2812 很脆**：改灯的时序/颜色顺序前先跑 `CHASSIS_LED_TEST`，确认能看到循环
2. **SWD 线容易松**：烧录失败时先查 `SWDIO/SWCLK/GND/3V3`，再试软复位
3. **UART 溢出**：改 rc_input 时不要去掉 OVRDIS，否则"收一会儿就死"
4. **CAN 通道**：板上 `CAN1`/`CAN2` 丝印和 FDCAN 编号不是一一对应，**实测是 FDCAN3**
5. **遥控输出模式**：遥控器一改 `Output mode`，接收机必须**断电重上**才生效
