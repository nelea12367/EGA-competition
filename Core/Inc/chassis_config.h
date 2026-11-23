/**
 * @file chassis_config.h
 * @brief 麦轮底盘的全部可调参数（调车只改这一个文件）。
 *
 * 分成 6 块：
 *   1. 麦轮几何   —— 决定解算对不对，必须实测
 *   2. 速度上限   —— 先保守，跑顺了再放开
 *   3. 加减速     —— 决定"起步/停车顺不顺"，比 PID 更影响手感
 *   4. 速度环参数 —— 决定"跟得快不快"
 *   5. 遥控映射   —— FS-i6 6 个通道怎么用
 *   6. 安全与自检 —— 失联、掉线、单轮点动
 */

#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/* ========================================================================
 * 1. 麦轮几何（⚠ 必须实测，量错这几个数，车就会走歪/原地打转）
 * ======================================================================*/

/* 麦轮压地后的有效半径，单位 m。
 * 实测方法：轮子在地面上滚 10 圈，量走过的距离 L，则 R = L / (10 * 2π)。
 * 4 寸麦轮（直径 100mm）标称 0.050m 轮毂 + 辊子外凸，实测一般在 0.075~0.080。 */
#define CHASSIS_WHEEL_RADIUS     0.0762f

/* 半轴距 a：左前轮到左后轮的距离 / 2，单位 m（前后方向）。 */
#define CHASSIS_HALF_WHEELBASE   0.250f

/* 半轮距 b：左前轮到右前轮的距离 / 2，单位 m（左右方向）。 */
#define CHASSIS_HALF_TRACK       0.250f

/* M3508 减速比，只做记录用（驱动层已经除过了，这里不用改）。 */
#define CHASSIS_GEAR_RATIO       (3591.0f / 187.0f)

/* 电机方向掩码：bit0=1号电机 bit1=2号 bit2=3号 bit3=4号。
 * 某一位写 1，表示这只电机的"正电流"在你车上转反了，驱动层会把它的
 * 控制量和反馈量同时取反，你在台架上就不用改接线。
 * 先全 0 上电看方向，哪只反了就把对应 bit 置 1。 */
#define CHASSIS_MOTOR_REVERSED_MASK  0x0AU

/* ========================================================================
 * 2. 速度上限（首版保守，场地里跑顺了再往上加）
 * ======================================================================*/
#define CHASSIS_MAX_VX            0.60f   /* 前后，m/s（常规档） */
#define CHASSIS_MAX_VY            0.60f   /* 左右平移，m/s（常规档） */
#define CHASSIS_MAX_WZ            1.50f   /* 自转，rad/s（常规档） */

/* 单个轮子的线速度硬上限，m/s。超过就整体等比缩小（保方向、不保速度）。
 * 3.5 m/s 对应转子约 8400 rpm，接近 M3508 空载极限，正常不会碰到。 */
#define CHASSIS_MAX_WHEEL_LINEAR  3.5f

/* 加速档（SWB 拨到 UP）：所有速度乘这个系数，用来跑快。 */
#define CHASSIS_FAST_SCALE        1.80f

/* ========================================================================
 * 3. 加减速（"起停平顺"的关键，单位 m/s²、rad/s²）
 * ======================================================================*/
#define CHASSIS_ACC_LINEAR        2.0f    /* 加速 */
#define CHASSIS_ACC_ANGULAR       3.0f
#define CHASSIS_DEC_LINEAR        2.0f    /* 松杆减速 */
#define CHASSIS_DEC_ANGULAR       6.0f

/* ========================================================================
 * 4. 速度环（反馈是轮子角速度 rad/s，输出是 C620 原始电流 -16384~16384）
 * ======================================================================*/
#define CHASSIS_SPEED_KP          600.0f
#define CHASSIS_SPEED_KI          400.0f
#define CHASSIS_SPEED_I_LIMIT     8000.0f

/* 电流上限（原始值）。16384 = 20A。12000 ≈ 14.6A，四轮同时打满会翘头，
 * 台架/首跑建议先用 6000，跑顺了再往上加。 */
#define CHASSIS_CURRENT_LIMIT     2000.0f

/* 每个控制周期电流最大变化量（斜坡），防止起步瞬间电流跳变"窜车"。 */
#define CHASSIS_CURRENT_SLEW      600.0f

/* 反馈超时：超过这么久没收到这只电机的 CAN 反馈，就判它掉线，四轮全停。 */
#define CHASSIS_MOTOR_TIMEOUT_MS  100U

/* ========================================================================
 * 5. 遥控映射
 *    左摇杆：上下 = 前进/后退（CH3），左右 = 左右平移（CH4）
 *    右摇杆：左右 = 原地自转（CH1）
 *    CH7 = SWA（控灯用）      CH8 = SWB（高速底盘/机械臂模式）
 * ======================================================================*/
#define RC_CH_FORWARD             2U      /* CH3：左摇杆上下 = 前进/后退 */
#define RC_CH_STRAFE              3U      /* CH4：左摇杆左右 = 平移 */
#define RC_CH_ROTATE              0U      /* CH1：右摇杆左右 = 自转 */
#define RC_CH_ARM                 6U      /* CH7：SWA（实测这台遥控器 SWA 在 CH7） */
#define RC_CH_MODE                7U      /* CH8：SWB 上 = 高速底盘；下 = 机械臂模式 */

/* 摇杆死区（归一化后 ±0.06，抵消中点漂移和手抖）。 */
#define RC_DEADBAND               0.06f

/* 通道原始值范围（一般不用动）。
 * iBUS：1000~2000us，中点 1500。
 * 标准 SBUS：172~1811，中点 992。
 * ⚠ 如果"摇杆全松，车却缓慢往一个方向跑"，说明这台接收机的中值不是 992，
 *   把 RC_SBUS_CENTER 改成 1500、RC_SBUS_SPAN 改成 500 再试。 */
#define RC_IBUS_CENTER            1500
#define RC_IBUS_SPAN              500.0f
#define RC_SBUS_CENTER            992
#define RC_SBUS_SPAN              820.0f

/* 摇杆曲线：0 = 线性，0.35 ≈ 中间柔和两头快（推荐 0.3~0.5）。 */
#define RC_EXPO                   0.35f

/* 符号：推杆方向反了就改这里，±1。
 * 前进：推 CH2 向上 = 前进          横移：推 CH1 向右 = 向右平移
 * 自转：推 CH4 向右 = 顺时针（右转） */
#define RC_SIGN_FORWARD          (+1.0f)
#define RC_SIGN_STRAFE           (-1.0f)
#define RC_SIGN_ROTATE           (-1.0f)

/* 使能开关：1 = 必须把 CH5 拨到指定档位才能动（更安全）；
 *           0 = 只要遥控在线、摇杆回中就能动（台架调试更方便）。
 * 建议：台架先用 0，车落地跑之前改成 1。 */
#define CHASSIS_REQUIRE_ARM       0

/* CH5 拨到哪一档算"允许行驶"：RC_SW_UP / RC_SW_MID / RC_SW_DOWN。
 * 如果方向反了（拨上去反而是不使能），把这里换成另一个。 */
#define RC_ARM_SWITCH_LEVEL       RC_SW_UP

/* SWB 拨到 RC_SW_UP 时进高速底盘档（速度 × CHASSIS_FAST_SCALE）。
 * SWB 下由 vehicle_control.c 切到机械臂模式，底盘不会出力。 */
#define RC_FAST_SWITCH_LEVEL      RC_SW_UP

/* ========================================================================
 * 6. 安全与自检
 * ======================================================================*/

/* 遥控超时：超过这么久收不到有效帧就判失联，四轮电流立即归零。 */
#define RC_LINK_TIMEOUT_MS        120U

/* 上电后必须把摇杆回中才能起步（防止上电瞬间窜出去），保持这么久才算回中。 */
#define CHASSIS_CENTER_HOLD_MS    200U

/* 单轮点动自检：改成 1 编译烧录，上电后 1→2→3→4 号电机各转 1.2 秒，
 * 用来确认"哪个 ID 装在哪个位置""正转会往哪边转"。测完改回 0 再烧一次。
 * ⚠ 自检必须架空车轮！ */
#ifndef CHASSIS_SELFTEST_ON_BOOT
#define CHASSIS_SELFTEST_ON_BOOT  0
#endif

#define CHASSIS_SELFTEST_CURRENT  1000    /* 原始电流，约 2A */
#define CHASSIS_SELFTEST_STEP_MS  2000U

/* 自检只驱动某一个槽位（0~3 = 1~4 号电机）；0xFF = 依次全部。 */
#ifndef CHASSIS_SELFTEST_ONLY_SLOT
#define CHASSIS_SELFTEST_ONLY_SLOT  2U
#endif

/* 电机 ID 探测：改成 1 编译烧录，板子只发 0 电流、只听不驱动，
 * 每 2 秒统计一次"0x201~0x208 里有哪些 ID 在回话"，然后
 * 灯闪 N 次 = 探测到的 ID 是 N 号（闪 0 次 = 一块都没看见，灯慢闪红）。
 * 用法：只接一块电调 → 上电 → 看它闪几下，就知道这块是几号。
 * 测完改回 0。 */
#ifndef CHASSIS_ID_PROBE_ON_BOOT
#define CHASSIS_ID_PROBE_ON_BOOT  0
#endif

/* 调试串口打印：1 = 通过 UART5_TX(PB13) 按遥控波特率输出状态，
 * 需要 USB 转 TTL 接 PB13+GND。默认关。 */
#define CHASSIS_DEBUG_PRINT       0

/* 硬件诊断模式：遥控完全没反应时用。板子只测引脚电平、不驱动电机，
 * 结果放在 RAM 里用 SWD 读（见 diag.c 顶部注释）。正常使用保持 0。 */
#ifndef CHASSIS_DIAG_MODE
#define CHASSIS_DIAG_MODE         0
#endif

/* ========================================================================
 * 6.5 CAN 通道（这块板子最容易搞错的地方）
 * ======================================================================*/

/* 上电自动扫描 FDCAN1/2/3，找出电调到底挂在哪一路：
 *   1 = 开（每次上电多花约 20 秒做完整扫描 + 波特率扫描，用灯报告结果）
 *   0 = 关，直接用下面的 CHASSIS_CAN_CHANNEL（正常跑车用这个）
 * 报告方式见 main.c 的 AppShowCanProbe()：
 *   第 1/2/3 个颜色 = FDCAN1/2/3，绿=收到电机反馈，黄=能发没人应答，
 *   蓝=压根没发出去，红=初始化失败。 */
#ifndef CHASSIS_CAN_PROBE_ON_BOOT
#define CHASSIS_CAN_PROBE_ON_BOOT 0
#endif

/* 关闭扫描时用哪一路：1 = PD0/PD1，2 = PB5/PB6，3 = PD12/PD13。
 * 厂商例程（DM_MC02_Test.ioc）用的是 FDCAN2。 */
#ifndef CHASSIS_CAN_CHANNEL
#define CHASSIS_CAN_CHANNEL       1
#endif

/* FDCAN3 在 PD12/PD13 上的复用功能号：2 还是 5 一直没确认。
 *
 * 现象：把 PD13 当普通 IO 拉低时，PD12 会跟着变低 -> 这一路**有活的收发器**；
 *      但 FDCAN3 内核连第一帧都发不出去（TXBRP 卡在 0xFF、TEC=0、TXBTO=0），
 *      这正好是"复用功能号配错，RX 信号没进到内核，内核以为总线一直是显性"。
 * 所以先按 5 试。如果这一路本来是好的，换过来立刻就能收到帧。 */
#ifndef CHASSIS_FDCAN3_AF
#define CHASSIS_FDCAN3_AF         5
#endif

/* SWA(CH5) 控灯测试：1 = 把 SWA 拨上/中/下，灯分别变蓝/绿/红（验证通道用）。
 * 0 = 恢复正常状态灯：红闪=没遥控、绿闪=正常、紫闪=电机掉线。 */
#ifndef CHASSIS_SWA_LED_TEST
#define CHASSIS_SWA_LED_TEST      0
#endif

/* 纯灯测试：1 = 上电后无限循环 红→绿→蓝→白→灭（每种 0.5 秒），用来确认灯驱动。 */
#ifndef CHASSIS_LED_TEST
#define CHASSIS_LED_TEST          0
#endif

/* ------------------------------------------------------------------------
 * 灯色标定（只在搞不清颜色顺序时用，正常保持 0）
 * ----------------------------------------------------------------------*/

/* 1 = 灯进入标定循环：复位后 0~3 秒显示"红"、3~6 秒显示"绿"、6~9 秒显示"蓝"，
 *     然后一直循环。用来确认这颗灯到底是 GRB 还是 RGB 顺序。
 * 看第一段（复位后头 3 秒）实际是什么颜色，就能反推出 CHASSIS_LED_ORDER。 */
#ifndef CHASSIS_LED_CALIB
#define CHASSIS_LED_CALIB         0
#endif

/* 颜色字节顺序：
 *   0 = GRB（代码按 绿-红-蓝 发）
 *   1 = RGB（代码按 红-绿-蓝 发）
 *
 * ✅ 这台板子实测就是 **GRB**（标准 WS2812B）：
 *   只点亮第 1 个字节 -> 绿
 *   只点亮第 2 个字节 -> 橙红
 *   只点亮第 3 个字节 -> 蓝
 *   三个全亮         -> 白
 *   所以保持 0。
 *
 * ⚠ 注意这颗灯的"红"肉眼看偏橙，满亮度也不会变成正红，属于正常现象。
 *   写文档/记颜色时按"红≈橙"理解，别以为通道错了。
 *
 * 重新确认的方法：把 g_led_test_enable 写 1（SWD），再写
 * g_led_test_r/g/b，看灯颜色跟不跟。 */
#ifndef CHASSIS_LED_ORDER
#define CHASSIS_LED_ORDER         0
#endif

/* 灯的整体亮度倍数：代码里所有颜色值（40 = "全亮"档）都会乘这个数。
 *  1 = 原始（很暗，40/255）
 *  2 = 80/255（默认，红色还能保持偏红）
 *  3 = 120/255（更亮，但红色开始偏橙/黄）
 *  6 = 240/255（很刺眼，红色会明显发橙）
 * 上限会被截到 255。
 *
 * ⚠ 这颗灯的红色芯片亮度一高就显橙红，所以"红"和"橙"在高亮下很难区分。
 *   状态灯靠"红/橙"区分状态时，别把亮度开太高。 */
#ifndef CHASSIS_LED_BRIGHTNESS
#define CHASSIS_LED_BRIGHTNESS    2
#endif

/* ------------------------------------------------------------------------
 * CAN 现场探针（没有万用表时用它找一个"活"的 CAN 口）
 * ----------------------------------------------------------------------*/

/* 1 = 灯变成 CAN 应答指示灯：
 *       绿 = 板上这一路 CAN 有节点应答（帧被 ACK 了，说明电调真的在这条总线上）
 *       红 = 没有应答（这一路不通）
 *   SWA 控灯会被临时覆盖。找完口子记得改回 0。 */
#ifndef CHASSIS_CAN_LIVE_LED
#define CHASSIS_CAN_LIVE_LED      0
#endif

#endif /* CHASSIS_CONFIG_H */
