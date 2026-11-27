/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 麦轮底盘主程序（DM-MC02 / STM32H723VGT6）
  ******************************************************************************
  * @attention
  *
  * 遥控：FS-i6 + FS-iA10B（iBUS 或 SBUS，自动识别）
  * 电机：4×M3508 + 4×C620，FDCAN3 @1Mbps，控制帧 0x200，反馈 0x201~0x204
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "can_probe.h"
#include "chassis.h"
#include "chassis_config.h"
#include "diag.h"
#include "rc_input.h"
#include "status_led.h"
#include "vehicle_control.h"

#if (CHASSIS_DEBUG_PRINT != 0)
#include <stdio.h>
#include <string.h>
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LED_PERIOD_MS     50U
#define DEBUG_PERIOD_MS  200U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static uint32_t s_last_led_ms;
static uint32_t s_last_debug_ms;

/* 固件版本号：每次烧录递增，用 SWD 读它就能确认"板子里跑的到底是哪一版"。 */
volatile uint32_t g_fw_version __attribute__((used)) = 11U;

/* 调试用：CAN 收到的帧总数和最后一个 ID，用 SWD 读（0x24000xxx）。 */
volatile uint32_t g_can_rx_count;
volatile uint32_t g_can_last_id;

/* 扫描后选中的 CAN 通道：0 = 三路都不通，1/2/3 = FDCAN1/2/3。 */
volatile uint32_t g_can_channel;

/* 现场探针用：1 = 至少有一帧被 ACK 过（说明这条总线上真的有节点）。 */
volatile uint32_t g_can_ack_seen;

/*
 * 用 SWD 实时调灯色：把 g_led_test_enable 写成 1，然后改 r/g/b（0~255）
 * 就能立刻看到效果，不用重新编译。
 */
volatile uint32_t g_led_test_enable;
volatile uint32_t g_led_test_r = 128U;
volatile uint32_t g_led_test_g;
volatile uint32_t g_led_test_b;

/* 1 = 用 SWA/SWB 切"只点亮第 N 个字节"，用来确定颜色顺序。 */
volatile uint32_t g_led_channel_test = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

#if (CHASSIS_LED_TEST == 0) && (CHASSIS_LED_CALIB == 0) && (CHASSIS_CAN_LIVE_LED == 0) && (VEHICLE_CHANNEL_PROBE == 0)
static LedState AppLedState(const ChassisState *state);
#endif
#if (CHASSIS_DEBUG_PRINT != 0)
static void AppDebugPrint(const ChassisState *state, const RcData *rc);
#endif
#if (CHASSIS_CAN_PROBE_ON_BOOT != 0) && (CHASSIS_LED_CALIB == 0)
static void AppShowCanProbe(void);
#endif
#if (VEHICLE_CHANNEL_PROBE != 0)
static void AppShowChannelProbe(const RcData *remote, uint32_t now_ms);
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief 把底盘状态翻译成一个灯色。
  */
#if (CHASSIS_LED_TEST == 0) && (CHASSIS_LED_CALIB == 0) && (CHASSIS_CAN_LIVE_LED == 0) && (VEHICLE_CHANNEL_PROBE == 0)
static LedState AppLedState(const ChassisState *state)
{
  const VehicleControlState *vehicle = VehicleControl_GetState();

  /* ID 探测 / 单轮自检优先：这两个模式下可能还没接遥控，要用灯告诉你结果。 */
  if (state->probe_mode)
  {
    return LED_STATE_PROBE;
  }

  if (state->self_test)
  {
    return LED_STATE_SELF_TEST;
  }

  /* 遥控状态直接看链路，不看 ChassisState：机械臂模式下 Chassis_Update 不跑，
   * state->rc_ok 会是上一次底盘模式留下的旧值。 */
  if (!Rc_IsSignalValid())
  {
    return LED_STATE_RC_LOST;
  }

  /* 机械臂模式：腕部 CAN 掉线才报故障。零点现在是自动建立的，没有"等回中"状态了。 */
  if (vehicle->mode == VEHICLE_MODE_ARM)
  {
    if (!vehicle->wrist_online)
    {
      return LED_STATE_MOTOR_FAULT;
    }
    return vehicle->enabled ? LED_STATE_READY : LED_STATE_LOCKED;
  }

  if (!state->motors_ok)
  {
    return LED_STATE_MOTOR_FAULT;
  }

  if (state->wait_center)
  {
    return LED_STATE_WAIT_CENTER;
  }

  if (state->armed)
  {
    return LED_STATE_READY;
  }

  return LED_STATE_LOCKED;
}
#endif

#if (VEHICLE_CHANNEL_PROBE != 0)
/**
  * @brief 通道自检：找出当前偏移最大的那一路，用"闪 N 下"报告它是第 N 路。
  *
  *   红灯闪 N 下 = 第 N 路正向推到底
  *   绿灯闪 N 下 = 第 N 路反向推到底
  *   暗蓝常亮   = 所有通道都在中位，或根本没收到帧
  *
  *   用法：把某一根杆推到顶保持住，数灯闪几下，就知道它对应 CH 几。
  *   一根一根来，记完一张表就能反过来改 vehicle_config.h 的通道号。
  */
static void AppShowChannelProbe(const RcData *remote, uint32_t now_ms)
{
  uint8_t best = 0U;
  float magnitude = 0.0f;
  uint8_t blinks;
  uint32_t period;
  uint32_t slot;

  for (uint8_t i = 0U; i < 10U; ++i)
  {
    float value = remote->analog[i];

    if (value < 0.0f) value = -value;
    if (value > magnitude)
    {
      magnitude = value;
      best = i;
    }
  }

  if ((!remote->frame_valid) || (magnitude < 0.25f))
  {
    StatusLed_SetColor(0U, 0U, 8U);
    return;
  }

  blinks = (uint8_t)(best + 1U);
  period = ((uint32_t)blinks * 300U) + 900U;
  slot = now_ms % period;

  if (slot < ((uint32_t)blinks * 300U))
  {
    bool on = ((slot % 300U) < 140U);

    if (!on)
    {
      StatusLed_SetColor(0U, 0U, 0U);
    }
    else if (remote->analog[best] > 0.0f)
    {
      StatusLed_SetColor(40U, 0U, 0U);
    }
    else
    {
      StatusLed_SetColor(0U, 40U, 0U);
    }
  }
  else
  {
    StatusLed_SetColor(0U, 0U, 0U);
  }
}
#endif

#if (CHASSIS_CAN_PROBE_ON_BOOT != 0) && (CHASSIS_LED_CALIB == 0)
/**
  * @brief 把三路扫描结果用灯报告出来：第 1/2/3 个颜色 = FDCAN1/2/3。
  *   绿 = 收到电机反馈（这一路就是接电调的那一路）
  *   黄 = 帧发出去了，但没有任何节点应答（收发器 OK，问题在接线/供电/集线板）
  *   蓝 = 帧压根没发出去（这一路没有收发器，或引脚没接到 CAN 座）
  *   红 = 初始化就失败
  *
  * 三个颜色每 500ms 换一个，中间灭 200ms 作分隔，总共约 2.1 秒。
  */
static void AppShowCanProbe(void)
{
  for (uint32_t channel = 1U; channel <= 3U; ++channel)
  {
    switch (g_can_probe[channel - 1U].verdict)
    {
      case CAN_PROBE_OK:      StatusLed_SetColor(0U, 40U, 0U);  break;
      case CAN_PROBE_TX_ONLY: StatusLed_SetColor(40U, 40U, 0U); break;
      case CAN_PROBE_DEAD:    StatusLed_SetColor(0U, 0U, 40U);  break;
      default:                StatusLed_SetColor(40U, 0U, 0U);  break;
    }

    HAL_Delay(500U);
    StatusLed_SetColor(0U, 0U, 0U);
    HAL_Delay(200U);
  }
}
#endif

#if (CHASSIS_DEBUG_PRINT != 0)
/**
  * @brief 200ms 一次的状态打印（UART5_TX = PB13，接 USB 转 TTL 就能看）。
  *
  * 格式：RC=<协议> raw1..4 SW5 SW6 | M<在线掩码> T<四轮温度>
  *       | W<四轮目标速度 x10> S<四轮实际速度 x10> C<四轮电流>
  */
static void AppDebugPrint(const ChassisState *state, const RcData *rc)
{
  char line[160];
  int length;

  length = snprintf(line, sizeof(line),
                    "RC=%s f=%lu raw=%u,%u,%u,%u sw=%u,%u | online=%u T=%u,%u,%u,%u | "
                    "W=%d,%d,%d,%d S=%d,%d,%d,%d C=%d,%d,%d,%d\r\n",
                    Rc_ProtocolName(),
                    (unsigned long)rc->frame_counter,
                    (unsigned)rc->raw[0], (unsigned)rc->raw[1],
                    (unsigned)rc->raw[2], (unsigned)rc->raw[3],
                    (unsigned)rc->switch_pos[4], (unsigned)rc->switch_pos[5],
                    (unsigned)(0x0FU & (uint8_t)~state->missing_mask),
                    (unsigned)0U, (unsigned)0U, (unsigned)0U, (unsigned)0U,
                    (int)(state->wheel_target[0] * 10.0f),
                    (int)(state->wheel_target[1] * 10.0f),
                    (int)(state->wheel_target[2] * 10.0f),
                    (int)(state->wheel_target[3] * 10.0f),
                    (int)(state->wheel_speed[0] * 10.0f),
                    (int)(state->wheel_speed[1] * 10.0f),
                    (int)(state->wheel_speed[2] * 10.0f),
                    (int)(state->wheel_speed[3] * 10.0f),
                    (int)state->current[0], (int)state->current[1],
                    (int)state->current[2], (int)state->current[3]);

  if (length > 0)
  {
    (void)HAL_UART_Transmit(&huart5, (uint8_t *)line, (uint16_t)length, 20U);
  }
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_UART5_Init();
  MX_FDCAN3_Init();
  MX_SPI6_Init();
  /* USER CODE BEGIN 2 */

  /* 状态灯：先红->绿->蓝自检，确认灯的通路没问题。 */
  StatusLed_Init();
  StatusLed_BootSequence();

#if (CHASSIS_DIAG_MODE != 0)
  /* 诊断模式：只测引脚/串口，不驱动、不返回。 */
  Diag_Run();
#endif

  /* --- CAN：上电扫描 FDCAN1/2/3，确定电调挂在哪一路 --------------------- */
#if (CHASSIS_CAN_PROBE_ON_BOOT != 0)
  /* 蓝灯常亮 = 正在扫总线（约 2.2 秒）。 */
  StatusLed_SetColor(0U, 0U, 40U);

  g_can_channel = CanProbe_Run();

#if (CHASSIS_LED_CALIB == 0)
  /* 用 3 个颜色依次报告 FDCAN1 / FDCAN2 / FDCAN3 的结果。 */
  AppShowCanProbe();
#endif
#else
#if (CHASSIS_CAN_CHANNEL == 1)
  MX_FDCAN1_Init();
  if (!Can_Init(&hfdcan1))
  {
    Error_Handler();
  }
#elif (CHASSIS_CAN_CHANNEL == 2)
  MX_FDCAN2_Init();
  if (!Can_Init(&hfdcan2))
  {
    Error_Handler();
  }
#else
  MX_FDCAN3_Init();
  if (!Can_Init(&hfdcan3))
  {
    Error_Handler();
  }
#endif
  g_can_channel = CHASSIS_CAN_CHANNEL;
#endif

  /* 底盘：初始化电机对象并先发一帧零电流，保证 C620 上电不会乱动。 */
  Chassis_Init();

  /* 遥控：先按 iBUS 开串口，之后自动在 iBUS / SBUS 之间探测。 */
  (void)Rc_Init();
  VehicleControl_Init();

  s_last_led_ms = HAL_GetTick();
  s_last_debug_ms = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* 让版本号变量被真正引用，避免被 --gc-sections 丢掉（SWD 要读它）。 */
    g_fw_version = 11U;

    /* 收遥控（内部组帧 + 协议自动识别，非阻塞）。 */
    (void)Rc_Update();

    /* 整车控制：在底盘和机械臂模式之间互斥路由。 */
    VehicleControl_Update(now);

    /* 用探测选中的那一路；三路都不通时 can 为 NULL，下面整段跳过。 */
    FDCAN_HandleTypeDef *can = Can_GetHandle();

#if (CHASSIS_CAN_LIVE_LED != 0)
    /*
     * 现场找 CAN 口模式：每 1.5 秒在两套波特率时序之间切一次。
     * 晶振是 24 还是 25 MHz 没确认，这样接线一接对，总有一半时间能对上。
     */
    {
      static uint32_t s_retime_ms;
      static uint32_t s_timing;

      if ((uint32_t)(now - s_retime_ms) >= 1500U)
      {
        s_retime_ms = now;
        s_timing ^= 1U;
        (void)CanProbe_Retime(s_timing);
      }
    }
#endif

    if (can != NULL)
    {
      /* CAN 被"总线关闭"（对面没人应答）时自动恢复：直接清 INIT 位。 */
      if ((can->Instance->CCCR & FDCAN_CCCR_INIT) != 0U)
      {
        can->Instance->CCCR &= ~FDCAN_CCCR_INIT;
        can->State = HAL_FDCAN_STATE_BUSY;
      }

      /* 轮询收 CAN 帧（不依赖中断，和回环自测用的是同一条路径） */
      while (HAL_FDCAN_GetRxFifoFillLevel(can, FDCAN_RX_FIFO0) > 0U)
      {
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];

        if (HAL_FDCAN_GetRxMessage(can, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
        {
          g_can_rx_count++;
          g_can_last_id = rx_header.Identifier;
          Chassis_OnCanRx(rx_header.Identifier, rx_data, (uint8_t)rx_header.DataLength);
          VehicleControl_OnCanRx(rx_header.Identifier, rx_data, (uint8_t)rx_header.DataLength);
        }
      }
    }

    /* 状态灯：每 50ms 刷新一次。 */
    if ((uint32_t)(now - s_last_led_ms) >= LED_PERIOD_MS)
    {
      s_last_led_ms = now;

#if (CHASSIS_LED_CALIB != 0)
      {
        uint32_t t = now % 13000U;

        if (t < 4000U) { StatusLed_SetColor(40U, 0U, 0U); }
        else if (t < 8000U) { StatusLed_SetColor(0U, 40U, 0U); }
        else if (t < 12000U) { StatusLed_SetColor(0U, 0U, 40U); }
        else
        {
          uint32_t visible = (((t - 12000U) / 250U) % 2U) == 0U;
          StatusLed_SetColor(visible ? 40U : 0U, visible ? 40U : 0U, visible ? 40U : 0U);
        }
      }
#elif (CHASSIS_LED_TEST != 0)
      {
        static const uint8_t colors[5][3] =
        {
          { 40U,  0U,  0U },
          {  0U, 40U,  0U },
          {  0U,  0U, 40U },
          { 40U, 40U, 40U },
          {  0U,  0U,  0U }
        };
        uint32_t idx = (now / 500U) % 5U;

        StatusLed_SetColor(colors[idx][0], colors[idx][1], colors[idx][2]);
      }
#else
      {
        const ChassisState *state = Chassis_GetState();
        const RcData *remote = Rc_Get();
        uint8_t index = state->self_test_wheel;

#if (VEHICLE_CHANNEL_PROBE != 0)
        /* 通道自检固件：灯只用来报"哪一路在动"，不显示状态。 */
        (void)state;
        (void)index;
        AppShowChannelProbe(remote, now);
#else
        if (state->probe_mode)
        {
          index = state->probe_count;
        }

        if (g_led_channel_test != 0U)
        {
          uint32_t ch = 0U;

          if (remote->switch_pos[RC_CH_ARM] == RC_SW_UP) { ch |= 1U; }
          if (remote->switch_pos[RC_CH_MODE] == RC_SW_UP) { ch |= 2U; }

          StatusLed_SetChannelRaw((uint8_t)ch, 40U);
        }
        else if (g_led_test_enable != 0U)
        {
          StatusLed_SetColorRaw((uint8_t)g_led_test_r,
                                (uint8_t)g_led_test_g,
                                (uint8_t)g_led_test_b);
        }
#if (CHASSIS_SWA_LED_TEST != 0)
        else if (remote->frame_valid)
        {
          if (remote->switch_pos[RC_CH_ARM] == RC_SW_UP)
          {
            StatusLed_SetColor(0U, 0U, 0U);
          }
          else
          {
            StatusLed_SetColor(40U, 40U, 40U);
          }
        }
#endif
        else
        {
          StatusLed_Update(AppLedState(state), index, now);
        }
#endif /* VEHICLE_CHANNEL_PROBE */
      }
#endif
    }

#if (CHASSIS_DEBUG_PRINT != 0)
    if ((uint32_t)(now - s_last_debug_ms) >= DEBUG_PERIOD_MS)
    {
      s_last_debug_ms = now;
      AppDebugPrint(Chassis_GetState(), Rc_Get());
    }
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 22;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

/**
  * @brief FDCAN RX FIFO0 回调：把 0x201~0x204 的反馈交给底盘驱动。
  *
  * @note 现在走的是主循环轮询，没有使能 RX 中断，这个回调只是留着备用；
  *       一旦有人重新打开中断，它也会跟着当前选中的通道走。
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef header;
  uint8_t data[8];

  if ((Can_GetHandle() == NULL) ||
      (hfdcan != Can_GetHandle()) ||
      ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U))
  {
    return;
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
  {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) == HAL_OK)
    {
      g_can_rx_count++;
      g_can_last_id = header.Identifier;
      Chassis_OnCanRx(header.Identifier, data, (uint8_t)header.DataLength);
      VehicleControl_OnCanRx(header.Identifier, data, (uint8_t)header.DataLength);
    }
  }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: source file line number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
