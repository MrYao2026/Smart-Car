/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "bsp_hcsr04.h"
#include "bsp_motor.h"
#include "bsp_oled.h"
#include "bsp_servo.h"
#include "bsp_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* 工程未集成 HAL UART 源文件；此轻量句柄仅供 main.c 的 USART1 单字节接收适配层使用。 */
typedef struct
{
  USART_TypeDef *Instance;
  uint8_t *pRxBuffPtr;
  volatile uint16_t RxXferCount;
} UART_HandleTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 动态安全距离为减速起点；15 cm 及以内始终停车。 */
#define OBSTACLE_STOP_DISTANCE_CM 15.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
/* USART1 用于 HC-04 蓝牙控制；接收缓冲区由单字节中断接口管理。 */
static UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* 修改说明：新增共享状态变量，供 USART1 回调与主循环共同实现遥控及自动模式。 */
/* 保存当前运行模式：0 空闲、1 自动避障、2 自动跟随、3 手动遥控。 */
volatile uint8_t car_mode = 0U;
/* 手动基础速度可正向或反向，初始化为停车；自动模式命令也复用该值。 */
volatile int16_t base_speed = 0;
/* 左右轮差速转向偏置，正值左轮减速、右轮加速，范围为 -30 至 30。 */
volatile int16_t turn_bias = 0;
/* 最近一次有效蓝牙控制命令时刻，由 UART 回调写入、主循环读取。 */
static volatile uint32_t last_cmd_tick = 0U;
/* 保存 USART1 中断最近一次收到的蓝牙字符。 */
volatile char rx_char = '\0';
/* OLED 按实际时间刷新；五次测距耗时变化时不会额外等待十轮。 */
static uint32_t oled_refresh_tick = 0U;
static uint32_t main_loop_tick = 0U;
/* 保存当前合成后的左右轮目标速度。 */
static volatile int16_t manual_left_speed = 0;
static volatile int16_t manual_right_speed = 0;
static volatile char manual_motion = '\0';
/* 跟随模式当前的平滑速度及上次渐变更新时间。 */
static volatile int16_t follow_speed = 0;
static volatile uint32_t follow_ramp_tick = 0U;
/* 超时后保持搜索状态，直到重新测得 0 到 50 厘米内的目标。 */
static uint8_t follow_search_active = 0U;
/* 避障蜂鸣器使用间歇短鸣，记录上次触发时刻。 */
static uint32_t obstacle_beep_tick = 0U;
/* 记录手动避障是否已在 15 cm 内停车，用于暂停高级动作与报警。 */
static volatile uint8_t manual_obstacle_active = 0U;
/* 主循环发布最新距离，UART 手动输出也必须按此距离执行减速。 */
static volatile float obstacle_distance_cm = 999.0f;
static volatile uint8_t config_save_request = 0U;

/* 可掉电保存的速度上限与避障基准距离；上电时从专用 Flash 页加载。 */
static Config_t app_config = {100U, 15U};

/* 搬运子状态只在模式 3 内有效，不额外占用一个 car_mode 编号。 */
typedef enum
{
  CARRY_STATE_INACTIVE = 0U,
  CARRY_STATE_WAITING,
  CARRY_STATE_SEARCHING,
  CARRY_STATE_APPROACHING,
  CARRY_STATE_SERVO_ALERT,
  CARRY_STATE_MANUAL,
  CARRY_STATE_COMPLETE
} CarryState;

/* 这些状态由 USART1 接收中断和主循环共同访问，声明为 volatile。 */
static volatile CarryState carry_state = CARRY_STATE_INACTIVE;
static volatile uint32_t carry_entry_tick = 0U;
static volatile uint8_t carry_servo_was_up = 0U;
static volatile char carry_servo_request = '\0';
/* 蜂鸣器滴滴报警的间隔只由主循环更新。 */
static volatile uint32_t carry_alarm_tick = 0U;

/* 示教记录每项存放按键 ASCII 码与该动作持续的毫秒数。 */
#define TEACH_RECORD_CAPACITY 200U
static volatile uint32_t teach_record[TEACH_RECORD_CAPACITY][2];
static volatile uint16_t teach_record_count = 0U;
static volatile uint8_t teach_recording = 0U;
static volatile uint8_t teach_playing = 0U;
static volatile uint8_t teach_dispatching = 0U;
static volatile uint32_t teach_last_event_tick = 0U;
static volatile uint16_t teach_play_index = 0U;
static volatile uint32_t teach_play_deadline = 0U;

/* 跳舞动作阶段；所有阶段转换均由主循环节拍驱动，不使用 HAL_Delay。 */
typedef enum
{
  DANCE_STATE_OFF = 0U,
  DANCE_STATE_LEFT,
  DANCE_STATE_RIGHT,
  DANCE_STATE_SERVO_UP,
  DANCE_STATE_SERVO_DOWN,
  DANCE_STATE_BEEP_FIRST_ON,
  DANCE_STATE_BEEP_FIRST_GAP,
  DANCE_STATE_BEEP_SECOND_ON,
  DANCE_STATE_BEEP_SECOND_GAP,
  DANCE_STATE_DONE
} DanceState;

static volatile DanceState dance_state = DANCE_STATE_OFF;
static volatile uint32_t dance_state_tick = 0U;
static uint8_t dance_servo_cycles = 0U;
static uint8_t dance_active = 0U;
static volatile uint32_t obstacle_task_pause_tick = 0U;

/* 记录蜂鸣器关闭时刻，使用非阻塞方式鸣叫以保持主循环及时响应。 */
static volatile uint32_t buzzer_off_tick = 0U;
static volatile uint8_t buzzer_active = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart,
                                              uint8_t *data,
                                              uint16_t size);
static void Buzzer_Beep(uint32_t duration_ms);
static void Buzzer_Update(void);
static void ApplyManualMotion(void);
static void Manual_UpdateOutput(void);
static float Obstacle_GetSafeDistance(int16_t requested_speed);
static float Obstacle_GetScale(float distance_cm, int16_t requested_speed);
static void Obstacle_SetSpeed(int16_t left_speed, int16_t right_speed);
static int16_t ClampMotorSpeed(int16_t speed);
static int16_t ClampBaseSpeed(int32_t speed);
static void Carry_Update(float distance_cm);
static void Follow_Control(float distance_cm);
static void Teach_RecordCommand(char cmd, uint32_t now);
static void Teach_Update(void);
static void Dance_Start(void);
static void Dance_Stop(void);
static void Dance_Update(void);
void Bluetooth_ProcessCmd(char cmd);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 初始化 SSD1306 OLED；若屏幕地址无应答，则进入工程统一错误处理。 */
  if (OLED_Init() != HAL_OK)
  {
    Error_Handler();
  }

  /* 初始化 HC-SR04 的 Trig、Echo 引脚及用于微秒测量的 DWT 计数器。 */
  HCSR04_Init();

  /* 初始化左右电机的方向引脚及 PWM 输出，供蓝牙命令控制电机。 */
  Motor_Init();

  /* 初始化 SG90 舵机；TIM2_CH4 已由 MX_TIM2_Init() 配置为 50 Hz PWM。 */
  Servo_Init();

  /* 读取 Flash 参数；数据未保存或校验失败时 Config_Load 返回默认配置。 */
  app_config = Config_Load();
  last_cmd_tick = HAL_GetTick();
  main_loop_tick = HAL_GetTick();

  /* 启动 USART1 单字节中断接收；回调完成后会自动挂接下一字节。 */
  if (HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_char, 1U) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 最短调度间隔为 20 ms；五次超声波采样本身会耗时数百毫秒。 */
    const uint32_t loop_now = HAL_GetTick();
    if ((uint32_t)(loop_now - main_loop_tick) < 20U)
    {
      __WFI();
      continue;
    }
    main_loop_tick = loop_now;

    /* 保留原有 30 秒无操作逻辑；搬运手动控制也必须在空闲超时后停车。 */
    if ((uint32_t)(HAL_GetTick() - last_cmd_tick) > 30000U)
    {
      base_speed = 0;
      turn_bias = 0;
      manual_left_speed = 0;
      manual_right_speed = 0;
      manual_motion = '\0';
      teach_playing = 0U;
      teach_recording = 0U;
      dance_active = 0U;
      dance_state = DANCE_STATE_OFF;
      if (carry_state != CARRY_STATE_INACTIVE)
      {
        /* 超时后留在搬运手动状态，避免自动寻找状态再次驱动车轮。 */
        carry_state = CARRY_STATE_MANUAL;
      }
      Motor_SetSpeed(0, 0);
      Buzzer_Beep(100U);
      last_cmd_tick = HAL_GetTick();
    }

    /* 更新蜂鸣器非阻塞定时状态，音效结束后自动关闭 PB5。 */
    Buzzer_Update();

    /* Flash 页擦写由主循环执行，绝不在 UART 中断服务程序中擦写。 */
    if (config_save_request != 0U)
    {
      config_save_request = 0U;
      if (Config_Save(&app_config) != (int)HAL_OK)
      {
        Error_Handler();
      }
    }

    /* 连续测量五次并去掉极值取平均；有效样本不足时返回 0。 */
    float dist = HCSR04_GetFilteredDistance();
    const float measured_dist = dist;
    if (dist <= 0.0f)
    {
      /*
       * 关键错误值过滤：驱动以 0 表示超时、回波无效或有效样本不足，
       * 绝不能将 0 当成“前方 0 厘米”而误触发停车、报警或倒车。
       * 强制改为 999 厘米，供运动控制和 OLED 的统一超时判断使用。
       */
      dist = 999.0f;
    }

    /* 用期望基础速度计算边界，禁止把减速结果写回 base_speed。 */
    obstacle_distance_cm = dist;
    const float safe_dist = Obstacle_GetSafeDistance(base_speed);

    /* 舵机渐进更新、回放调度均为非阻塞调用，不影响蓝牙中断接收。 */
    Servo_Update();
    if ((carry_servo_request != '\0') && (car_mode == 3U))
    {
      const char servo_command = carry_servo_request;
      carry_servo_request = '\0';
      Servo_RotateSmoothly((servo_command == 'U') ? 90U : 0U);
      if ((servo_command == 'N') && (carry_state != CARRY_STATE_INACTIVE))
      {
        carry_servo_was_up = 0U;
      }
    }
    /* 根据 car_mode 更新电机状态；所有速度均使用 0 至 100 的基础速度。 */
    switch (car_mode)
    {
      case 0U:
        /* 空闲模式始终停车，并清除跟随模式的渐变速度状态。 */
        Motor_SetSpeed(0, 0);
        follow_speed = 0;
        follow_search_active = 0U;
        break;

      case 1U:
        /* 先按距离输出轮速：边界外全速，减速区线性降低，15 cm 内停车。 */
        Obstacle_SetSpeed(base_speed, base_speed);
        if ((dist > 0.0f) && (dist < safe_dist))
        {
          if ((uint32_t)(HAL_GetTick() - obstacle_beep_tick) >= 500U)
          {
            Buzzer_Beep(80U);
            obstacle_beep_tick = HAL_GetTick();
          }
        }
        follow_speed = 0;
        follow_search_active = 0U;
        break;

      case 2U:
        /* 跟随控制使用完成超时处理的滤波距离；测距失败视为目标距离很远。 */
        Follow_Control(dist);
        break;

      case 3U:
        /*
         * 搬运子状态优先于普通手动避障：此分支不检查 safe_dist，
         * 保证寻物、靠近和抓取时可以贴近目标，不触发避障停车或报警。
         */
        if (carry_state != CARRY_STATE_INACTIVE)
        {
          if (manual_obstacle_active != 0U)
          {
            /* 关闭上一轮普通避障遗留的蜂鸣，搬运提示仍由状态机负责。 */
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
            buzzer_active = 0U;
          }
          manual_obstacle_active = 0U;
          /* 搬运子状态完全绕过避障覆盖；自动寻物使用未经超时替换的测距值。 */
          Carry_Update(measured_dist);
          follow_speed = 0;
          follow_search_active = 0U;
          break;
        }

        /*
         * 普通模式 3 的避障优先级高于手动、跳舞和示教动作：
         * 只有有效距离进入 15 cm 内才强制停车并暂停高级动作。
         * 15 cm 到动态边界之间继续执行动作，由统一输出入口限制轮速。
         */
        if ((dist > 0.0f) && (dist <= OBSTACLE_STOP_DISTANCE_CM))
        {
          const uint32_t pause_now = HAL_GetTick();
          const uint8_t first_stop = (manual_obstacle_active == 0U);
          /* 最新距离已发布；UART 的统一输出入口同样会强制输出零。 */
          manual_obstacle_active = 1U;
          Motor_SetSpeed(0, 0);
          if (first_stop != 0U)
          {
            obstacle_beep_tick = HAL_GetTick();
            obstacle_task_pause_tick = pause_now;
            Buzzer_Beep(100U);
            if (OLED_ShowString(0U, "Obstacle! Stop.") != HAL_OK)
            {
              Error_Handler();
            }
          }
          else if ((buzzer_active == 0U) &&
                   ((uint32_t)(HAL_GetTick() - obstacle_beep_tick) >= 300U))
          {
            /* 障碍持续存在时以间隔短鸣形成滴-滴-滴提示，不阻塞主循环。 */
            Buzzer_Beep(100U);
            obstacle_beep_tick = HAL_GetTick();
          }

          /* 避障期间暂停回放和跳舞的计时，清障后从中断处继续执行。 */
          const uint32_t pause_elapsed =
              (uint32_t)(pause_now - obstacle_task_pause_tick);
          if (dance_active != 0U)
          {
            dance_state_tick += pause_elapsed;
          }
          if (teach_playing != 0U)
          {
            teach_play_deadline += pause_elapsed;
          }
          obstacle_task_pause_tick = pause_now;
          Motor_SetSpeed(0, 0);
          break;
        }

        /* 离开 15 cm 停车区后恢复动作；仍在减速区时只输出受限轮速。 */
        if (manual_obstacle_active != 0U)
        {
          if (OLED_ShowString(0U, "Manual Mode") != HAL_OK)
          {
            Error_Handler();
          }
        }
        manual_obstacle_active = 0U;
        obstacle_task_pause_tick = HAL_GetTick();

        if ((dist > OBSTACLE_STOP_DISTANCE_CM) && (dist < safe_dist) &&
            ((uint32_t)(HAL_GetTick() - obstacle_beep_tick) >= 500U))
        {
          Buzzer_Beep(80U);
          obstacle_beep_tick = HAL_GetTick();
        }

        if (dance_active != 0U)
        {
          Dance_Update();
        }
        else
        {
          Teach_Update();
          /* 回放命令已在 Teach_Update 派发；此处输出当前差速目标。 */
          Manual_UpdateOutput();
        }
        follow_speed = 0;
        follow_search_active = 0U;
        break;

      default:
        /* 非法模式值恢复为空闲并立即停车，确保状态机故障时处于安全状态。 */
        car_mode = 0U;
        Motor_SetSpeed(0, 0);
        follow_speed = 0;
        follow_search_active = 0U;
        break;
    }

    /* 测距完成后按时间刷新，避免五次滤波让屏幕滞后数秒。 */
    if ((uint32_t)(HAL_GetTick() - oled_refresh_tick) >= 200U)
    {
      char distance_text[22];
      char speed_text[22];
      const char *mode_text = "Mode: IDLE";
      int formatted_length;

      /* 按当前运行模式选择模式状态文本。 */
      if ((car_mode == 3U) &&
          (carry_state == CARRY_STATE_INACTIVE) &&
          (manual_obstacle_active != 0U))
      {
        mode_text = "Obstacle! Stop.";
      }
      else if ((car_mode == 3U) &&
               (carry_state == CARRY_STATE_INACTIVE) &&
               (dist > OBSTACLE_STOP_DISTANCE_CM) && (dist < safe_dist))
      {
        mode_text = "Obstacle: Slow";
      }
      else if ((car_mode == 3U) && (dance_active != 0U))
      {
        mode_text = "Dancing! (^_^)";
      }
      else if ((car_mode == 3U) && (teach_recording != 0U))
      {
        mode_text = "REC";
      }
      else if ((car_mode == 3U) && (teach_playing != 0U))
      {
        mode_text = "PLAY";
      }
      else if (car_mode == 1U)
      {
        mode_text = "Mode: AVOID";
      }
      else if (car_mode == 2U)
      {
        mode_text = "Mode: FOLLOW";
      }
      else if (car_mode == 3U)
      {
        mode_text = (carry_state != CARRY_STATE_INACTIVE)
                        ? "Mode: CARRY"
                        : "Mode: MANUAL";
      }

      if (OLED_ShowString(0U, mode_text) != HAL_OK)
      {
        Error_Handler();
      }

      /* 舵机初始在抬起位置时，明确提示用户先手动放下推板。 */
      if (carry_state == CARRY_STATE_SERVO_ALERT)
      {
        formatted_length =
            snprintf(distance_text, sizeof(distance_text), "Please Lower Servo");
      }
      /*
       * 显示与模式 1/模式 3 避障共用同一个 dist：
       * 有效滤波距离保持原值，无效读数已在主循环前面转换成 999 cm，
       * 因此正常有效读数不会在这里被误判为 0；999 则显示为 Timeout。
       */
      else if ((dist <= 0.0f) || (dist > 400.0f))
      {
        const char *reason = "few";
        switch (HCSR04_GetLastStatus())
        {
          case HCSR04_STATUS_NOT_READY: reason = "init"; break;
          case HCSR04_STATUS_ECHO_HIGH: reason = "high"; break;
          case HCSR04_STATUS_NO_RISE:   reason = "rise"; break;
          case HCSR04_STATUS_NO_FALL:   reason = "fall"; break;
          case HCSR04_STATUS_RANGE:     reason = "range"; break;
          default: break;
        }
        /* 例如 Timeout:rise 0/5：五次全无效，最近失败在等待上升沿。 */
        formatted_length =
            snprintf(distance_text, sizeof(distance_text), "Timeout:%s %u/5",
                     reason, (unsigned int)HCSR04_GetValidSampleCount());
      }
      else
      {
        /* OLED 只打印整数厘米，不使用 GCC 默认可能不支持的浮点格式。 */
        formatted_length =
            snprintf(distance_text, sizeof(distance_text), "Dist: %d cm",
                     (int)dist);
      }
      if ((formatted_length < 0) ||
          ((size_t)formatted_length >= sizeof(distance_text)))
      {
        Error_Handler();
      }
      if (OLED_ShowString(1U, distance_text) != HAL_OK)
      {
        Error_Handler();
      }

      /* 第三行显示基础速度，检查格式化结果及 OLED 写入状态。 */
      formatted_length =
          snprintf(speed_text, sizeof(speed_text), "Spd: %d", (int)base_speed);
      if ((formatted_length < 0) ||
          ((size_t)formatted_length >= sizeof(speed_text)))
      {
        Error_Handler();
      }
      if (OLED_ShowString(2U, speed_text) != HAL_OK)
      {
        Error_Handler();
      }

      oled_refresh_tick = HAL_GetTick();
    }

    /* 本轮末尾再次检查蜂鸣器定时状态。 */
    Buzzer_Update();
  }
  /* USER CODE END 3 */
}

/**
  * @brief USART1 Initialization Function
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  /* 使能 GPIOA 和 USART1 时钟，分别访问 PA9/PA10 与串口寄存器。 */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  /* 将 PA9 配置为 USART1_TX 复用推挽输出。 */
  GPIO_InitTypeDef gpio_init = {0};
  gpio_init.Pin = GPIO_PIN_9;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  /* 将 PA10 配置为 USART1_RX 输入，使用上拉保持空闲时的高电平。 */
  gpio_init.Pin = GPIO_PIN_10;
  gpio_init.Mode = GPIO_MODE_INPUT;
  gpio_init.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  /* 配置 USART1 8N1、无硬件流控；BRR 使用 APB2 时钟计算 9600 波特率。 */
  USART1->CR1 = 0U;
  USART1->CR2 = 0U;
  USART1->CR3 = 0U;
  USART1->BRR = (uint16_t)(HAL_RCC_GetPCLK2Freq() / 9600U);
  huart1.Instance = USART1;

  /* 使能发送器、接收器及 USART 外设，开始 USART1 通信。 */
  USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

  /* 配置 USART1 接收中断优先级并使能 NVIC 中断通道。 */
  HAL_NVIC_SetPriority(USART1_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
  * @brief 为当前工程提供 HAL_UART_Receive_IT 的 USART1 单字节接收适配。
  * @param huart USART1 接收句柄。
  * @param data 接收缓冲区。
  * @param size 本驱动适配仅支持接收一个字节。
  * @retval HAL_OK 成功启动；HAL_ERROR 参数无效；HAL_BUSY 已有接收任务。
  */
static HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart,
                                              uint8_t *data,
                                              uint16_t size)
{
  /* 检查外设、缓冲区和单字节长度，防止写入无效地址或越界。 */
  if ((huart == NULL) || (huart->Instance != USART1) ||
      (data == NULL) || (size != 1U))
  {
    return HAL_ERROR;
  }
  if (huart->RxXferCount != 0U)
  {
    return HAL_BUSY;
  }

  /* 保存单字节接收目标并打开 RXNE 中断，等待 USART1 收到字符。 */
  huart->pRxBuffPtr = data;
  huart->RxXferCount = size;
  huart->Instance->CR1 |= USART_CR1_RXNEIE;
  return HAL_OK;
}

/**
  * @brief USART1 接收中断服务程序，将数据交给接收完成回调。
  */
void USART1_IRQHandler(void)
{
  /* 读取状态寄存器后读数据寄存器，可清除 STM32 USART 接收及错误标志。 */
  const uint32_t status = USART1->SR;
  if ((status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE |
                 USART_SR_FE | USART_SR_PE)) != 0U)
  {
    if ((status & USART_SR_RXNE) != 0U)
    {
      /* 修改说明：检测帧错误、噪声、溢出或奇偶校验错误时丢弃该字节并恢复接收。 */
      if ((status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U)
      {
        (void)USART1->DR;
        huart1.RxXferCount = 0U;
        USART1->CR1 &= ~USART_CR1_RXNEIE;
        (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_char, 1U);
        return;
      }

      /* 若单字节接收已挂接，将 DR 中的字符写入缓冲区并完成本次接收。 */
      const uint8_t received = (uint8_t)(USART1->DR & 0xFFU);
      if (huart1.RxXferCount != 0U)
      {
        *huart1.pRxBuffPtr = received;
        huart1.RxXferCount = 0U;
        USART1->CR1 &= ~USART_CR1_RXNEIE;
        HAL_UART_RxCpltCallback(&huart1);
      }
    }
    else
    {
      /* 读取 DR 清除错误条件，避免错误标志导致中断反复进入。 */
      (void)USART1->DR;
    }
  }
}

/**
  * @brief HAL UART 单字节接收完成回调，解析命令并重启下一次接收。
  * @param huart 完成本次接收的 UART 句柄。
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* 仅处理 USART1 收到的字节，并将最新值保存到全局字符变量。 */
  if ((huart != NULL) && (huart->Instance == USART1))
  {
    /* 接收缓冲区 rx_char 已由中断适配写入，按字符协议立即处理命令。 */
    Bluetooth_ProcessCmd(rx_char);

    /* 重新挂接单字节接收，以便 USART1 持续接收后续蓝牙命令。 */
    if (HAL_UART_Receive_IT(huart, (uint8_t *)&rx_char, 1U) != HAL_OK)
    {
      /* 无法恢复接收时关闭电机，防止蓝牙失联后继续运行。 */
      Motor_SetSpeed(0, 0);
      car_mode = 0U;
    }
  }
}

/**
  * @brief 根据手机蓝牙收到的字符更新运行模式或控制电机。
  * @param cmd USART1 收到的蓝牙命令字符。
  */
void Bluetooth_ProcessCmd(char cmd)
{
  const uint32_t now = HAL_GetTick();

  /* 仅有效控制命令刷新 30 秒空闲计时；未知字节不算有效操作。 */
  if ((cmd == 'W') || (cmd == 'A') || (cmd == 'S') || (cmd == 'D') ||
      (cmd == 'F') || (cmd == 'V') || (cmd == 'P') || (cmd == 'U') ||
      (cmd == 'N') || (cmd == 'O') || (cmd == 'X') || (cmd == 'C') ||
      (cmd == 'R') || (cmd == 'E') || (cmd == 'T') || (cmd == 'J') ||
      ((cmd >= '0') && (cmd <= '3')))
  {
    last_cmd_tick = now;
    if ((cmd != 'C') && (cmd != 'R') && (cmd != 'E') &&
        (cmd != 'T') && (cmd != 'J') && (cmd != 'O') &&
        !((cmd >= '0') && (cmd <= '3')))
    {
      Buzzer_Beep(50U);
    }
  }

  /*
   * 全局模式键优先级最高：必须在示教录制、舞蹈和回放拦截之前处理。
   * 这样 0/1/2/3 可在 UART 接收中断中立即取消搬运寻物、示教回放及舞蹈，
   * 并停车后切换模式，不会被任何当前动作状态锁住。
   */
  if ((cmd >= '0') && (cmd <= '3'))
  {
    teach_recording = 0U;
    teach_playing = 0U;
    teach_dispatching = 0U;
    dance_active = 0U;
    dance_state = DANCE_STATE_OFF;
    carry_state = CARRY_STATE_INACTIVE;
    carry_servo_request = '\0';
    carry_servo_was_up = 0U;
    manual_obstacle_active = 0U;
    manual_motion = '\0';
    manual_left_speed = 0;
    manual_right_speed = 0;
    turn_bias = 0;
    follow_speed = 0;
    follow_search_active = 0U;
    obstacle_task_pause_tick = now;

    /* 停止动作时冻结正在平滑移动的舵机目标，保留其当前位置。 */
    Servo_RotateSmoothly(Servo_GetAngle());

    /* 模式切换立即取消旧动作及其蜂鸣，不等待主循环下一次调度。 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    buzzer_active = 0U;
    Motor_SetSpeed(0, 0);

    car_mode = (uint8_t)(cmd - '0');
    if (car_mode == 0U)
    {
      base_speed = 0;
    }
    else if (((car_mode == 1U) || (car_mode == 2U)) &&
             (base_speed == 0))
    {
      base_speed = ClampBaseSpeed(50);
    }
    return;
  }

  /*
   * O 是搬运子状态退出键，也必须越过舞蹈/示教的命令拦截。
   * 退出后停车并关闭搬运报警；舵机位置保留，普通手动模式可继续使用。
   */
  if ((cmd == 'O') && (carry_state != CARRY_STATE_INACTIVE))
  {
    carry_state = CARRY_STATE_INACTIVE;
    carry_servo_request = '\0';
    carry_servo_was_up = 0U;
    base_speed = ClampBaseSpeed(50);
    turn_bias = 0;
    manual_motion = '\0';
    manual_left_speed = 0;
    manual_right_speed = 0;
    manual_obstacle_active = 0U;
    teach_playing = 0U;
    teach_recording = 0U;
    dance_active = 0U;
    dance_state = DANCE_STATE_OFF;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    buzzer_active = 0U;
    Motor_SetSpeed(0, 0);
    return;
  }

  if ((teach_recording != 0U) && (teach_dispatching == 0U) &&
      (car_mode == 3U))
  {
    Teach_RecordCommand(cmd, now);
  }

  /* 跳舞中忽略其他动作指令，X 除外，以确保舞蹈可随时停止。 */
  if ((dance_active != 0U) && (cmd != 'X'))
  {
    return;
  }

  /* 回放期间只有回放调度器和紧急停止可改变运动状态。 */
  if ((teach_playing != 0U) && (teach_dispatching == 0U) &&
      (cmd != 'X') && (cmd != 'J') && (cmd != 'T'))
  {
    return;
  }

  switch (cmd)
  {
    case 'X':
      /* X 是通用紧急停车；数字模式键另有更高优先级并会完整退出搬运状态。 */
      base_speed = 0;
      turn_bias = 0;
      teach_playing = 0U;
      dance_active = 0U;
      dance_state = DANCE_STATE_OFF;
      manual_motion = '\0';
      manual_left_speed = 0;
      manual_right_speed = 0;
      Motor_SetSpeed(0, 0);
      break;

    case 'F':
      /* F 增加速度；有符号速度始终受 Flash 保存的最大速度约束。 */
      base_speed = ClampBaseSpeed((int32_t)base_speed + 10);
      if ((car_mode == 3U) && (manual_motion != '\0'))
      {
        ApplyManualMotion();
      }
      break;

    case 'V':
      base_speed = ClampBaseSpeed((int32_t)base_speed - 10);
      if ((car_mode == 3U) && (manual_motion != '\0'))
      {
        ApplyManualMotion();
      }
      break;

    case 'C':
      /* 将运行中的配置保存请求交给主循环，避免在 UART 中断内擦写 Flash。 */
      if (car_mode == 3U)
      {
        config_save_request = 1U;
      }
      break;

    case 'R':
      if ((car_mode == 3U) && (carry_state == CARRY_STATE_INACTIVE))
      {
        teach_playing = 0U;
        teach_record_count = 0U;
        teach_play_index = 0U;
        teach_last_event_tick = now;
        teach_recording = 1U;
        dance_active = 0U;
        dance_state = DANCE_STATE_OFF;
      }
      break;

    case 'E':
      if (teach_recording != 0U)
      {
        if (teach_record_count != 0U)
        {
          const uint16_t last_index = (uint16_t)(teach_record_count - 1U);
          teach_record[last_index][1] +=
              (uint32_t)(now - teach_last_event_tick);
        }
        teach_recording = 0U;
      }
      break;

    case 'T':
      if ((car_mode == 3U) && (carry_state == CARRY_STATE_INACTIVE) &&
          (teach_recording == 0U) && (teach_record_count != 0U))
      {
        dance_active = 0U;
        dance_state = DANCE_STATE_OFF;
        teach_playing = 1U;
        teach_play_index = 0U;
        teach_play_deadline = now;
        manual_motion = '\0';
        Motor_SetSpeed(0, 0);
      }
      break;

    case 'J':
      if ((car_mode == 3U) && (carry_state == CARRY_STATE_INACTIVE))
      {
        teach_recording = 0U;
        teach_playing = 0U;
        dance_active = 0U;
        dance_state = DANCE_STATE_OFF;
        Dance_Start();
      }
      break;

    case 'P':
      /* P 仅在普通模式 3 中生效；搬运自动寻找保留独立的慢速搜索速度。 */
      if ((car_mode == 3U) && (carry_state == CARRY_STATE_INACTIVE))
      {
        carry_state = CARRY_STATE_WAITING;
        carry_entry_tick = HAL_GetTick();
        carry_alarm_tick = carry_entry_tick;
        carry_servo_was_up = (Servo_GetAngle() >= 90U) ? 1U : 0U;
        carry_servo_request = '\0';
        base_speed = ClampBaseSpeed(10);
        turn_bias = 0;
        manual_motion = '\0';
        manual_left_speed = 0;
        manual_right_speed = 0;
        Motor_SetSpeed(0, 0);
      }
      break;

    case 'U':
    case 'N':
      /* U/N 只登记舵机目标，由主循环非阻塞地平滑执行并保持目标位置。 */
      if (car_mode == 3U)
      {
        carry_servo_request = cmd;
        if (carry_state != CARRY_STATE_INACTIVE)
        {
          carry_state = CARRY_STATE_MANUAL;
          carry_servo_was_up = (cmd == 'U') ? 1U : 0U;
          manual_motion = '\0';
          manual_left_speed = 0;
          manual_right_speed = 0;
          Motor_SetSpeed(0, 0);
        }
      }
      break;

    case 'O':
      /* O 退出搬运子状态，恢复约定的普通手动速度，不重置舵机位置。 */
      if (carry_state != CARRY_STATE_INACTIVE)
      {
        carry_state = CARRY_STATE_INACTIVE;
        carry_servo_request = '\0';
        carry_servo_was_up = 0U;
        base_speed = ClampBaseSpeed(50);
        turn_bias = 0;
        manual_motion = '\0';
        manual_left_speed = 0;
        manual_right_speed = 0;
        manual_obstacle_active = 0U;
        Motor_SetSpeed(0, 0);
      }
      break;

    case 'W':
      /* W 无条件增加基础速度；避障只覆盖电机输出，不得阻止速度累加。 */
      base_speed = ClampBaseSpeed((int32_t)base_speed + 10);
      car_mode = 3U;
      if (carry_state != CARRY_STATE_INACTIVE)
      {
        carry_state = CARRY_STATE_MANUAL;
      }
      manual_motion = 'W';
      Manual_UpdateOutput();
      break;

    case 'S':
      /* S 降低基础速度，低至 -100 后饱和，立即合成并输出左右轮速度。 */
      base_speed = ClampBaseSpeed((int32_t)base_speed - 10);
      car_mode = 3U;
      if (carry_state != CARRY_STATE_INACTIVE)
      {
        carry_state = CARRY_STATE_MANUAL;
      }
      manual_motion = 'S';
      Manual_UpdateOutput();
      break;

    case 'A':
      /* A 增大左转偏置；独立限制到 +30，并立即输出差速合成结果。 */
      turn_bias = (turn_bias <= 25) ? (int16_t)(turn_bias + 5) : 30;
      car_mode = 3U;
      if (carry_state != CARRY_STATE_INACTIVE)
      {
        carry_state = CARRY_STATE_MANUAL;
      }
      manual_motion = 'A';
      Manual_UpdateOutput();
      break;

    case 'D':
      /* D 减小转向偏置；独立限制到 -30，并立即输出差速合成结果。 */
      turn_bias = (turn_bias >= -25) ? (int16_t)(turn_bias - 5) : -30;
      car_mode = 3U;
      if (carry_state != CARRY_STATE_INACTIVE)
      {
        carry_state = CARRY_STATE_MANUAL;
      }
      manual_motion = 'D';
      Manual_UpdateOutput();
      break;

    default:
      /* 未定义的命令不改变模式和电机状态，但已通过蜂鸣器提示收到字符。 */
      break;
  }
}

/**
  * @brief 根据保存的手动方向和基础速度更新左右轮速度。
  */
static void ApplyManualMotion(void)
{
  /* 不存在有效手动方向时保持停车，防止切入手动模式后意外自行移动。 */
  if (manual_motion == '\0')
  {
    manual_left_speed = 0;
    manual_right_speed = 0;
  }
  else
  {
    /* 按需求合成差速轮速：左轮减偏置、右轮加偏置。 */
    int16_t left_speed = (int16_t)(base_speed - turn_bias);
    int16_t right_speed = (int16_t)(base_speed + turn_bias);

    /* 搬运手动驾驶与普通模式 3 共用同一限幅及差速计算。 */
    left_speed = ClampMotorSpeed(left_speed);
    right_speed = ClampMotorSpeed(right_speed);

    manual_left_speed = left_speed;
    manual_right_speed = right_speed;
  }

}

/**
  * @brief 根据最新速度及转向偏置持续更新手动电机输出。
  */
static void Manual_UpdateOutput(void)
{
  /* 防止主循环合成轮速时被 UART 打断，之后再写入过期的手动目标。 */
  const uint32_t saved_primask = __get_PRIMASK();
  __disable_irq();
  ApplyManualMotion();
  Obstacle_SetSpeed(manual_left_speed, manual_right_speed);
  __set_PRIMASK(saved_primask);
}

/* 动态边界始终以期望速度计算，避免减速后边界缩小造成反复加减速。 */
static float Obstacle_GetSafeDistance(int16_t requested_speed)
{
  int32_t magnitude = (int32_t)requested_speed;
  if (magnitude < 0)
  {
    magnitude = -magnitude;
  }
  return OBSTACLE_STOP_DISTANCE_CM + ((float)magnitude / 5.0f);
}

/* 距离对应的轮速比例：边界处为 1，15 cm 处为 0，区间内线性变化。 */
static float Obstacle_GetScale(float distance_cm, int16_t requested_speed)
{
  if (!(distance_cm > 0.0f))
  {
    return 1.0f; /* 保留超时按无障碍处理的约定。 */
  }
  if (distance_cm <= OBSTACLE_STOP_DISTANCE_CM)
  {
    return 0.0f;
  }

  const float safe_distance = Obstacle_GetSafeDistance(requested_speed);
  if (distance_cm >= safe_distance)
  {
    /* 速度为 0 时安全距离为 15，此分支也避免除以 0。 */
    return 1.0f;
  }

  return (distance_cm - OBSTACLE_STOP_DISTANCE_CM) /
         (safe_distance - OBSTACLE_STOP_DISTANCE_CM);
}

/* 自动、手动、示教和跳舞共用的输出入口；搬运与跟随保持原有控制。 */
static void Obstacle_SetSpeed(int16_t left_speed, int16_t right_speed)
{
  /* 同时保护状态读取及左右轮写入，避免 UART 在两轮之间覆盖输出。 */
  const uint32_t saved_primask = __get_PRIMASK();
  __disable_irq();
  if ((car_mode == 1U) ||
      ((car_mode == 3U) && (carry_state == CARRY_STATE_INACTIVE)))
  {
    /* 每次按 W/A/S/D 都用最新基础速度重算，不能绕过减速或停车。 */
    const float scale = Obstacle_GetScale(obstacle_distance_cm, base_speed);
    left_speed = (int16_t)((float)left_speed * scale);
    right_speed = (int16_t)((float)right_speed * scale);
  }
  else if (car_mode == 0U)
  {
    left_speed = 0;
    right_speed = 0;
  }
  Motor_SetSpeed(left_speed, right_speed);
  __set_PRIMASK(saved_primask);
}

/**
  * @brief 将电机速度限制在驱动支持的 -100 至 100 范围。
  * @param speed 未限幅的单侧电机目标速度。
  * @return 限幅后的电机速度。
  */
static int16_t ClampMotorSpeed(int16_t speed)
{
  const int16_t maximum = (int16_t)app_config.max_speed;
  if (speed > maximum)
  {
    return maximum;
  }
  if (speed < (int16_t)-maximum)
  {
    return (int16_t)-maximum;
  }
  return speed;
}

/**
  * @brief 根据已加载的配置限制基础速度，允许前进和倒车。
  */
static int16_t ClampBaseSpeed(int32_t speed)
{
  int32_t maximum = (int32_t)app_config.max_speed;
  if (speed > maximum)
  {
    return (int16_t)maximum;
  }
  if (speed < -maximum)
  {
    return (int16_t)-maximum;
  }
  return (int16_t)speed;
}

/**
  * @brief 记录一条手动/舵机命令，并在下一条操作到达时结算其持续时间。
  */
static void Teach_RecordCommand(char cmd, uint32_t now)
{
  if ((cmd != 'W') && (cmd != 'A') && (cmd != 'S') &&
      (cmd != 'D') && (cmd != 'U') && (cmd != 'N'))
  {
    return;
  }

  /* 数组满后自动结束录制，绝不写入第 201 项。 */
  if (teach_record_count >= TEACH_RECORD_CAPACITY)
  {
    const uint16_t last_index =
        (uint16_t)(TEACH_RECORD_CAPACITY - 1U);
    teach_record[last_index][1] =
        (uint32_t)(now - teach_last_event_tick);
    teach_recording = 0U;
    return;
  }

  if (teach_record_count != 0U)
  {
    const uint16_t previous_index = (uint16_t)(teach_record_count - 1U);
    teach_record[previous_index][1] =
        (uint32_t)(now - teach_last_event_tick);
  }

  teach_record[teach_record_count][0] = (uint32_t)(uint8_t)cmd;
  teach_record[teach_record_count][1] = 0U;
  teach_record_count++;
  teach_last_event_tick = now;
}

/**
  * @brief 按非阻塞毫秒节拍回放示教记录中的操作及持续时间。
  */
static void Teach_Update(void)
{
  if ((teach_playing == 0U) || (car_mode != 3U) ||
      (carry_state != CARRY_STATE_INACTIVE) ||
      (dance_active != 0U))
  {
    return;
  }

  if ((int32_t)(HAL_GetTick() - teach_play_deadline) < 0)
  {
    return;
  }

  if (teach_play_index >= teach_record_count)
  {
    teach_playing = 0U;
    teach_play_index = 0U;
    manual_motion = '\0';
    manual_left_speed = 0;
    manual_right_speed = 0;
    Motor_SetSpeed(0, 0);
    return;
  }

  const uint16_t current_index = teach_play_index;
  teach_dispatching = 1U;
  Bluetooth_ProcessCmd((char)(uint8_t)teach_record[current_index][0]);
  teach_dispatching = 0U;
  teach_play_index = (uint16_t)(current_index + 1U);
  teach_play_deadline =
      HAL_GetTick() + teach_record[current_index][1];
}

/**
  * @brief 启动非阻塞跳舞序列；当前动作将在后续主循环中执行。
  */
static void Dance_Start(void)
{
  car_mode = 3U;
  dance_servo_cycles = 0U;
  dance_state = DANCE_STATE_LEFT;
  dance_state_tick = HAL_GetTick();
  dance_active = 1U;
  manual_motion = '\0';
  manual_left_speed = 0;
  manual_right_speed = 0;
}

/**
  * @brief 立即取消跳舞并停止电机。
  */
static void Dance_Stop(void)
{
  dance_active = 0U;
  dance_state = DANCE_STATE_OFF;
  Motor_SetSpeed(0, 0);
}

/**
  * @brief 非阻塞更新跳舞动作；所有阶段均由 HAL_GetTick() 定时。
  */
static void Dance_Update(void)
{
  const uint32_t now = HAL_GetTick();

  switch (dance_state)
  {
    case DANCE_STATE_LEFT:
      {
        const int16_t turn_speed =
            (app_config.max_speed < 30U) ? (int16_t)app_config.max_speed : 30;
        Obstacle_SetSpeed((int16_t)-turn_speed, turn_speed);
      }
      if ((uint32_t)(now - dance_state_tick) >= 1000U)
      {
        dance_state = DANCE_STATE_RIGHT;
        dance_state_tick = now;
      }
      break;

    case DANCE_STATE_RIGHT:
      {
        const int16_t turn_speed =
            (app_config.max_speed < 30U) ? (int16_t)app_config.max_speed : 30;
        Obstacle_SetSpeed(turn_speed, (int16_t)-turn_speed);
      }
      if ((uint32_t)(now - dance_state_tick) >= 1000U)
      {
        Motor_SetSpeed(0, 0);
        dance_state = DANCE_STATE_SERVO_UP;
        Servo_RotateSmoothly(90U);
      }
      break;

    case DANCE_STATE_SERVO_UP:
      Motor_SetSpeed(0, 0);
      if (Servo_GetAngle() >= 90U)
      {
        dance_state = DANCE_STATE_SERVO_DOWN;
        Servo_RotateSmoothly(0U);
      }
      break;

    case DANCE_STATE_SERVO_DOWN:
      Motor_SetSpeed(0, 0);
      if (Servo_GetAngle() == 0U)
      {
        dance_servo_cycles++;
        if (dance_servo_cycles < 3U)
        {
          dance_state = DANCE_STATE_SERVO_UP;
          Servo_RotateSmoothly(90U);
        }
        else
        {
          dance_state = DANCE_STATE_BEEP_FIRST_ON;
          Buzzer_Beep(120U);
        }
      }
      break;

    case DANCE_STATE_BEEP_FIRST_ON:
      Motor_SetSpeed(0, 0);
      if (buzzer_active == 0U)
      {
        dance_state = DANCE_STATE_BEEP_FIRST_GAP;
        dance_state_tick = now;
      }
      break;

    case DANCE_STATE_BEEP_FIRST_GAP:
      if ((uint32_t)(now - dance_state_tick) >= 180U)
      {
        dance_state = DANCE_STATE_BEEP_SECOND_ON;
        Buzzer_Beep(120U);
      }
      break;

    case DANCE_STATE_BEEP_SECOND_ON:
      if (buzzer_active == 0U)
      {
        dance_state = DANCE_STATE_DONE;
      }
      break;

    case DANCE_STATE_DONE:
    case DANCE_STATE_OFF:
    default:
      Dance_Stop();
      break;
  }
}

/**
  * @brief 更新模式 3 内的搬运状态机，控制寻物、靠近、舵机和报警。
  * @param distance_cm 当前未经滤波的超声波距离，单位为厘米。
  */
static void Carry_Update(float distance_cm)
{
  /* 在主循环执行平滑舵机动作，避免在 UART 中断中调用 HAL_Delay。 */
  const char servo_request = carry_servo_request;
  if (servo_request != '\0')
  {
    carry_servo_request = '\0';
    if (servo_request == 'U')
    {
      Servo_PushUp();
    }
    else
    {
      Servo_PushDown();
      carry_servo_was_up = 0U;
    }
  }

  switch (carry_state)
  {
    case CARRY_STATE_WAITING:
      /* 进入后先等待 5 秒，期间停车并允许用户先手动操作舵机。 */
      if ((uint32_t)(HAL_GetTick() - carry_entry_tick) < 5000U)
      {
        if (manual_motion != '\0')
        {
          Manual_UpdateOutput();
        }
        else
        {
          Motor_SetSpeed(0, 0);
        }
        break;
      }

      /* 舵机入场时已抬起则禁止自动找物，转入等待用户放下的报警态。 */
      if (carry_servo_was_up != 0U)
      {
        carry_state = CARRY_STATE_SERVO_ALERT;
        carry_alarm_tick = HAL_GetTick();
        if (manual_motion != '\0')
        {
          Manual_UpdateOutput();
        }
        else
        {
          Motor_SetSpeed(0, 0);
        }
        break;
      }

      carry_state = CARRY_STATE_SEARCHING;
      /* 继续执行寻物状态逻辑，无需额外等待一个主循环周期。 */
      /* fall through */

    case CARRY_STATE_SEARCHING:
      /* 有效距离小于 20 cm 表示已找到目标，随后进入慢速靠近阶段。 */
      if ((distance_cm > 0.0f) && (distance_cm < 20.0f))
      {
        carry_state = CARRY_STATE_APPROACHING;
        Motor_SetSpeed(0, 0);
      }
      else
      {
        /* 未找到目标时原地慢转：左轮反转，右轮正转。 */
        const int16_t carry_speed = ClampBaseSpeed(10);
        Motor_SetSpeed((int16_t)-carry_speed, carry_speed);
      }
      break;

    case CARRY_STATE_APPROACHING:
      if ((distance_cm > 0.0f) && (distance_cm < 10.0f))
      {
        /* 到达目标距离后停车，平滑抬起推板并长鸣提示完成。 */
        Motor_SetSpeed(0, 0);
        carry_state = CARRY_STATE_COMPLETE;
        Servo_PushUp();
        Buzzer_Beep(1000U);
      }
      else if (distance_cm > 0.0f)
      {
        /* 只有测距有效时才向前挪动，防止超时读数导致无条件前冲。 */
        const int16_t carry_speed = ClampBaseSpeed(10);
        Motor_SetSpeed(carry_speed, carry_speed);
      }
      else
      {
        Motor_SetSpeed(0, 0);
      }
      break;

    case CARRY_STATE_SERVO_ALERT:
      /* 舵机仍抬起时周期短鸣，持续提示放下；不自动操作舵机或寻物。 */
      if (((uint32_t)(HAL_GetTick() - carry_alarm_tick) >= 400U) &&
          (buzzer_active == 0U))
      {
        Buzzer_Beep(100U);
        carry_alarm_tick = HAL_GetTick();
      }

      if (manual_motion != '\0')
      {
        Manual_UpdateOutput();
      }
      else
      {
        Motor_SetSpeed(0, 0);
      }
      break;

    case CARRY_STATE_MANUAL:
      /* 手动舵机操作后保持搬运状态，继续允许限速手动驾驶且不避障。 */
      Manual_UpdateOutput();
      break;

    case CARRY_STATE_COMPLETE:
      /* 自动搬运完成后保持搬运显示并停车，等待后续人工指令。 */
      Motor_SetSpeed(0, 0);
      break;

    case CARRY_STATE_INACTIVE:
    default:
      Motor_SetSpeed(0, 0);
      break;
  }
}

/**
  * @brief 根据距离平滑控制跟随速度；目标丢失时原地缓慢旋转搜索。
  * @param distance_cm 当前未滤波的超声波距离，单位为厘米。
  */
static void Follow_Control(float distance_cm)
{
  if (!(distance_cm > 0.0f))
  {
    follow_search_active = 1U;
  }

  /* 丢失后持续搜索，只有重新测得 0 至 50 厘米内的目标才退出搜索。 */
  if ((!(distance_cm > 0.0f)) ||
      ((follow_search_active != 0U) && (distance_cm >= 50.0f)))
  {
    int16_t search_speed = (int16_t)(base_speed / 2);
    if (search_speed < 15)
    {
      search_speed = 15;
    }
    else if (search_speed > 25)
    {
      search_speed = 25;
    }
    if ((uint16_t)search_speed > app_config.max_speed)
    {
      search_speed = (int16_t)app_config.max_speed;
    }

    follow_speed = 0;
    follow_ramp_tick = HAL_GetTick();
    Motor_SetSpeed((int16_t)-search_speed, search_speed);
    return;
  }

  follow_search_active = 0U;

  /* 目标小于 15 厘米时立即以基础速度后退，避免等待渐变造成碰撞。 */
  if (distance_cm < 15.0f)
  {
    follow_speed = (int16_t)-base_speed;
    follow_ramp_tick = HAL_GetTick();
    Motor_SetSpeed(follow_speed, follow_speed);
    return;
  }

  /* 15 至 30 厘米停车，其中 20 至 30 厘米是用于抑制抖动的保持死区。 */
  if (distance_cm <= 30.0f)
  {
    follow_speed = 0;
    follow_ramp_tick = HAL_GetTick();
    Motor_SetSpeed(0, 0);
    return;
  }

  /* 30 至 50 厘米以半速靠近；超过 50 厘米时目标速度提高到基础速度。 */
  const int16_t target_speed =
      (distance_cm > 50.0f) ? base_speed : (int16_t)(base_speed / 2);
  if (target_speed == 0)
  {
    follow_speed = 0;
    Motor_SetSpeed(0, 0);
    return;
  }

  /* 每 100 毫秒最多改变 5 个速度单位，避免远距离时突然全速冲出。 */
  if ((uint32_t)(HAL_GetTick() - follow_ramp_tick) >= 100U)
  {
    if (follow_speed < target_speed)
    {
      const int16_t remaining =
          (int16_t)(target_speed - follow_speed);
      follow_speed =
          (remaining > 5) ? (int16_t)(follow_speed + 5) : target_speed;
    }
    else if (follow_speed > target_speed)
    {
      const int16_t remaining =
          (int16_t)(follow_speed - target_speed);
      follow_speed =
          (remaining > 5) ? (int16_t)(follow_speed - 5) : target_speed;
    }
    follow_ramp_tick = HAL_GetTick();
  }

  Motor_SetSpeed(follow_speed, follow_speed);
}

/**
  * @brief 控制 PB5 蜂鸣器响指定时长。
  * @param duration_ms 蜂鸣持续时间，单位为毫秒。
  */
static void Buzzer_Beep(uint32_t duration_ms)
{
  /* PB5 是工程预留的 GPIO 输出，按高电平有效方式启动蜂鸣器。 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  /* 根据当前系统节拍记录关闭时间，避免使用阻塞延时。 */
  buzzer_off_tick = HAL_GetTick() + duration_ms;
  buzzer_active = 1U;
}

/**
  * @brief 检查蜂鸣持续时间并在到期时关闭蜂鸣器。
  */
static void Buzzer_Update(void)
{
  /* 用有符号差值判断节拍到期，正确处理 HAL 毫秒计数器回绕。 */
  if ((buzzer_active != 0U) &&
      ((int32_t)(HAL_GetTick() - buzzer_off_tick) >= 0))
  {
    /* 拉低 PB5 关闭蜂鸣器，并清除鸣叫状态。 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    buzzer_active = 0U;
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /* 使能 GPIOC 外设时钟，确保可以访问 PC13 板载 LED 引脚。 */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  /* 电机控制脚和 Trig 上电默认拉低；蜂鸣器另行初始化并保持默认关闭。 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  /* 初始化 LED 输出锁存为高电平，避免配置输出时板载 LED 意外点亮。 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  /* 选择 PC13，作为板载 LED 的控制引脚。 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  /* 配置为推挽输出，以便主动输出点亮或熄灭 LED 所需的电平。 */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  /* 不启用内部上拉或下拉电阻。 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  /* 使用低速输出，满足 LED 控制需求并减少不必要的开关噪声。 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  /* 将上述配置应用到 GPIOC 的 PC13 引脚。 */
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11 电机控制，PB12 Trig */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PB5 专用于高电平触发的有源蜂鸣器，单独配置为推挽 GPIO 输出。 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* 将 PB13 配置为 GPIO 输入，用于直接检测 Echo 引脚的高低电平。 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  /* 设为普通输入模式，不启用输出驱动。 */
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  /* 不启用内部上拉或下拉，由外部信号决定 PB13 电平。 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  /* 将输入配置写入 GPIOB 的 PB13 引脚。 */
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
  * @param  line: assert_param error line source number
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
