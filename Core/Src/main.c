/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Cockpit SteeringWheelUnit Firmware
  *                   FFB via SLCAN (USART1, 2Mbaud) + CAN1 (500Kbps)
  *                   Motor: L298N + DC Motor | Sensor: Quadrature Encoder
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_PPR         400u
#define GEAR_RATIO          1u  /* Encoder directly connected, no gear */
/* Quadrature 4x counts per motor rev × gear ratio = counts per wheel rev */
#define COUNTS_PER_REV      (ENCODER_PPR * 4u * GEAR_RATIO)   /* 6400 */
#define MAX_STEERING_DEG    450.0f
/* ±450° in encoder counts: 6400 * 450/360 = 8000 */
#define MAX_ENC_COUNT       8000
#define PWM_PERIOD          999u

#define CAN_ID_STEERING     0x100u
#define CAN_ID_SWITCHES     0x101u
#define CAN_ID_FFB_DIAG     0x102u  /* FFB diagnostic */
#define CAN_ID_CAN_ESR      0x103u  /* CAN ESR diagnostic */
#define CAN_ID_FFB          0x105u
#define CAN_ID_ENC_ZERO     0x106u  /* Encoder zero calibration command */
#define CAN_ID_ANGLE_CAL    0x107u  /* Angle scale calibration: int16 known_angle×10 */
#define ANGLE_TX_PERIOD_MS  10u
#define SW_TX_PERIOD_MS     100u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static volatile int16_t  g_ffb_torque    = 0;
static volatile int32_t  g_enc_total     = 0;
static volatile int32_t  g_enc_zero      = 0;  /* Encoder zero offset (set by 0x106 cmd) */
static volatile float    g_angle_scale   = (180.0f / 520.0f); /* Angle scale: 180° physical = 520° raw */
static volatile float    g_steering_deg  = 0.0f;
static          int16_t  g_enc_prev      = 0;
static volatile uint8_t  g_slcan_open    = 0;
static volatile uint8_t  g_turn_left     = 0;
static volatile uint8_t  g_turn_right    = 0;

static volatile uint32_t g_ffb_rx_count  = 0;
static volatile int16_t  g_ffb_last_raw = 0;
static volatile uint32_t g_last_duty    = 0;

static uint8_t  g_slcan_line[32];
static uint8_t  g_slcan_pos     = 0;
static uint8_t  g_uart_rx_byte  = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Motor_SetTorque(int16_t torque);
static void Encoder_Update(void);
static void SLCAN_ProcessByte(uint8_t byte);
static void SLCAN_ParseLine(void);
static void SLCAN_SendAngle(void);
static void TurnSignal_Send(void);
static uint8_t Hex2Nibble(uint8_t c);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void Motor_SetTorque(int16_t torque)
{
    /* End-stop protection: block torque pushing further into hard stop */
    if (g_steering_deg >=  MAX_STEERING_DEG && torque > 0) torque = 0;
    if (g_steering_deg <= -MAX_STEERING_DEG && torque < 0) torque = 0;

    int32_t abs_t = (torque < 0) ? -torque : torque;
    uint32_t duty = (uint32_t)(abs_t * PWM_PERIOD / 32767u);

    /* Minimum duty: L298N needs ~15% to overcome motor friction */
    const uint32_t min_duty = 150u;
    if (abs_t > 0 && duty < min_duty) duty = min_duty;

    if (torque > 0) {
        HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
    } else if (torque < 0) {
        HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_SET);
    } else {
        /* Brake: both IN low */
        HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
        duty = 0;
    }
    g_last_duty = duty;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
}

static void Encoder_Update(void)
{
    /* Two's-complement delta handles 16-bit counter overflow correctly */
    int16_t raw   = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    int16_t delta = raw - g_enc_prev;
    g_enc_prev    = raw;

    /* Only update if encoder moved */
    if (delta != 0) {
        g_enc_total += delta;

        int32_t enc_rel = g_enc_total - g_enc_zero;
        if (enc_rel >  MAX_ENC_COUNT) enc_rel =  MAX_ENC_COUNT;
        if (enc_rel < -MAX_ENC_COUNT) enc_rel = -MAX_ENC_COUNT;

        g_steering_deg = (float)enc_rel * 360.0f / (float)COUNTS_PER_REV * g_angle_scale;
        
        /* Visual feedback: toggle LED on encoder movement */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
}

static uint8_t Hex2Nibble(uint8_t c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10u;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10u;
    return 0;
}

static void SLCAN_SendAngle(void)
{
    int16_t raw = (int16_t)(g_steering_deg * 10.0f);
    uint8_t d[2] = { (uint8_t)(raw & 0xFF), (uint8_t)((raw >> 8) & 0xFF) };

    if (g_slcan_open) {
        uint8_t buf[14];
        uint8_t len = (uint8_t)snprintf((char *)buf, sizeof(buf),
            "t%03X2%02X%02X\r", CAN_ID_STEERING, d[0], d[1]);
        HAL_UART_Transmit(&huart1, buf, len, 20);
    }

    /* Also broadcast on physical CAN bus */
    CAN_TxHeaderTypeDef hdr = {
        .StdId              = CAN_ID_STEERING,
        .IDE                = CAN_ID_STD,
        .RTR                = CAN_RTR_DATA,
        .DLC                = 2,
        .TransmitGlobalTime = DISABLE,
    };
    uint32_t mailbox;
    HAL_CAN_AddTxMessage(&hcan, &hdr, d, &mailbox);
}

static void SLCAN_ParseLine(void)
{
    const uint8_t ack = '\r';
    switch (g_slcan_line[0]) {
    case 'O':
        g_slcan_open = 1;
        HAL_UART_Transmit(&huart1, (uint8_t *)&ack, 1, 10);
        break;

    case 'C':
        g_slcan_open = 0;
        Motor_SetTorque(0);
        HAL_UART_Transmit(&huart1, (uint8_t *)&ack, 1, 10);
        break;

    case 'S':
        /* Speed command ignored: baud is fixed via UART hardware */
        HAL_UART_Transmit(&huart1, (uint8_t *)&ack, 1, 10);
        break;

    case 'V': case 'v':
        HAL_UART_Transmit(&huart1, (uint8_t *)"V0101\r", 6, 10);
        break;

    case 't': {
        /* t<3hex_id><1hex_dlc><2hex × dlc> */
        if (g_slcan_pos < 5u) break;
        uint16_t can_id = (uint16_t)(
            ((uint16_t)Hex2Nibble(g_slcan_line[1]) << 8) |
            ((uint16_t)Hex2Nibble(g_slcan_line[2]) << 4) |
             (uint16_t)Hex2Nibble(g_slcan_line[3]));
        uint8_t dlc = g_slcan_line[4] - '0';
        if (dlc > 8u || g_slcan_pos < (uint8_t)(5u + dlc * 2u)) break;

        uint8_t data[8];
        for (uint8_t i = 0; i < dlc; i++) {
            data[i] = (uint8_t)(
                (Hex2Nibble(g_slcan_line[5u + i * 2u]) << 4) |
                 Hex2Nibble(g_slcan_line[6u + i * 2u]));
        }

        if (can_id == CAN_ID_FFB && dlc >= 2u) {
            g_ffb_torque = (int16_t)(data[0] | ((uint16_t)data[1] << 8));
            Motor_SetTorque(g_ffb_torque);
        }
        HAL_UART_Transmit(&huart1, (uint8_t *)"z\r", 2, 10);
        break;
    }
    default:
        break;
    }
}

static void SLCAN_ProcessByte(uint8_t byte)
{
    if (byte == '\r' || byte == '\n') {
        if (g_slcan_pos > 0u) {
            g_slcan_line[g_slcan_pos] = '\0';
            SLCAN_ParseLine();
            g_slcan_pos = 0;
        }
    } else if (g_slcan_pos < (uint8_t)(sizeof(g_slcan_line) - 1u)) {
        g_slcan_line[g_slcan_pos++] = byte;
    }
}

static void TurnSignal_Send(void)
{
    uint16_t sw = (uint16_t)(g_turn_left | ((uint16_t)g_turn_right << 1));
    uint8_t d[2] = { (uint8_t)(sw & 0xFF), (uint8_t)(sw >> 8) };

    if (g_slcan_open) {
        uint8_t buf[14];
        uint8_t len = (uint8_t)snprintf((char *)buf, sizeof(buf),
            "t%03X2%02X%02X\r", CAN_ID_SWITCHES, d[0], d[1]);
        HAL_UART_Transmit(&huart1, buf, len, 20);
    }

    CAN_TxHeaderTypeDef hdr = {
        .StdId              = CAN_ID_SWITCHES,
        .IDE                = CAN_ID_STD,
        .RTR                = CAN_RTR_DATA,
        .DLC                = 2,
        .TransmitGlobalTime = DISABLE,
    };
    uint32_t mailbox;
    HAL_CAN_AddTxMessage(&hcan, &hdr, d, &mailbox);
}

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
  MX_CAN_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  /* Initialize encoder previous value to current counter */
  g_enc_prev = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

  /* CAN: accept all frames, then start */
  CAN_FilterTypeDef filter = {
      .FilterBank           = 0,
      .FilterMode           = CAN_FILTERMODE_IDMASK,
      .FilterScale          = CAN_FILTERSCALE_32BIT,
      .FilterIdHigh         = 0x0000,
      .FilterIdLow          = 0x0000,
      .FilterMaskIdHigh     = 0x0000,
      .FilterMaskIdLow      = 0x0000,
      .FilterFIFOAssignment = CAN_RX_FIFO0,
      .FilterActivation     = ENABLE,
  };
  HAL_CAN_ConfigFilter(&hcan, &filter);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(&hcan);

  /* === CAN SELF-TEST: TX 0x105, check if RX callback fires === */
  {
      CAN_TxHeaderTypeDef hdr = {
          .StdId              = CAN_ID_FFB,
          .IDE                = CAN_ID_STD,
          .RTR                = CAN_RTR_DATA,
          .DLC                = 2,
          .TransmitGlobalTime = DISABLE,
      };
      uint8_t test_data[2] = { 0x7F, 0x00 }; // torque = 127
      uint32_t mailbox;
      HAL_CAN_AddTxMessage(&hcan, &hdr, test_data, &mailbox);
      HAL_Delay(50); // Wait for RX callback
      // rxCount should be 1 if self-reception works
  }
  /* === END SELF-TEST === */

  /* UART: start single-byte interrupt receive */
  HAL_UART_Receive_IT(&huart1, &g_uart_rx_byte, 1);

  /* Motor idle: brake */
  HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

  /* EXTI interrupts for turn signal stalks */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  HAL_NVIC_SetPriority(EXTI3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  uint32_t last_angle_tx   = 0;
  uint32_t last_enc_update = 0;
  uint32_t last_sw_tx      = 0;
  uint32_t last_led        = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* Update encoder position every 1 ms */
    if ((now - last_enc_update) >= 1u) {
      Encoder_Update();
      last_enc_update = now;
    }

    /* Send steering angle and re-apply FFB every 10 ms (100 Hz) */
    if ((now - last_angle_tx) >= ANGLE_TX_PERIOD_MS) {
      Motor_SetTorque(g_ffb_torque);
      SLCAN_SendAngle();

      /* FFB diagnostic: send rx_count, last_raw, last_duty, steering_deg */
      {
        uint8_t dbg[8];
        dbg[0] = (uint8_t)(g_ffb_rx_count & 0xFF);
        dbg[1] = (uint8_t)((g_ffb_rx_count >> 8) & 0xFF);
        dbg[2] = (uint8_t)(g_ffb_last_raw & 0xFF);
        dbg[3] = (uint8_t)((g_ffb_last_raw >> 8) & 0xFF);
        dbg[4] = (uint8_t)(g_last_duty & 0xFF);
        dbg[5] = (uint8_t)((g_last_duty >> 8) & 0xFF);
        int16_t deg10 = (int16_t)(g_steering_deg * 10.0f);
        dbg[6] = (uint8_t)(deg10 & 0xFF);
        dbg[7] = (uint8_t)((deg10 >> 8) & 0xFF);

        CAN_TxHeaderTypeDef hdr = {
            .StdId              = CAN_ID_FFB_DIAG,
            .IDE                = CAN_ID_STD,
            .RTR                = CAN_RTR_DATA,
            .DLC                = 8,
            .TransmitGlobalTime = DISABLE,
        };
        uint32_t mailbox;
        HAL_CAN_AddTxMessage(&hcan, &hdr, dbg, &mailbox);
      }

      /* CAN ESR diagnostic: send CAN error + bus status */
      {
        uint32_t esr = hcan.Instance->ESR;
        uint32_t rf0r = hcan.Instance->RF0R;  /* RX FIFO 0: bits [1:0]=FMP (pending msgs) */
        uint32_t tsr = hcan.Instance->TSR;     /* TX status */
        uint32_t idr = GPIOB->IDR;             /* PB8=CAN_RX actual pin level */

        uint8_t dbg[8];
        dbg[0] = (uint8_t)(esr & 0xFF);        /* EWG/EPV/BOFF/LEC */
        dbg[1] = (uint8_t)((esr >> 16) & 0xFF);/* TEC */
        dbg[2] = (uint8_t)((esr >> 24) & 0xFF);/* REC */
        dbg[3] = (uint8_t)(rf0r & 0xFF);       /* RF0R: FMP0, FULL, FOVR */
        dbg[4] = (uint8_t)((tsr >> 24) & 0xFF);/* TSR: TXOK0/1/2, TME0/1/2 */
        dbg[5] = (uint8_t)((idr >> 8) & 0x01); /* PB8 pin level (0=dominant, 1=recessive) */
        dbg[6] = (uint8_t)(hcan.Instance->BTR & 0xFF); /* BTR low byte: prescaler bits */
        dbg[7] = (uint8_t)((hcan.Instance->BTR >> 8) & 0xFF); /* BTR: TS1/TS2/SJW */

        CAN_TxHeaderTypeDef hdr = {
            .StdId              = CAN_ID_CAN_ESR,
            .IDE                = CAN_ID_STD,
            .RTR                = CAN_RTR_DATA,
            .DLC                = 8,
            .TransmitGlobalTime = DISABLE,
        };
        uint32_t mailbox;
        HAL_CAN_AddTxMessage(&hcan, &hdr, dbg, &mailbox);
      }
      last_angle_tx = now;
    }

    /* Periodic turn signal heartbeat every 100 ms */
    if ((now - last_sw_tx) >= SW_TX_PERIOD_MS) {
      TurnSignal_Send();
      last_sw_tx = now;
    }

    /* LED heartbeat: toggle every 500 ms */
    if ((now - last_led) >= 500u) {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      last_led = now;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_TI1F_ED;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  {
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
      Error_Handler();
    }
  }
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  huart1.Init.BaudRate = 2000000;
  if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR_IN1_Pin|MOTOR_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOTOR_IN1_Pin MOTOR_IN2_Pin */
  GPIO_InitStruct.Pin = MOTOR_IN1_Pin|MOTOR_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  {
    GPIO_InitTypeDef g = {0};
    g.Pin  = TURN_LEFT_Pin;
    g.Mode = GPIO_MODE_IT_FALLING;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(TURN_LEFT_GPIO_Port, &g);

    g.Pin  = TURN_RIGHT_Pin;
    g.Mode = GPIO_MODE_IT_FALLING;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(TURN_RIGHT_GPIO_Port, &g);

    /* PC13: onboard LED (active LOW) — blinks when encoder moves */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    g.Pin   = GPIO_PIN_13;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); /* LED off */
  }
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* CAN RX FIFO0 callback — forwards all frames via SLCAN, handles FFB torque */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_arg)
{
  CAN_RxHeaderTypeDef rx_hdr;
  uint8_t             rx_data[8];

  if (HAL_CAN_GetRxMessage(hcan_arg, CAN_RX_FIFO0, &rx_hdr, rx_data) != HAL_OK)
    return;

  if (rx_hdr.StdId == CAN_ID_FFB && rx_hdr.DLC >= 2u) {
    g_ffb_torque = (int16_t)(rx_data[0] | ((uint16_t)rx_data[1] << 8));
    g_ffb_rx_count++;
    g_ffb_last_raw = g_ffb_torque;
    Motor_SetTorque(g_ffb_torque);
  } else if (rx_hdr.StdId == CAN_ID_ENC_ZERO) {
    /* Set current encoder position as zero (wheel must be at physical center) */
    g_enc_zero = g_enc_total;
    g_steering_deg = 0.0f;
  } else if (rx_hdr.StdId == CAN_ID_ANGLE_CAL && rx_hdr.DLC >= 2u) {
    /* Angle scale calibration: data = known angle × 10 (int16 LE).
     * Turn wheel to a known angle from zero, then send this command.
     * STM32 computes the correction factor from enc_rel vs known angle. */
    int16_t known_deg10 = (int16_t)(rx_data[0] | ((uint16_t)rx_data[1] << 8));
    int32_t enc_rel = g_enc_total - g_enc_zero;
    if (enc_rel != 0 && known_deg10 != 0) {
      float reported = (float)enc_rel * 360.0f / (float)COUNTS_PER_REV;
      g_angle_scale = (known_deg10 / 10.0f) / reported;
    }
  }

  /* Forward every received frame to host via SLCAN (always, regardless of open state) */
  {
    uint8_t buf[30];
    uint8_t len = (uint8_t)snprintf((char *)buf, sizeof(buf), "t%03X%u",
                                    (unsigned)rx_hdr.StdId, (unsigned)rx_hdr.DLC);
    for (uint8_t i = 0; i < rx_hdr.DLC; i++)
      len += (uint8_t)snprintf((char *)buf + len, sizeof(buf) - len, "%02X", rx_data[i]);
    buf[len++] = '\r';
    HAL_UART_Transmit(&huart1, buf, len, 20);
  }
}

/* UART RX complete callback — feeds SLCAN line parser */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    SLCAN_ProcessByte(g_uart_rx_byte);
    HAL_UART_Receive_IT(&huart1, &g_uart_rx_byte, 1);
  }
}

/* EXTI callback — turn signal stalks toggle state */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == TURN_LEFT_Pin) {
    g_turn_left ^= 1u;
    TurnSignal_Send();
  } else if (GPIO_Pin == TURN_RIGHT_Pin) {
    g_turn_right ^= 1u;
    TurnSignal_Send();
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
