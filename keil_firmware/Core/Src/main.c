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
#include <string.h>
#include <stdio.h>

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

#define DEBUG_UART_ENABLED       0U

#define XBEE_SLEEP_GPIO_PORT     GPIOB
#define XBEE_SLEEP_PIN           GPIO_PIN_7

/*
 * The upper two response bits contain the sensor ID, so valid IDs are 0..3.
 * Each node uses a separate response slot after the broadcast request.
 */
#define SENSOR_ID                1U
#define RESPONSE_SLOT_MS         10U

#define NR                       1000U
#define FRAME                    (2U * NR)
#define NS                       256U
#define NG                       64U

#define ADC_FS                   4095.0f
#define VREF                     3.300f
#define I_FRONTEND               40e-9f
#define TM_SEC                   1.0e-3f

#define ADC_FRAME_TIMEOUT_MS     20U
#define CAP_SCALE_PER_PF         10.0f
#define CAP_PAYLOAD_MAX          0x3FFEU
#define CAP_PAYLOAD_ERROR        0x3FFFU

#if SENSOR_ID > 3U
#error "SENSOR_ID must fit in two bits (0..3)."
#endif

#if ((2U * NG + NS) >= NR)
#error "NR, NS and NG do not define valid slope-measurement segments."
#endif

static volatile uint16_t adcBuf[FRAME];
static volatile uint8_t adcFrameReady = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

static float avg_codes_u16(const volatile uint16_t *buf,
                           uint32_t start,
                           uint32_t len);
static float compute_cp_farad(const volatile uint16_t *buf);
static HAL_StatusTypeDef acquire_adc_frame(void);
static void stop_acquisition(void);
static uint16_t pack_measurement(float cp_farad, uint8_t valid);
static void debug_report(float cp_farad,
                         uint8_t valid,
                         HAL_StatusTypeDef acquisition_status);
/* USER CODE END PFP */

uint8_t request_byte = 0U;
int main(void)
{
  // uint8_t request_byte = 0U;

  HAL_Init();
  SystemClock_Config();

  /*
   * Peripherals are configured once. They are started and stopped for each
   * measurement, but their MX_* initialization functions are not called again.
   */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();

#if DEBUG_UART_ENABLED
  MX_USART2_UART_Init();
#endif

  /* Low means "awake" if XBee pin-sleep mode is ever enabled. */
  HAL_GPIO_WritePin(XBEE_SLEEP_GPIO_PORT,
                    XBEE_SLEEP_PIN,
                    GPIO_PIN_RESET);

  /* Run the ADC self-calibration once, while the ADC is idle. */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

#if DEBUG_UART_ENABLED
  {
    static const char boot_message[] =
        "Boot: waiting for XBee request 'r' or 'R'.\r\n";

    (void)HAL_UART_Transmit(&huart2,
                            (uint8_t *)boot_message,
                            (uint16_t)(sizeof(boot_message) - 1U),
                            100U);
  }
#endif

  while (1)
  {
    HAL_StatusTypeDef uart_status;

    /*
     * The sensor node is event driven: it sleeps logically here until the
     * central XBee broadcasts one request byte.
     */
#if DEBUG_UART_ENABLED

    uart_status = HAL_UART_Receive(&huart2,
                                   &request_byte,
                                   1U,
                                   HAL_MAX_DELAY);
#else
    uart_status = HAL_UART_Receive(&huart1,
                                   &request_byte,
                                   1U,
                                   HAL_MAX_DELAY);
#endif
		
    if (uart_status != HAL_OK)
    {
			
#if DEBUG_UART_ENABLED
      static const char uart_error_message[] =
          "USART1 receive error; re-arming receiver.\r\n";

      (void)HAL_UART_Transmit(&huart2,
                              (uint8_t *)uart_error_message,
                              (uint16_t)(sizeof(uart_error_message) - 1U),
                              100U);
#endif
      (void)HAL_UART_AbortReceive(&huart1);
      continue;
    }

    if ((request_byte == (uint8_t)'r') ||
        (request_byte == (uint8_t)'R'))
    {
      HAL_StatusTypeDef acquisition_status;
      float cp_farad = -1.0f;
      uint8_t valid = 0U;
      uint16_t packed;
      uint8_t response[2];

      acquisition_status = acquire_adc_frame();

      if (acquisition_status == HAL_OK)
      {
        cp_farad = compute_cp_farad(adcBuf);
        valid = (cp_farad > 0.0f) ? 1U : 0U;
      }

      /*
       * Broadcast requests reach every node at almost the same time.
       * Give each sensor ID a different response slot to reduce collisions.
       */
      HAL_Delay((uint32_t)SENSOR_ID * RESPONSE_SLOT_MS);

      packed = pack_measurement(cp_farad, valid);
      response[0] = (uint8_t)(packed >> 8);
      response[1] = (uint8_t)(packed & 0xFFU);

#if DEBUG_UART_ENABLED

			debug_report(cp_farad, valid, acquisition_status);
#else			
      HAL_UART_Transmit(&huart1,
                              response,
                              (uint16_t)sizeof(response),
                              100U);
#endif
      
    }
  }
}


/**************************************************************************************/
/* Application functions                                                            */
/**************************************************************************************/

static HAL_StatusTypeDef acquire_adc_frame(void)
{
  HAL_StatusTypeDef status;
  uint32_t start_tick;

  adcFrameReady = 0U;

  /* Begin every one-shot acquisition from a known timer phase. */
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

  /*
   * Start order:
   * 1. Arm ADC + normal-mode DMA for FRAME samples.
   * 2. Arm TIM2, which generates the 2 MHz ADC trigger.
   * 3. Start TIM1 CH3N; this produces the 1 kHz excitation waveform and
   *    synchronizes TIM2 through TIM1 TRGO.
   */
  status = HAL_ADC_Start_DMA(&hadc1,
                             (uint32_t *)adcBuf,
                             FRAME);
  if (status != HAL_OK)
  {
    stop_acquisition();
    return status;
  }

  status = HAL_TIM_Base_Start(&htim2);
  if (status != HAL_OK)
  {
    stop_acquisition();
    return status;
  }

  /*
   * HAL_TIMEx_PWMN_Start() enables the complementary PWM output and starts
   * TIM1. A second HAL_TIM_Base_Start(&htim1) is unnecessary.
   */
  status = HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  if (status != HAL_OK)
  {
    stop_acquisition();
    return status;
  }

  start_tick = HAL_GetTick();

  while (adcFrameReady == 0U)
  {
    if ((HAL_GetTick() - start_tick) >= ADC_FRAME_TIMEOUT_MS)
    {
      stop_acquisition();
      return HAL_TIMEOUT;
    }
  }

  stop_acquisition();
  return HAL_OK;
}


static void stop_acquisition(void)
{
  /*
   * These calls also restore the HAL peripheral state so the next normal-mode
   * DMA acquisition can be started cleanly.
   */
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
  (void)HAL_TIM_Base_Stop(&htim2);
  (void)HAL_ADC_Stop_DMA(&hadc1);
}


static uint16_t pack_measurement(float cp_farad, uint8_t valid)
{
  uint16_t payload;

  if (valid != 0U)
  {
    float scaled = cp_farad * 1.0e12f * CAP_SCALE_PER_PF;

    if (scaled < 0.0f)
    {
      payload = 0U;
    }
    else if (scaled >= (float)CAP_PAYLOAD_MAX)
    {
      payload = CAP_PAYLOAD_MAX;
    }
    else
    {
      /* Round to the nearest 0.1 pF instead of always truncating. */
      payload = (uint16_t)(scaled + 0.5f);
    }
  }
  else
  {
    /*
     * 0x3FFF is reserved as an error payload. Valid measurements are capped
     * at 0x3FFE, so the central receiver always gets a response.
     */
    payload = CAP_PAYLOAD_ERROR;
  }

  return (uint16_t)((((uint16_t)SENSOR_ID & 0x0003U) << 14) |
                    (payload & 0x3FFFU));
}


static float compute_cp_farad(const volatile uint16_t *buf)
{
  const uint32_t s1 = NG;
  const uint32_t s2 = NR - NG - NS;
  const uint32_t off = NR;
  const uint32_t s3 = off + NG;
  const uint32_t s4 = off + NR - NG - NS;

  float a1_codes = avg_codes_u16(buf, s1, NS);
  float a2_codes = avg_codes_u16(buf, s2, NS);
  float a3_codes = avg_codes_u16(buf, s3, NS);
  float a4_codes = avg_codes_u16(buf, s4, NS);

  float a1 = a1_codes * (VREF / ADC_FS);
  float a2 = a2_codes * (VREF / ADC_FS);
  float a3 = a3_codes * (VREF / ADC_FS);
  float a4 = a4_codes * (VREF / ADC_FS);

  float falling_delta = a2 - a1;
  float rising_delta = a4 - a3;
  float denominator = rising_delta - falling_delta;
  float segment_factor;
  float cp_farad;

  if ((denominator > -1.0e-6f) && (denominator < 1.0e-6f))
  {
    return -1.0f;
  }

  segment_factor =
      (float)(NR - (2U * NG) - NS) / (float)NR;

  cp_farad =
      (I_FRONTEND * TM_SEC / denominator) * segment_factor;

  /*
   * The result sign depends on the physical CH3N polarity. Capacitance itself
   * must be positive.
   */
  if (cp_farad < 0.0f)
  {
    cp_farad = -cp_farad;
  }

  return cp_farad;
}


static float avg_codes_u16(const volatile uint16_t *buf,
                           uint32_t start,
                           uint32_t len)
{
  uint32_t sum = 0U;
  uint32_t i;

  for (i = 0U; i < len; ++i)
  {
    sum += buf[start + i];
  }

  return (float)sum / (float)len;
}


static void debug_report(float cp_farad,
                         uint8_t valid,
                         HAL_StatusTypeDef acquisition_status)
{

  char line[112];
  int length;

  if (valid != 0U)
  {
    length = snprintf(line,
                      sizeof(line),
                      "Cp=%.3f pF, sensor ID=%u\r\n",
                      (double)(cp_farad * 1.0e12f),
                      (unsigned int)SENSOR_ID);
  }
  else
  {
    length = snprintf(line,
                      sizeof(line),
                      "Cp=ERROR, sensor ID=%u, acquisition status=%d\r\n",
                      (unsigned int)SENSOR_ID,
                      (int)acquisition_status);
  }

  if (length > 0)
  {
    uint16_t tx_length =
        (length < (int)sizeof(line)) ?
        (uint16_t)length :
        (uint16_t)(sizeof(line) - 1U);

    (void)HAL_UART_Transmit(&huart2,
                            (uint8_t *)line,
                            tx_length,
                            100U);
  }

  (void)cp_farad;
  (void)valid;
  (void)acquisition_status;

}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    adcFrameReady = 1U;
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 39;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_COMBINED_RESETTRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*
   * Set the output latch low before changing PB7 to output mode. This avoids
   * a possible high pulse on XBee SLEEP_RQ during startup.
   */
  HAL_GPIO_WritePin(XBEE_SLEEP_GPIO_PORT,
                    XBEE_SLEEP_PIN,
                    GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = XBEE_SLEEP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(XBEE_SLEEP_GPIO_PORT, &GPIO_InitStruct);
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