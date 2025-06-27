/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
// Include the main STM32 HAL library header file.
#include "main.h"
// Include standard C libraries for input/output, string manipulation, and standard library functions.
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Define a magic number to verify if the RTC backup register has been set.
#define RTC_BKP_REG_MAGIC 0x32F2


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
// ADC handle for the Analog-to-Digital Converter peripheral.
ADC_HandleTypeDef hadc1;

// RTC handle for the Real-Time Clock peripheral.
RTC_HandleTypeDef hrtc;

// Timer handles for TIM1, TIM2, and TIM3 peripherals.
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

// UART handle for the USART2 peripheral for serial communication.
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Define timer channels for each LED for clarity.
#define GREEN_LED_CHANNEL TIM_CHANNEL_1  // Green LED is on TIM2, Channel 1
#define YELLOW_LED_CHANNEL TIM_CHANNEL_1 // Yellow LED is on TIM3, Channel 1
#define RED_LED_CHANNEL TIM_CHANNEL_2    // Red LED is on TIM3, Channel 2

// Define the GPIO pin and port for the pedestrian button.
#define PED_BUTTON_PIN GPIO_PIN_13
#define PED_BUTTON_PORT GPIOC

// Global State Variables to control the traffic light logic.
// Flag to indicate a pedestrian has pressed the button. 'volatile' is used because it can be changed by an interrupt.
volatile uint8_t pedestrian_request = 0;
// Flag to indicate if the ambient temperature is high.
volatile uint8_t high_temp_flag = 0;
// Flag to indicate if the green light phase is currently active.
volatile uint8_t green_phase_active = 0;
// Flag to check if the green light duration has already been reduced by a pedestrian request.
volatile uint8_t green_duration_reduced = 0;
// Flag to indicate if the red light phase is currently active.
volatile uint8_t red_phase_active = 0;
// Flag to indicate if the yellow light phase is currently active.
volatile uint8_t yellow_phase_active = 0;
// Tracks the remaining time for the green light, used for shortening the duration.
volatile uint8_t green_time_remaining = 0;

volatile uint8_t night_mode_active = 0;

// A 1-byte buffer for receiving UART data character by character via interrupt.
uint8_t uart_rx_buffer[1];
// Define the size for the buffer that assembles the incoming RTC time string.
#define RTC_UART_BUF_SIZE 32
// Buffer to store the incoming RTC time string from UART.
char rtc_uart_buffer[RTC_UART_BUF_SIZE];
// Index to keep track of the current position in the rtc_uart_buffer.
uint8_t rtc_uart_index = 0;

// Flag to indicate if the RTC time has been set via UART.
volatile uint8_t rtc_time_set = 0;

// Prototypes for HAL interrupt callback functions.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
// Prototypes for custom application functions.
void set_pwm_duty(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t duty);
void run_traffic_light(void);
void night_mode_blink(void);
void check_temperature(void);
void log_uart(const char *msg);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  // Start PWM signal generation for all LED channels.
  HAL_TIM_PWM_Start(&htim2, GREEN_LED_CHANNEL);
  HAL_TIM_PWM_Start(&htim3, YELLOW_LED_CHANNEL);
  HAL_TIM_PWM_Start(&htim3, RED_LED_CHANNEL);
  // Start TIM1 in interrupt mode. It will generate an interrupt periodically.
  //HAL_TIM_Base_Start_IT(&htim1);

  // Start UART reception with an interrupt. The MCU will expect 1 byte.
  HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 1);

  // Declare variables to hold the time and date read from the RTC.
  RTC_TimeTypeDef time;
  RTC_DateTypeDef date;
  // Get the current time from the RTC hardware.
  HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
  // Get the current date from the RTC hardware. Must be called after GetTime.
  HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

  // Create a message string to log the initial RTC time and date.
  char time_msg[64];
  snprintf(time_msg, sizeof(time_msg),
           "[RTC] Time: %02d:%02d:%02d | Date: %02d/%02d/20%02d\r\n",
           time.Hours, time.Minutes, time.Seconds,
           date.Date, date.Month, date.Year);
  // Send the initial time/date message over UART.
  log_uart(time_msg);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // The main application loop.
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // Continuously run the main traffic light state machine.
	  run_traffic_light();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  // Structures to hold oscillator and clock configuration.
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  // Enable the Power Controller (PWR) clock.
  __HAL_RCC_PWR_CLK_ENABLE();
  // Configure the voltage scaling for the regulator to ensure performance.
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  // Configure oscillators: HSI (High-Speed Internal) for the PLL and LSI (Low-Speed Internal) for the RTC.
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  // Set the HSI state to RCC_HSI_ON to enable the High-Speed Internal oscillator.
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  // Set the HSI calibration value to the default.
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  // Set the LSI state to ON for the RTC.
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  // Set the PLL state to ON to use it as the system clock source.
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  // Set the PLL source to HSI.
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  // Configure PLL factors to generate the system clock frequency
  // PLL stands for Phase-Locked Loop, which is used to generate a higher frequency clock from a lower frequency source.
  // PLLM is the division factor for the HSI clock, PLLN is the multiplication factor,
  // PLLP is the division factor for the main system clock, and PLLQ is used for USB clock.
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  // Apply the oscillator configuration.
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  // Configure the system, AHB, and APB bus clocks.
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  // Set the System Clock source to the PLL output.
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  // Set dividers for the bus clocks.
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  // Apply the clock configuration with a specified Flash latency.
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

  // Structure for ADC channel configuration.
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  // Enable the internal temperature sensor and voltage reference for the ADC.
  ADC->CCR |= ADC_CCR_TSVREFE;

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  // Set the ADC instance.
  hadc1.Instance = ADC1;
  // Configure ADC clock prescaler, resolution, alignment, etc.
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  // Initialize the ADC with the specified settings.
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  // Select the internal temperature sensor channel for conversion.
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  // Set a long sampling time for better accuracy on the temperature sensor.
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  // Configure the ADC channel.
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  // Structures for default time and date initialization.
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  // Set the RTC instance.
  hrtc.Instance = RTC;
  // Configure RTC clock dividers and format.
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  // 127 is the asynchronous prescaler value, 255 is the synchronous prescaler value.
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  // Initialize the RTC.
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  // This section is intended for checking a backup register to see if RTC is already configured,
  /* USER CODE END Check_RTC_BKUP */
  // Check if the backup register contains our magic number. This indicates
    // that the RTC has been set before and is likely running on battery power.
    if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == RTC_BKP_REG_MAGIC)
    {
    	rtc_time_set = 1;
		// RTC has been configured before, so we skip setting the default time/date
		// and just return, leaving the current RTC value intact.
		return;
    }


  /** Initialize RTC and set the Time and Date
  */
  // Set a default time (00:00:00) if not configured otherwise.
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  // Set a default date (Monday, January 1, 2000).
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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

  // Structures for timer clock source and master configuration.
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  // This timer is used to trigger periodic events (like temperature checks).
  htim1.Instance = TIM1;
  // Prescaler value to slow down the timer clock.
  htim1.Init.Prescaler = 8399;
  // Counter mode is up-counting.
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  // Period determines the frequency of the update event (interrupt).
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  // Initialize the base timer.
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  // Set the clock source to the internal clock.
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  // Configure the timer's master mode settings.
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  // Structures for timer master and PWM output channel configuration.
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  // TIM2 is used for PWM control of the green LED.
  htim2.Instance = TIM2;
  // Prescaler to set the PWM frequency.
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  // Period sets the PWM resolution (100 steps for 0-100% duty cycle).
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  // Initialize the timer in PWM mode.
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  // Configure master mode.
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  // Configure the PWM channel settings.
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0; // Initial duty cycle is 0.
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  // Apply configuration to TIM2 Channel 1 (Green LED).
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  // Perform post-initialization for timer GPIOs.
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  // Structures for timer master and PWM output channel configuration.
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  // TIM3 is used for PWM control of the yellow and red LEDs.
  htim3.Instance = TIM3;
  // Prescaler to set the PWM frequency.
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  // Period sets the PWM resolution (100 steps for 0-100% duty cycle).
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  // Initialize the timer in PWM mode.
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  // Configure master mode.
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  // Configure the PWM channel settings.
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0; // Initial duty cycle is 0.
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  // Apply configuration to TIM3 Channel 1 (Yellow LED).
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  // Apply configuration to TIM3 Channel 2 (Red LED).
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  // Perform post-initialization for timer GPIOs.
  HAL_TIM_MspPostInit(&htim3);

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
  // Configure USART2 for serial communication (e.g., with a PC).
  huart2.Instance = USART2;
  // Set baud rate, word length, stop bits, and parity.
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  // Enable both transmit (TX) and receive (RX) modes.
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  // Initialize the UART.
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  // Structure for GPIO pin configuration.
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  // Enable the clock for GPIOC and GPIOA ports.
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : BlueButton_Pin */
  // The pedestrian button is connected to PC13 (often the user button on Nucleo boards).
  GPIO_InitStruct.Pin = BlueButton_Pin;
  // Configure the pin to trigger an interrupt on a falling edge (button press).
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  // Enable the internal pull-up resistor.
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  // Initialize the GPIO pin.
  HAL_GPIO_Init(BlueButton_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  // Set the priority for the external interrupt line.
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  // Enable the external interrupt request line in the NVIC.
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief Main traffic light state machine logic.
  */
void run_traffic_light(void) {
	RTC_TimeTypeDef sTime;
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

	check_temperature();

	// Check RTC time to determine if we are in night or day mode.
	// The current time is 1:57 PM, so night_mode_active will be 0.
	if (sTime.Hours >= 15 && sTime.Hours < 17) {
		night_mode_active = 1; // SET the flag for night mode
		night_mode_blink();    // Enter the night mode function (which runs forever)
		return;
	} else {
		night_mode_active = 0; // CLEAR the flag for day mode
	}
    // --- Normal Day/Evening Cycle ---

    // Define the default durations for red and green lights.
    uint8_t red_time = 12;
    uint8_t green_time = 8;

    // If the high temperature flag is set, reduce the light durations.
    if (high_temp_flag) {
        red_time = (red_time > 2) ? red_time - 2 : 1;
        green_time = (green_time > 2) ? green_time - 2 : 1;
    }


    // --- RED PHASE ---
    log_uart("[RED] ON\r\n");
    red_phase_active = 1; // Set the red phase active flag.
    // Loop for the duration of the red light.
    for (int t = 0; t < red_time; t++) {
        char msg[50];
        snprintf(msg, sizeof(msg), "[RED] Time Left: %d sn\r\n", red_time - t);
        log_uart(msg);
        // Gradually decrease the brightness of the red light over time.
        uint8_t brightness = 100 - (t / 3) * 20;
        set_pwm_duty(&htim3, RED_LED_CHANNEL, brightness);
        HAL_Delay(1000); // Wait for one second.
    }
    set_pwm_duty(&htim3, RED_LED_CHANNEL, 0); // Turn off the red light.
    red_phase_active = 0; // Clear the red phase flag.

    // --- YELLOW PHASE ---
    log_uart("[YELLOW] Blinking\r\n");
    yellow_phase_active = 1; // Set the yellow phase active flag.
    // Blink the yellow light for a fixed duration before green.
    for (int i = 0; i < 8; i++) {
        set_pwm_duty(&htim3, YELLOW_LED_CHANNEL, 100); // Yellow ON
        HAL_Delay(125);
        set_pwm_duty(&htim3, YELLOW_LED_CHANNEL, 0);   // Yellow OFF
        HAL_Delay(125);
    }
    yellow_phase_active = 0; // Clear the yellow phase flag.

    // --- GREEN PHASE ---
    // Reset flags for the start of the green phase.
    green_phase_active = 1;
    green_duration_reduced = 0;

    // Check for a pedestrian request that arrived before the green phase started.
    if (pedestrian_request && green_time > 2) {
        green_time -= 2; // Reduce green time.
        green_duration_reduced = 1; // Mark that it has been reduced.
        log_uart("[GREEN] Pedestrian request: Duration reduced\r\n");
        pedestrian_request = 0; // Clear the request.
    }

    log_uart("[GREEN] ON\r\n");
    // Set global flags for the green phase.
    green_phase_active = 1;
    green_duration_reduced = 0;
    green_time_remaining = green_time; // Initialize the countdown variable.

    // Loop for the duration of the green light.
    for (int t = 0; t < green_time;) {
        char msg[50];
        snprintf(msg, sizeof(msg), "[GREEN] Remaining: %d sec\r\n", green_time_remaining);
        log_uart(msg);
        // Gradually decrease the brightness of the green light.
        uint8_t brightness = 100 - t * 20;
        set_pwm_duty(&htim2, GREEN_LED_CHANNEL, brightness);
        HAL_Delay(1000); // Wait one second.

        // Decrement the remaining time.
        if (green_time_remaining > 0) green_time_remaining--;

        // Check for a new pedestrian request during the green phase.
        if (pedestrian_request && !green_duration_reduced && green_time_remaining > 2) {
            // Shorten the remaining time if the button is pressed.
            green_time_remaining -= 2;
            if (green_time_remaining < 1) green_time_remaining = 1; // Ensure it doesn't become zero.
            log_uart("[GREEN] Duration shortened by button press\r\n");
            green_duration_reduced = 1; // Flag that we've shortened it once.
            pedestrian_request = 0; // Clear the request.
        }

        t++; // Increment the main loop counter.
    }

    set_pwm_duty(&htim2, GREEN_LED_CHANNEL, 0); // Turn off the green light.
    green_phase_active = 0; // Clear the green phase flag.
}

/**
  * @brief Blinks the yellow LED during night mode and checks for pedestrian requests.
  */
/**
  * @brief Handles all night mode operations in a continuous loop.
  * Blinks the yellow light, sends periodic status, and handles pedestrian requests.
  */
/**
  * @brief Handles all night mode operations in a continuous loop.
  * Blinks the yellow light, sends periodic status, and handles pedestrian requests.
  */
void night_mode_blink(void) {
    // This function now runs in an infinite loop and handles all night mode logic.
    while (1) {

        // --- 1. CHECK FOR PEDESTRIAN REQUEST ---
        if (pedestrian_request) {
            log_uart("[NIGHT] Pedestrian request. Switching to RED for cars.\r\n");
            pedestrian_request = 0; // Clear the flag immediately

            // Ensure the blinking yellow light is off before switching to red.
            set_pwm_duty(&htim3, YELLOW_LED_CHANNEL, 0);

            // --- Run a 5-second RED light cycle to stop cars ---
            for (int t = 0; t < 5; t++) {
                char msg[64];
                snprintf(msg, sizeof(msg), "[NIGHT-RED] Remaining: %d sec\r\n", 5 - t);
                log_uart(msg);

                // *** THIS IS THE KEY CHANGE: Turn the RED light ON, not green. ***
                set_pwm_duty(&htim3, RED_LED_CHANNEL, 100);
                HAL_Delay(1000);
            }
            // Turn the RED light off after 5 seconds.
            set_pwm_duty(&htim3, RED_LED_CHANNEL, 0);

            // After the red cycle, restart the main night mode loop to resume blinking yellow.
            continue;
        }

        // --- 2. IF NO REQUEST, PERFORM YELLOW BLINKING ---

        // ON-phase (1 second)
        log_uart("[NIGHT MODE] Yellow blinking...\r\n");
        set_pwm_duty(&htim3, YELLOW_LED_CHANNEL, 100);
        // We break the 1-second delay into 10 smaller delays to check for a button
        // press more frequently, making the system more responsive.
        for(int i = 0; i < 10; i++) {
            HAL_Delay(100);
            if(pedestrian_request) break; // If pressed, exit the delay early.
        }

        // OFF-phase (1 second)
        set_pwm_duty(&htim3, YELLOW_LED_CHANNEL, 0);
        for(int i = 0; i < 10; i++) {
            HAL_Delay(100);
            if(pedestrian_request) break; // Check again during the off-phase.
        }
    }
}

/**
  * @brief Sets the duty cycle of a PWM channel.
  * @param htim: Pointer to the timer handle.
  * @param channel: The timer channel to modify.
  * @param duty_percent: The desired duty cycle (0-100).
  */
void set_pwm_duty(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t duty_percent) {
    // Get the timer's auto-reload register value (the period).
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    // Calculate the pulse value corresponding to the duty cycle percentage.
    uint32_t pulse = (arr * duty_percent) / 100;
    // Set the compare register for the specified channel to the calculated pulse value.
    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}

/**
  * @brief Reads the internal temperature sensor and sets a flag if it's too high.
  */
void check_temperature(void) {
    // Configure the ADC channel for the temperature sensor.
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // Start the ADC conversion.
    HAL_ADC_Start(&hadc1);
    // Wait for the conversion to complete.
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    // Get the raw 12-bit ADC value.
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    // Stop the ADC.
    HAL_ADC_Stop(&hadc1);

    // Convert the raw ADC value to voltage. (Vref = 3.3V, ADC resolution = 12-bit -> 4095).
    float voltage = (3.3f * raw) / 4095.0f;

    // Convert the voltage to temperature using the formula from the STM32 datasheet.
    // V_sense = (V_25) + Avg_Slope * (Temp - 25) => Temp = ((V_sense - V_25) / Avg_Slope) + 25
    // V_25 is typically 0.76V, Avg_Slope is typically 2.5mV/°C (0.0025V/°C).
    float temperature = ((voltage - 0.76f) / 0.0025f) + 25.0f;

    // Log the measured temperature to UART.
    char msg[64];
    snprintf(msg, sizeof(msg), "[TEMP] %.2f °C measured\r\n", temperature);
    log_uart(msg);

    // Check if the temperature exceeds the threshold (35°C).
    if (temperature > 35.0f) {
        // If this is the first time exceeding the threshold, log a message.
        if (!high_temp_flag) {
            log_uart("[TEMP] High temperature detected. Reducing durations\r\n");
        }
        // Set the high temperature flag.
        high_temp_flag = 1;
    } else {
        // If the temperature is normal, clear the flag.
        high_temp_flag = 0;
    }
}

// This is the corrected function for the physical button.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (green_phase_active) {
	    // This is the correct logic for the daytime green phase.
	    if (green_time_remaining > 3 && !green_duration_reduced) {
			pedestrian_request = 1;
			log_uart("[BUTTON] Valid press — green light will be shortened\r\n");
	    } // Other conditions are implicitly ignored.
	} else {
	    // If not in the green phase, check if it's because we are in night mode.
	    if (night_mode_active) {
			if(pedestrian_request == 0) {
				log_uart("[BUTTON] Request registered (Night Mode).\r\n");
				pedestrian_request = 1;
			}
	    } else {
			// If it's not night mode and not green, it must be daytime red/yellow.
			// In this case, we ignore the press completely as requested.
			log_uart("[BUTTON] Ignored during day mode (Red/Yellow phase).\r\n");
	    }
	}
}

// This is the complete and corrected UART callback function.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		char c = uart_rx_buffer[0];

		// This block handles the initial time sync from the Python script.
		if (!rtc_time_set) {
			if (c == '\n' || c == '\r') {
				rtc_uart_buffer[rtc_uart_index] = '\0';
				if (rtc_uart_buffer[0] == 'T') {
					RTC_TimeTypeDef sTime;
					RTC_DateTypeDef sDate;
					if (strlen(rtc_uart_buffer) >= 13) {
						char buf[3] = {0};
						strncpy(buf, &rtc_uart_buffer[1], 2); sTime.Hours = atoi(buf);
						strncpy(buf, &rtc_uart_buffer[3], 2); sTime.Minutes = atoi(buf);
						strncpy(buf, &rtc_uart_buffer[5], 2); sTime.Seconds = atoi(buf);
						strncpy(buf, &rtc_uart_buffer[7], 2); sDate.Date = atoi(buf);
						strncpy(buf, &rtc_uart_buffer[9], 2); sDate.Month = atoi(buf);
						strncpy(buf, &rtc_uart_buffer[11], 2); sDate.Year = atoi(buf);
						sDate.WeekDay = RTC_WEEKDAY_MONDAY;
						HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
						HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
						HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, RTC_BKP_REG_MAGIC);
						log_uart("[RTC] Time set from UART.\r\n");
						rtc_time_set = 1;
					}
				}
				rtc_uart_index = 0;
			} else if (rtc_uart_index < RTC_UART_BUF_SIZE - 1) {
				rtc_uart_buffer[rtc_uart_index++] = c;
			}
		} else {
			// This block handles single-character commands AFTER the time has been set.

			// --- 'p' COMMAND LOGIC ---
			if (uart_rx_buffer[0] == 'p' || uart_rx_buffer[0] == 'P') {
				// The logic here is identical to the physical button handler.
				if (green_phase_active) { // DAY MODE - GREEN
					if (green_time_remaining > 3 && !green_duration_reduced) {
						pedestrian_request = 1;
						log_uart("[UART] Valid press — green light will be shortened\r\n");
					}
				} else { // NOT GREEN (could be night, or day-red/yellow)
					if (night_mode_active) { // NIGHT MODE
						if(pedestrian_request == 0) {
							log_uart("[UART] Request registered (Night Mode).\r\n");
							pedestrian_request = 1;
						}
					} else { // DAY MODE - RED/YELLOW
						log_uart("[UART] Ignored during day mode (Red/Yellow phase).\r\n");
					}
				}
			}
			// *** THIS IS THE ADDED LOGIC FOR THE 't' COMMAND ***
			else if (uart_rx_buffer[0] == 't' || uart_rx_buffer[0] == 'T') {
				log_uart("[UART] Temperature Request\r\n");
				check_temperature();
			}
		}

		// Re-enable UART interrupt for the next character.
		HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 1);
	}
}

/**
  * @brief Callback function for timer period elapsed interrupt.
  * @param htim: Pointer to the timer handle.
  */
/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    // Check if the interrupt was triggered by TIM1.
    if (htim->Instance == TIM1) {
        // Periodically call the temperature check function.
        //check_temperature();
    }
}*/

/**
  * @brief A utility function to transmit a string over UART.
  * @param msg: The null-terminated string to send.
  */
void log_uart(const char* msg) {
    // Transmit the message using blocking (polled) mode.
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  // This function is called when a HAL function returns an error status.
  // It disables interrupts and enters an infinite loop for debugging.
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
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
