/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../Drivers/tm1637.h"
#include "eeprom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Virtual addreses
#define ADDRESS_HOURS_H		1
#define ADDRESS_HOURS_L		2
#define ADDRESS_MINUTES		3
#define ADDRESS_ADDRESSE	4
#define ADDRESS_PRE_TIME	5
#define ADDRESS_COOL_TIME	6
#define ADDRESS_EXT_MODE	7
#define ADDRESS_TEMP_MIN	8
#define ADDRESS_TEMP_MAX	9

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
enum states {stopped, waiting, working, cooling, show_wrk_hours, set_pre_time, clear_time};
uint8_t current_state = 0;
static uint16_t seconds_counter;
static uint16_t minutes_counter;
static uint16_t msec_counter=0;

unsigned char controller_address = 0x0e;
int curr_status;
int prev_status;
int curr_time;

volatile uint32_t g_ADCValue;
volatile uint32_t raw_temp;
volatile uint32_t g_ADCMeanValue;
uint32_t g_MeasurementNumber;
uint32_t g_Temperature;

uint16_t data = 0;
unsigned char  time_to_set = 0;
unsigned int   work_hours[3] = {0,0,0}; //HH HL MM - Hours[2], Minutes[1]
unsigned char  preset_pre_time = 7;
unsigned char  preset_cool_time = 3;
unsigned char  temp_min_threshold;
unsigned char  temp_max_threshold;
unsigned char  ext_mode = 0;
unsigned char  lamps_mode = 0;
unsigned char  temperature_current;
unsigned char  last_rx_address;
unsigned int   error_code;
int start_counter = 0;
int last_button = 0;
int prev_button = 0;
int display_data;
int pre_time, main_time, cool_time;
unsigned int Gv_miliseconds = 0;
int Gv_UART_Timeout = 1000; // Timeout in mSec
int pre_time_sent = 0, main_time_sent = 0, cool_time_sent = 0;
int rx_state= 0;

typedef struct time_str{
	 uint16_t hours_h;
	 uint16_t hours_l;
	 uint16_t minutes;
}time_str;
typedef struct settings_str{
	 uint16_t addresse;
	 uint16_t pre_time;
	 uint16_t cool_time;
	 uint16_t ext_mode;
	 uint16_t volume;
	 uint16_t temperatue_max;
}settings_str;
typedef struct flash_struct{
	 time_str time;
	 settings_str settings;
}flash_struct;
flash_struct flash_data;
uint16_t VirtAddVarTab[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14};//sizeof(flash_data)/2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

void process_key(uint8_t key);
void new_read_eeprom(void);
void new_write_eeprom(void);
float analog2tempBed(int raw);
int ToBCD(int value);
int FromBCD(int value);
void write_stored_time(void);
void write_settings(void);
void read_settings(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim17,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim17) < us);  // wait for the counter to reach the us input in the parameter
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
  MX_ADC_Init();
  MX_IWDG_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  TM1637_init(true,TM1637_BRIGHTNESS_MAX);
  HAL_Delay(100);
  TM1637_init(true,TM1637_BRIGHTNESS_MAX);
  TM1637_display_digit(0,10);
  TM1637_display_digit(1,11);
  TM1637_display_digit(2,12);
  TM1637_display_digit(3,0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static uint8_t colon;
    static uint8_t last_key = 0;
    colon++;
	HAL_Delay(10);
	TM1637_display_colon(seconds_counter &1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	uint8_t key = TM1637_getKeys();
	if(key > 0) {
		TM1637_display_digit(3,key);
	}
	if(last_key != key) {
		last_key = key;
		if(key) {
			process_key(key);
		}
	}
	TM1637_display_digit(0,minutes_counter/10);
	TM1637_display_digit(1,minutes_counter %10);
	TM1637_display_digit(2,seconds_counter/10);
	TM1637_display_digit(3,seconds_counter%10);
	HAL_IWDG_Refresh(&hiwdg);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
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
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 0xFFFF;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */
  HAL_TIM_Base_Start(&htim17);
  /* USER CODE END TIM17_Init 2 */

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
  huart1.Init.BaudRate = 1200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_TXINVERT_INIT|UART_ADVFEATURE_RXINVERT_INIT;
  huart1.AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
  huart1.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
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
  huart2.Init.BaudRate = 9600;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RF_Prog_Pin|P1_Pin|P2_Pin|CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Start_Pin */
  GPIO_InitStruct.Pin = Start_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Start_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_Prog_Pin P1_Pin P2_Pin CLK_Pin */
  GPIO_InitStruct.Pin = RF_Prog_Pin|P1_Pin|P2_Pin|CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SIO_Pin */
  GPIO_InitStruct.Pin = SIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SIO_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void process_key(uint8_t key) {

}



void read_settings(void)
{
	uint8_t result = 0;
	uint16_t  data;

	result += EE_ReadVariable(ADDRESS_ADDRESSE,  &data);
	controller_address = data;
	result += EE_ReadVariable(ADDRESS_PRE_TIME,  &data);
	preset_pre_time = data;
	result += EE_ReadVariable(ADDRESS_COOL_TIME,  &data);
	preset_cool_time = data;
	result += EE_ReadVariable(ADDRESS_EXT_MODE,  &data);
	ext_mode = data;
	result += EE_ReadVariable(ADDRESS_TEMP_MIN,  &data);
	temp_min_threshold = data;
	result += EE_ReadVariable(ADDRESS_TEMP_MAX,  &data);
	temp_max_threshold = data;

	if(result)
	{
		// Set defaults
		preset_pre_time = 7;
		preset_cool_time = 3;
		controller_address = 14;
		temp_min_threshold = 17;
		temp_max_threshold = 85;
		ext_mode = 0;
		work_hours[0] = 0;
		work_hours[1] = 0;
		work_hours[2] = 0;
		write_settings();
		write_stored_time();
	}
}
void read_stored_time(void)
{
	uint16_t  data;
	if(!EE_ReadVariable(ADDRESS_HOURS_H,  &data))
	{
		work_hours[0] = data;
	}
	else
	{
		work_hours[0] = 0;
	}
	if(!EE_ReadVariable(ADDRESS_HOURS_L,  &data))
	{
		work_hours[1] = data;
	}
	else
	{
		work_hours[1] = 0;
	}
	if(!EE_ReadVariable(ADDRESS_MINUTES,  &data))
	{
		work_hours[2] = data;
	}
	else
	{
		work_hours[2] = 0;
	}
}

void write_settings(void)
{
	uint16_t counter;
	for(counter = 0; counter < 3; counter++)
	{
		if(FLASH_COMPLETE != EE_WriteVariable(ADDRESS_ADDRESSE,  controller_address))
		{
			// Second chance
			continue;
		}
		if(FLASH_COMPLETE != EE_WriteVariable(ADDRESS_PRE_TIME,  preset_pre_time))
		{
			// Second chance
			continue;
		}
		if(FLASH_COMPLETE != EE_WriteVariable(ADDRESS_COOL_TIME,  preset_cool_time))
		{
			// Second chance
			continue;
		}
		if(FLASH_COMPLETE != EE_WriteVariable(ADDRESS_EXT_MODE,  ext_mode))
		{
			// Second chance
			continue;
		}
		if(FLASH_COMPLETE != EE_WriteVariable(ADDRESS_TEMP_MIN,  temp_min_threshold))
		{
			// Second chance
			continue;
		}
		if(FLASH_COMPLETE != EE_WriteVariable(ADDRESS_TEMP_MAX,  temp_max_threshold))
		{
			// Second chance
			continue;
		}
		break;
	}
}

void write_stored_time(void)
{
	uint16_t counter;
	for(counter = 0; counter < 3; counter++)
	{
		if(FLASH_COMPLETE != EE_WriteVariable(ADDRESS_HOURS_H,  work_hours[0]))
		{
			// Second chance
			continue;
		}
		if(FLASH_COMPLETE != EE_WriteVariable(ADDRESS_HOURS_L,  work_hours[1]))
		{
			// Second chance
			continue;
		}
		if(FLASH_COMPLETE != EE_WriteVariable(ADDRESS_MINUTES,  work_hours[2]))
		{
			// Second chance
			continue;
		}
		break;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
  {
	//CorrectedValue = (((RawValue � RawLow) * ReferenceRange) / RawRange) + ReferenceLow
      g_ADCValue = HAL_ADC_GetValue(AdcHandle);
      g_ADCMeanValue = (g_ADCMeanValue*9 + g_ADCValue)/10;
      raw_temp = 1024 - g_ADCValue;
      g_MeasurementNumber++;
      g_Temperature = analog2tempBed(g_ADCValue); // Must be an error...
      if(g_Temperature > 200)
      {
    	  g_Temperature = 0;
      }
//      g_Temperature = (g_Temperature*9 + (((g_ADCValue - ADC_0_DEGREE_VALUE)*366)/(ADC_36_6_DEGREE_VALUE - ADC_0_DEGREE_VALUE)) + 0)/10;
  }

void usart1_IT_handler()
{
	 uint32_t isrflags   = READ_REG(USART2->ISR);
	 uint32_t cr1its     = READ_REG(USART2->CR1);
	 uint32_t cr3its     = READ_REG(USART2->CR3);
	 uint32_t errorflags = 0x00U;
//	 uint32_t dmarequest = 0x00U;
	 enum rxstates {rx_state_none, rx_state_pre_time, rx_state_main_time, rx_state_cool_time, rx_state_get_checksum};

	 /* If no error occurs */
	 errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));
	 if(errorflags == RESET)
	 {
		 /* UART in mode Receiver -------------------------------------------------*/
		 if(((isrflags & USART_CR1_RXNEIE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
		 {
			 data = USART2->RDR & (uint16_t)0x00FF;
			 if ((data & 0x80)){
			 		last_rx_address = (data >> 3U)&0x0f;
			 		unsigned char addr_is_ok = 0;
		 			addr_is_ok = (last_rx_address == controller_address);
			 		if (!addr_is_ok) return;
			 		// Command
			 		if(addr_is_ok){
			 			Gv_UART_Timeout = 1500;
			 		}
			 		if((data & 0x07) == 0x00 ){ // Status
			 			data = (curr_status<<6)| ToBCD(curr_time);
			 //			data = (STATUS_WORKING<<6)|4;
			 			USART1->TDR = data;
			 		}
			 		else if ((data & 0x07) == 1) //Command 1 - Start
			 		{
			 			pre_time = 0;
//			 			update_status();
			 		}
			 		else if ((data & 0x07) == 2)  //Command 2 == Pre_time_set
			 		{
			 			rx_state = rx_state_pre_time;
			 		}
			 		else if ((data & 0x07) == 5) //Command 5 == Main_time_set
			 		{
			 			rx_state = rx_state_main_time;
			 		}
			 		else if ((data & 0x07) == 3) //Command 3 == Cool_time_set
			 		{
			 			rx_state = rx_state_cool_time;
			 		}

			 	} else if (rx_state){
			 		// payload
			 		int time_in_hex = ToBCD(main_time_sent);
			 		if(rx_state == rx_state_get_checksum){
			 			int checksum = (pre_time_sent + cool_time_sent  - time_in_hex - 5) & 0x7F;
			 			if(	data == checksum){
			 				pre_time = pre_time_sent;
			 				main_time = main_time_sent;
			 				cool_time = cool_time_sent;
//			 				update_status();
			 				Gv_miliseconds = 0;
			 			}
			 			rx_state = 0;
			 		}
			 		if(rx_state == rx_state_pre_time){
			 			pre_time_sent = data;
			 			rx_state = 0;
			 		}
			 		if(rx_state == rx_state_main_time){
			 			main_time_sent = FromBCD(data);
			 			rx_state = 0;
			 		}
			 		if(rx_state == rx_state_cool_time){
			 			cool_time_sent = data;
			 			rx_state = rx_state_get_checksum;
			 			int checksum = (pre_time_sent + cool_time_sent  - time_in_hex - 5) & 0x7F;
			 			data = checksum;
			 			USART2->TDR = data;
			 		}


			 	}
		 }
	 }
	 else
	 {
		 rx_state= 0;
		 USART2->ISR = 0; // Clear Errors
	 }
}


void user_systick_callback() {

	msec_counter++;
	if(msec_counter > 999) {
		msec_counter = 0;
		seconds_counter++;
		if(seconds_counter > 59) {
			seconds_counter = 0;
			minutes_counter++;
			if(minutes_counter > 99) {
				minutes_counter = 0;
			}
		}
	}
}

// 10k thermistor Chineese
const float temptable_10[][2] = {
  {    1 , 430 },
  {   0x67A , 100 },
  {  0x835, 36 },
  {  0xA0A,  20 },
  {  0xAA0,  13 },
  {  0xCCC,  0 },
};
#define COUNT(a) (sizeof(a)/sizeof(*a))
#define BEDTEMPTABLE temptable_10
#define BEDTEMPTABLE_LEN COUNT(BEDTEMPTABLE)
float analog2tempBed(int raw) {

    float celsius = 0;
    unsigned char i;

    for (i = 1; i < BEDTEMPTABLE_LEN; i++) {
      if ((BEDTEMPTABLE[i][0]) > raw) {
        celsius  = BEDTEMPTABLE[i - 1][1] +
                   (raw - BEDTEMPTABLE[i - 1][0]) *
                   (float)(BEDTEMPTABLE[i][1] - BEDTEMPTABLE[i - 1][1]) /
                   (float)(BEDTEMPTABLE[i][0] - BEDTEMPTABLE[i - 1][0]);
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == BEDTEMPTABLE_LEN) celsius = BEDTEMPTABLE[i - 1][1];

    return celsius;

}

int ToBCD(int value){
	int digits[3];
	int result;
	digits[0] = value %10;
	digits[1] = (value/10) % 10;
	digits[2] = (value/100) % 10;
	result = digits[0] | (digits[1]<<4) | (digits[2]<<8);
	return result;
}

int FromBCD(int value){
	int digits[3];
	int result;
	digits[0] = value & 0x0F;
	digits[1] = (value>>4) & 0x0F;
	digits[2] = (value>>8) & 0x0F;
	result = digits[0] + digits[1]*10 + digits[2]*100;
	return result;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
