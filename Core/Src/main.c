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
#include "math.h"
#include "stdlib.h"
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

#define FLASH_DIVIDER  0x10

// Buttons
#define BUTTON_START     (1)
#define BUTTON_PLUS      (2)
#define BUTTON_STOP      (3)
#define BUTTON_MINUS     (4)

#define STATUS_FREE    (0L)
#define STATUS_WAITING (3L)
#define STATUS_WORKING (1L)
#define STATUS_COOLING (2L)
#define MULTIPLIER     (1)
#define START_COUNTER_TIME  (1000L*MULTIPLIER)
#define ENTER_SERVICE_DELAY (500L*MULTIPLIER)
#define SERVICE_NEXT_DELAY  (100*MULTIPLIER)
#define EXIT_SERVICE_TIME   (3000L*MULTIPLIER)

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
typedef enum states {state_show_time,state_set_time,state_show_hours,state_enter_service,state_clear_hours,state_address,state_pre_time,state_cool_time}states;
states state;
typedef enum modes {mode_null,mode_clear_hours,mode_set_address,mode_set_pre_time,mode_set_cool_time}modes;
modes service_mode;

uint8_t current_state = 0;
static uint16_t seconds_counter;
static uint16_t minutes_counter;
static uint16_t msec_counter=0;

unsigned char controller_address = 0x0e;
int curr_status;
int prev_status;

int flash_counter = 0;
volatile uint32_t g_ADCValue;
volatile uint32_t raw_temp;
volatile uint32_t g_ADCMeanValue;
uint32_t g_MeasurementNumber;
uint32_t g_Temperature;

uint16_t data = 0;
unsigned int   time_to_set = 0;
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
int flash_mode = 0;
int start_counter = 0;
int last_button = 0;
int prev_button = 0;
display_data_t display_data = {0,0,0,0,255};

// Timers in seconds
int pre_time;
int main_time;
int cool_time;
// Current time counter in seconds
int *curr_time = &pre_time;
int useUart=0;

unsigned int Gv_miliseconds = 0;
int Gv_UART_Timeout = 1000; // Timeout in mSec
int pre_time_sent = 0, main_time_sent = 0, cool_time_sent = 0;
int rx_state= 0;


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

void update_status(void);
void process_key(uint8_t key);
void ProcessButtons(void);
void new_read_eeprom(void);
void new_write_eeprom(void);
float analog2tempBed(int raw);
int ToBCD(unsigned int value);
int FromBCD(int value);
void write_stored_time(void);
void write_settings(void);
void read_settings(void);
void SetDisplayDataInt(unsigned value);
void relay_fan_on();
void relay_fan_off();
void relay_lampi_on();
void relay_lampi_off();
void UART_RxISR(UART_HandleTypeDef *huart);

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
	__disable_irq();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  __disable_irq();
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
  __enable_irq();
  TM1637_init(true,TM1637_BRIGHTNESS_MAX);
  HAL_Delay(100);
  TM1637_init(true,TM1637_BRIGHTNESS_MAX);
  TM1637_display_digit(0,10);
  TM1637_display_digit(1,11);
  TM1637_display_digit(2,12);
  TM1637_display_digit(3,0);
  EE_Init();
  read_settings();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static uint8_t colon;
    colon++;
	HAL_Delay(10);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	last_button = TM1637_getKeys();
	if(HAL_GPIO_ReadPin(Start_GPIO_Port, Start_Pin) == 0){
		last_button |= BUTTON_START;
	}
//	if(button > 0) {
//		TM1637_display_digit(3,key);
//	}

	//-------------------

	if ((state < state_enter_service) && ((flash_counter >> 4) & 1))
	{
//		if (controller_address == 15) {
//			external_read = (external_read << 1) | (!!(GPIOB->IDR & GPIO_IDR_IDR7));
//			if (!main_time && (!external_read)) {
//				if (curr_status != STATUS_COOLING) {
//					main_time = -1;
//					cool_time = preset_cool_time;
//					Gv_miliseconds = 0;
//					flash_mode = 0;
//					state = state_set_time;
//					update_status();
//				}
//			}
//			if (main_time && !~external_read) {
//				main_time = 0;
//				Gv_miliseconds = 0;
//				update_status();
//			}
//		}
	}
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
	ProcessButtons();
	switch (state) {
	case state_show_time:
	case state_set_time:
		if (!pre_time && !main_time && !cool_time) {
			flash_mode = 0;
			state = state_set_time;
			SetDisplayDataMinSec(time_to_set*60);
			TM1637_display_colon(1);
			//				display_data = ToBCD(last_button); //Debug
		}
		else if (!pre_time && !main_time && cool_time)
		{
			TM1637_display_colon(1);
			if(flash_counter & 0x20)
			{
				SetDisplayDataInt(0xFFFF);
			}
			else
			{
				SetDisplayDataMinSec(cool_time);
			}
		}
		else {
			state = state_show_time;
			//				  time_to_set = 0;
			SetDisplayDataMinSec(*curr_time);
			if(curr_status == STATUS_WORKING )
			{
				TM1637_display_colon(seconds_counter &1);
			}
			else
			{
				TM1637_display_colon(1);
			}

			//				display_data = ToBCD(last_button); //Debug
		}
		break;
	case state_show_hours:

		if (flash_mode != 3) {
			flash_mode = 3; // All flashing
		}
		{
			int index = ((flash_counter /FLASH_DIVIDER)) % 8;
			if ((index < 6) && (index & 1)) {
				SetDisplayDataInt(work_hours[index/2]);
			}
			else {
				SetDisplayDataInt(0xFFFF);
			}
		}
		break;
	case state_enter_service:
		SetDisplayDataInt(service_mode | 0xF0);
		flash_mode = 0;
		break;

	case state_clear_hours:
		//			flash_mode = 3;
		if ((flash_counter /FLASH_DIVIDER) & 1) {
			SetDisplayDataInt(0xFFFC);
		}
		else {
			SetDisplayDataInt(0xFFFF);
		}
		break;
	case state_address:
		flash_mode = 0;
		if ((flash_counter /FLASH_DIVIDER) & 1) {
			SetDisplayDataInt(controller_address);
		}
		else {
			SetDisplayDataInt(0xFFFA);
		}
		break;
	case state_pre_time:
		//			flash_mode = 3;
		if ((flash_counter /FLASH_DIVIDER) & 1) {
			SetDisplayDataInt(preset_pre_time);
		}
		else {
			SetDisplayDataInt(0xFFF3);
		}
		break;
	case state_cool_time:
		//			flash_mode = 3;
		if ((flash_counter /FLASH_DIVIDER) & 1) {
			SetDisplayDataInt(preset_cool_time);
		}
		else {
			SetDisplayDataInt(0xFFF4);
		}
		break;
	}
	//		show_digit(display_data);
	if (state == state_show_time) {
		if (!(pre_time || main_time || cool_time)) {
			// Some Paranoia...
			// turn off all
//			set_lamps(OFF);
//			percent_fan1 = 0;
//			set_fan1(0);

			//				state = state_set_time;
		}
	}
	//
	//		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET); // wAIT UNTIL RX BUFFER IS EMPTY
	//		int data =  USART_ReceiveData(USART1);
	//		USART_SendData(USART1,0x80);
	//				while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // wAIT UNTIL TX BUFFER IS EMPTY
	if (pre_time) {
		// Indicate pre_time by moving bars
//		ShowBarIndicators((flash_counter >> 4) %10, (flash_counter >> 4) % 10);
	}
	else if (cool_time) {
//		ShowBarIndicators(volume_level, fan_level);
	}

	HAL_Delay(10);


	//Uncomment to check temperature
	if ((last_button == BUTTON_STOP) && (curr_status == STATUS_FREE))
	{
		SetDisplayDataInt(ToBCD(g_Temperature));
//		SetDisplayDataInt(ToBCD(g_ADCMeanValue));
	}
	if(error_code)
	{
		SetDisplayDataInt(error_code);
	}
	TM1637_set_brightness(display_data.brightness);
	TM1637_display_digit(3,display_data.digit_3);
	TM1637_display_digit(2,display_data.digit_2);
	TM1637_display_digit(1,display_data.digit_1);
	TM1637_display_digit(0,display_data.digit_0);
	flash_counter++;
	if(	(temp_max_threshold > 0) && (g_Temperature > 0))
	{
		if((g_Temperature > temp_max_threshold))
		{
//			percent_fan1 = 10;
//			set_fan1(percent_fan1);
			if(curr_status == STATUS_WORKING)
			{
				main_time = 0;
				pre_time = 0;
				update_status();
			}
			error_code = 0xFE1;
		}
		else
		{
			if((main_time == 0) && (cool_time == 0))
			{
//				percent_fan1 = 0;
//				set_fan1(0);
			}
		}
	}
	if( (curr_status == STATUS_WORKING) && /*(percent_fan1 == 0) &&*/ (g_Temperature > 70)) // 70 degrees
	{
//		percent_fan1 = 3;
//		set_fan1(percent_fan1);
	}
	/* Reload IWDG counter */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

  /** Initializes the CPU, AHB and APB buses clocks
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
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
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
  htim1.Init.Period = 65535;
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
  huart1.Init.BaudRate = 9600;
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
  huart1.RxISR = UART_RxISR;
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
  huart2.Init.BaudRate = 1200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_8;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart2.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  huart2.RxISR = UART_RxISR;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RF_Prog_Pin|P1_Pin|P2_Pin|CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Start_Pin */
  GPIO_InitStruct.Pin = Start_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
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

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
      if(!g_ADCMeanValue) {
    	  g_ADCMeanValue = g_ADCValue;
      }
      g_ADCMeanValue = (g_ADCMeanValue*9 + g_ADCValue)/10;
      g_MeasurementNumber++;
      g_Temperature = analog2tempBed(g_ADCMeanValue); // Must be an error...
      if(g_Temperature > 200)
      {
    	  g_Temperature = 0;
      }
//      g_Temperature = (g_Temperature*9 + (((g_ADCValue - ADC_0_DEGREE_VALUE)*366)/(ADC_36_6_DEGREE_VALUE - ADC_0_DEGREE_VALUE)) + 0)/10;
  }

void UART_RxISR(UART_HandleTypeDef *huart)
{
	 uint32_t isrflags   = READ_REG(huart->Instance->ISR);
	 uint32_t cr1its     = READ_REG(huart->Instance->CR1);
	 uint32_t cr3its     = READ_REG(huart->Instance->CR3);
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
			 data = huart->Instance->RDR & (uint16_t)0x00FF;
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
			 			uint16_t time = *curr_time/60;
			 			if (*curr_time) {
			 				// Compensate for int rounding
			 				time = time+1;
			 			}
			 			data = (curr_status<<6)| ToBCD(time);
			 //			data = (STATUS_WORKING<<6)|4;
			 			huart->Instance->TDR = data;
			 		}
			 		else if ((data & 0x07) == 1) //Command 1 - Start
			 		{
			 			pre_time = 0;
			 			update_status();
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
			 				pre_time = pre_time_sent*60;
			 				main_time = main_time_sent*60;
			 				cool_time = cool_time_sent*60;
			 				update_status();
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
			 			huart->Instance->TDR = data;
			 		}


			 	}
		 }
	 }
	 else
	 {
		 rx_state= 0;
		 huart->Instance->ISR = 0; // Clear Errors
	 }
	  /* Clear RXNE interrupt flag */
	 __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
//	 USART2->ISR = 0; // Clear Errors
}


void user_systick_callback() {
	if(Gv_miliseconds++>1000L){
		Gv_miliseconds = 0;
		seconds_counter++;
		if (pre_time)
		{
//			play_message(1,message_hurry_up);
			pre_time--;
		}
		else if (main_time) {
			main_time--;
		}
		else if (cool_time) cool_time--;
		update_status();
		HAL_ADC_Start_IT(&hadc);
	}
	if  (Gv_UART_Timeout){
		Gv_UART_Timeout--;
		if(! Gv_UART_Timeout) {
			rx_state = 0;
			pre_time_sent = 0, main_time_sent = 0, cool_time_sent = 0;
		}
	}

//	msec_counter++;
//	if(msec_counter > 999) {
//		msec_counter = 0;
//		seconds_counter++;
//		if(seconds_counter > 59) {
//			seconds_counter = 0;
//			minutes_counter++;
//			if(minutes_counter > 99) {
//				minutes_counter = 0;
//			}
//		}
//	}
}

// 10k thermistor Chineese
const float temptable_10[][2] = {
  {    1 , 430 },
  {  2455 , 96 },
  {  3493, 36 },
  {  3726,  17.5 },
  {  3830,  3 },
  {  3845,  0 },
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

int ToBCD(unsigned value){
	int digits[4];
	int result;
	digits[0] = value %10;
	digits[1] = (value/10) % 10;
	digits[2] = (value/100) % 10;
	digits[3] = (value/1000) % 10;
	result = digits[0] | (digits[1]<<4) | (digits[2]<<8) | (digits[3]<<12);
}

void SetDisplayDataMinSec(unsigned value){
	unsigned int seconds = value %60;
	unsigned int minutes = value /60;
	{
		display_data.digit_3 = seconds %10;
		display_data.digit_2  = (seconds/10) % 10;
		display_data.digit_1  = (minutes) % 10;
		display_data.digit_0  = (minutes/10) % 10;
	}
}

void SetDisplayDataInt(unsigned value){
	{
		display_data.digit_3 = value &0x0f;
		display_data.digit_2  = (value>>4) &0x0f;
		display_data.digit_1  = (value>>8) &0x0f;
		display_data.digit_0  = (value>>12) &0x0f;
	}
}

int FromBCD(int value){
	int digits[4];
	int result;
	digits[0] = value & 0x0F;
	digits[1] = (value>>4) & 0x0F;
	digits[2] = (value>>8) & 0x0F;
	digits[3] = (value>>12) & 0x0F;
	result = digits[0] + digits[1]*10 + digits[2]*100 + digits[3]*1000;
	return result;
}

/**
 * @brief  Manage the activity on buttons
 * @param  None
 * @retval None
 */
void ProcessButtons(void)
{

	if (last_button) {
		if (last_button != prev_button) {
			switch (last_button) {
			case BUTTON_START:
				if(error_code)
				{
					error_code = 0;
				}
				if (pre_time) {
					//send_start();
					Gv_miliseconds = 0;
					pre_time = 0;
					state = state_show_time;
					update_status();
					if((temp_min_threshold > 0) && (g_Temperature > 0) && (g_Temperature < temp_min_threshold))
					{
					}
					else
					{
					}
				}
				if (!pre_time && !main_time && !cool_time) {
					if (time_to_set) {
						// Send of time moved elsewhere
						pre_time = preset_pre_time*60;
						main_time = time_to_set*60;
						cool_time = preset_cool_time*60;
						state = state_show_time;
						time_to_set = 0;
						Gv_miliseconds = 0;
						update_status();
						//						play_message(0);
					}
					else {

							// Write EEPROM
						if (state == state_clear_hours)
						{
							work_hours[0] = 0;
							work_hours[1] = 0;
							work_hours[2] = 0;

							write_stored_time();
							start_counter = 0;
							service_mode = mode_null;
						}
						else if (state > state_enter_service)
						{
							write_settings();
							service_mode = mode_null;
							if (state == state_address) {
//								init_periph();
							}
							start_counter = 0;
						}
						else {
							start_counter = START_COUNTER_TIME;
						}
						state = state_show_hours;
						flash_counter = 0;
					}
				}
				break;
			case BUTTON_STOP:
				if (curr_status == STATUS_FREE) {
					time_to_set = 0;
					update_status();
				}
				if (curr_status == STATUS_WAITING) {
					main_time = 0;
					pre_time = 0;
					cool_time = 0;
					update_status();
				}
				if (curr_status == STATUS_WORKING) {
					main_time = 0;
					pre_time = 0;
					update_status();
//					percent_fan1 = 10;
//					set_fan1(percent_fan1);
				}
				if (curr_status == STATUS_COOLING) {
					cool_time = 0;
					update_status();
//					percent_fan1 = 10;
//					set_fan1(percent_fan1);
				}
				if (state >= state_enter_service) {
					start_counter = 0;
					state = state_show_time;
					service_mode = mode_null;
				}
				break;
			case BUTTON_PLUS:
			{
				if (state == state_show_hours) {
					state = state_set_time;
					start_counter = 0;
				}
				if (state == state_set_time) {
					if (time_to_set < 60L) { // Limit max time
						if (!useUart) time_to_set++;
					}
					else
					{
						while (1); // Reboot device using WatchDog
					}
				}
				else if (state > state_enter_service) {
					start_counter = EXIT_SERVICE_TIME;
					switch (service_mode) {
					case mode_set_address:
						if (controller_address < 15) controller_address++;
						break;
					case mode_set_pre_time:
						if (preset_pre_time < 9) preset_pre_time++;
						break;
					case mode_set_cool_time:
						if (preset_cool_time < 9) preset_cool_time++;
						break;
					default:
						break;
					}
				}

			}
			break;
			case BUTTON_MINUS:
				if (state == state_show_hours) {
					state = state_set_time;
					start_counter = 0;
				}
				if (state == state_set_time) {
					if (time_to_set) {
						if (!useUart)  time_to_set--;
					}
				}
				else if (state > state_enter_service) {
					start_counter = EXIT_SERVICE_TIME;
					switch (service_mode) {
					case mode_set_address:
						if (controller_address) controller_address--;
						break;
					case mode_set_pre_time:
						if (preset_pre_time) preset_pre_time--;
						break;
					case mode_set_cool_time:
						if (preset_cool_time) preset_cool_time--;
						break;
					default:
						break;
					}
				}
				//				if(selected_led_bits & LED_FAN2_L){
				//					if(percent_fan2) percent_fan2 = 0;
				//					set_fan2(percent_fan2);
				//				} else if(selected_led_bits & LED_FAN1_L){
				//					if(percent_fan1) percent_fan1-=25;
				//					set_fan1(percent_fan1);
				//				}
				//				else if(selected_led_bits & LED_LICEVI_L){
				//					if(percent_licevi) percent_licevi=0;
				//					set_licevi_lamps(percent_licevi);
				//					update_status();
				//				}

				break;
			default:
				while (0);
			}
		}
	}
	prev_button = last_button;

	if (last_button == BUTTON_START)
	{
		// LED1_ON;
		if (start_counter < (START_COUNTER_TIME + ENTER_SERVICE_DELAY + 6 * SERVICE_NEXT_DELAY)) start_counter++;
		if (start_counter == START_COUNTER_TIME + ENTER_SERVICE_DELAY) {
			if (curr_status == STATUS_FREE && (state < state_enter_service))
			{
				state = state_enter_service;
				service_mode = mode_clear_hours; // Clear Hours
			}
		}
		if (state == state_enter_service) {
			if (start_counter == START_COUNTER_TIME + ENTER_SERVICE_DELAY + 1 * SERVICE_NEXT_DELAY) {
				service_mode = mode_set_address; //
			}
			else if (start_counter == START_COUNTER_TIME + ENTER_SERVICE_DELAY + 2 * SERVICE_NEXT_DELAY) {
				service_mode = mode_set_pre_time; //
			}
			else if (start_counter == START_COUNTER_TIME + ENTER_SERVICE_DELAY + 3 * SERVICE_NEXT_DELAY) {
				service_mode = mode_set_cool_time; //
			}
		}
//		set_start_out_signal(1);
	}
	else
	{
//		set_start_out_signal(0);
		if (start_counter) {
			if (state == state_show_hours) {
				start_counter--;
				if (!start_counter) {
					state = state_show_time;
				}
			}
			else if (state >= state_enter_service) {
				if (state == state_enter_service) {
					state = service_mode + state_enter_service;
				}
				start_counter--;
				if (!start_counter) {
					state = state_show_time;
//					new_write_eeprom();
//					new_read_eeprom();
				}
			}
			else {
				start_counter = 0;

			}
		}
	}

	if (prev_status != curr_status) {
		if (curr_status == STATUS_WORKING) {
//			if (1) {
//				work_hours[2] += main_time;
//				if (work_hours[2] > 59L) {
//					work_hours[2] = work_hours[2] - 60;
//					work_hours[1]++;
//					if (work_hours[1] > 99L) {
//						work_hours[1] = 0;
//						work_hours[0]++;
//					}
//				}
//				write_stored_time();
//			}
			flash_mode = 0;
//			set_lamps(100);
			if((temp_min_threshold > 0) && (g_Temperature > 0) && (g_Temperature < 70)) // 70 degrees
			{
//				percent_fan1 = 0;
			}
//			set_fan1(percent_fan1);
			relay_lampi_on();
			relay_fan_on();
		}
		if (curr_status == STATUS_COOLING) {
			if (controller_address == 15) {
				work_hours[2] += abs(main_time);
				if (work_hours[2] > 59L) {
					work_hours[2] = work_hours[2] - 60;
					work_hours[1]++;
					if (work_hours[1] > 99L) {
						work_hours[1] = 0;
						work_hours[0]++;
					}
				}
				write_stored_time();

			}
			relay_lampi_off();
			relay_fan_on();
//			percent_fan1 = 10L;
//			set_lamps(OFF);
			//			zero_crossed = 0;
//			stop_delay = 1000;
			//			set_colarium_lamps(0);
//			fan_level = 10;
//			set_fan1(percent_fan1);
			flash_mode = 3;
		}
		if (curr_status == STATUS_FREE) {
			relay_lampi_off();
			relay_fan_off();
//			set_lamps(OFF);
//			set_fan1(0);
			//			set_aquafresh(percent_aquafresh);
//			minute_counter = 0;
			flash_mode = 0;
		}
		prev_status = curr_status;

	}
	//    LED1_OFF;

}

//---------------------------------------------------------------
void update_status(void) {
	if (pre_time) {
		curr_time = &pre_time;
		curr_status = STATUS_WAITING;
	}
	else if (main_time) {
		curr_time = &main_time;
		curr_status = STATUS_WORKING;
	}
	else if (cool_time) {
		curr_time = &cool_time;
		curr_status = STATUS_COOLING;
	}
	else {
		curr_time = &main_time;
		curr_status = STATUS_FREE;
	}
}

//---------------------------------------------------------------
void relay_fan_on()
{
  HAL_GPIO_WritePin(P1_GPIO_Port, P2_Pin, GPIO_PIN_SET);
}
//---------------------------------------------------------------
void relay_fan_off()
{
  HAL_GPIO_WritePin(P1_GPIO_Port, P2_Pin, GPIO_PIN_RESET);
}
//---------------------------------------------------------------
void relay_lampi_on()
{
	HAL_GPIO_WritePin(P1_GPIO_Port, P1_Pin, GPIO_PIN_SET);
}
//---------------------------------------------------------------
void relay_lampi_off()
{
	HAL_GPIO_WritePin(P1_GPIO_Port, P1_Pin, GPIO_PIN_RESET);
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
