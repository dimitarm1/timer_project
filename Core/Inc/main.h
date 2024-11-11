/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define true 1
#define false 0
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void delay_us (uint16_t us);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Start_Pin GPIO_PIN_1
#define Start_GPIO_Port GPIOA
#define RF_Prog_Pin GPIO_PIN_4
#define RF_Prog_GPIO_Port GPIOA
#define P1_Pin GPIO_PIN_5
#define P1_GPIO_Port GPIOA
#define P2_Pin GPIO_PIN_6
#define P2_GPIO_Port GPIOA
#define CLK_Pin GPIO_PIN_15
#define CLK_GPIO_Port GPIOA
#define SIO_Pin GPIO_PIN_4
#define SIO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
typedef struct time_str{
	 uint16_t hours_h;
	 uint16_t hours_l;
	 uint16_t minutes;
}time_str;
typedef struct {
	 uint8_t digit_0;
	 uint8_t digit_1;
	 uint8_t digit_2;
	 uint8_t digit_3;
	 uint8_t brightness;
}display_data_t;

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
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
