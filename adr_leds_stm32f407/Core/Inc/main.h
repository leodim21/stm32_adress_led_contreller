/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define PIXELS_NUM  	900
#define PIXELS_BYTE  	PIXELS_NUM*3
#define PIXELS_BIT  	PIXELS_BYTE*8

#define PERIOD_SYG		270
#define NUL_SYG			230*0.22
#define ONE_SYG			230*0.6

#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
struct NEO_Pixel_s
{
	uint8_t grb[3];
};
struct LED_data_s
{
	struct NEO_Pixel_s line1[PIXELS_NUM];
	struct NEO_Pixel_s line2[PIXELS_NUM];
	struct NEO_Pixel_s line3[PIXELS_NUM];
	struct NEO_Pixel_s line4[PIXELS_NUM];
	struct NEO_Pixel_s line5[PIXELS_NUM];
	struct NEO_Pixel_s line6[PIXELS_NUM];
	struct NEO_Pixel_s line7[PIXELS_NUM];
	struct NEO_Pixel_s line8[PIXELS_NUM];
	uint16_t data_buf_len;
};
typedef union
{
	struct LED_data_s led_data;
	uint8_t data_buf[sizeof(struct LED_data_s)];
}LED_HandleTypeDef;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void TIM1_UPD_Servo_Set( void );
void TIM4_UPD_Servo_Set( void );
void TIM1_CC_Servo_Work_IT( void );
void LED_SendData(uint16_t data_len);
void Delay( uint16_t ms );
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
