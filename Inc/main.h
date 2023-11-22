/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
	
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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

void Can_Send_Msg(uint8_t ucStdId, uint8_t* msg, uint8_t len);
static void OneNet_Send(uint8_t *Send_date,uint8_t device_id,uint8_t dat_len);
static void OneNet_FillBuf(uint8_t *buff,uint8_t devicer_id);
static void Gps_Msg_Show(void);
static void printf_sdcard_info(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define oil_yycj_Pin GPIO_PIN_0
#define oil_yycj_GPIO_Port GPIOC
#define xjwy2_Pin GPIO_PIN_1
#define xjwy2_GPIO_Port GPIOC
#define xjwy1_Pin GPIO_PIN_2
#define xjwy1_GPIO_Port GPIOC
#define FR_Pin GPIO_PIN_0
#define FR_GPIO_Port GPIOA
#define FL_Pin GPIO_PIN_1
#define FL_GPIO_Port GPIOA
#define youliang_Pin GPIO_PIN_4
#define youliang_GPIO_Port GPIOA
#define QR_Pin GPIO_PIN_5
#define QR_GPIO_Port GPIOA
#define QL_Pin GPIO_PIN_7
#define QL_GPIO_Port GPIOA
#define xjwy4_Pin GPIO_PIN_4
#define xjwy4_GPIO_Port GPIOC
#define ymwy_Pin GPIO_PIN_5
#define ymwy_GPIO_Port GPIOC
#define xjwy3_Pin GPIO_PIN_0
#define xjwy3_GPIO_Port GPIOB
#define batter_Pin GPIO_PIN_1
#define batter_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
