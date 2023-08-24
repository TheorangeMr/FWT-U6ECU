/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
	
#define USART3_MAX_RECV_LEN		2048					//最大接收缓存字节数
#define USART3_MAX_SEND_LEN		600					//最大发送缓存字节数
#define USART3_RX_EN 			1					//0,不接收;1,接收.
	
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void usart3_init(uint32_t bound);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

