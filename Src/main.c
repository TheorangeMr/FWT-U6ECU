/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "fatfs.h"
#include "sdio.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "wit_c_sdk.h"
#include "atk_m750.h"
#include "RingBuffer.h"
#include "string.h"
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

/* USER CODE BEGIN PV */
uint8_t cantx_dat[8] = {0};
RingBuffer *p_uart2_rxbuf;
uint8_t DTU_flag = 0;
extern _dtu_4g_device dtu_device1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
	��������Wit_Can_Send_Msg()
	��  �ܣ�CAN����֡����
  ��  ����u8 ucStdId����չ��ʶ��, u8* msg������ָ��, u8 len�����ݳ���
  ����ֵ��PASSED:����ɹ���FAILED:ʧ��
*/

void Wit_Can_Send_Msg(uint8_t ucStdId, uint8_t* msg, uint32_t len)
{
	uint16_t i=0;uint32_t can_tx_mailbox = 0;
	CAN_TxHeaderTypeDef TxMessage;
	TxMessage.StdId=ucStdId;
	TxMessage.ExtId=0;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.DLC=len;
	TxMessage.TransmitGlobalTime=DISABLE;	
	for(i=0;i<len;i++)
	cantx_dat[i]=msg[i];
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) < 1);
	if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,cantx_dat,&can_tx_mailbox) != HAL_OK)
	{
		printf ("���ݷ���ʧ�ܣ�\r\n");
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint16_t timeout = 0;
	int ret;
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	
	//nine-axis dvice initialize
	WitInit(WIT_PROTOCOL_CAN, 0x50);
	WitRegisterCallBack(SensorDataUpdata);
	WitCanWriteRegister(Wit_Can_Send_Msg);
	WitDelayMsRegister(Wit_Delayms);
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
	
	my_mem_init(SRAMIN);
	p_uart2_rxbuf = RingBuffer_Malloc(1024);        /*���ڴ���з���1K���ڴ������3����DTU����*/
	
	printf("Wait for Cat1 DTU to start, wait 10s.... \r\n");
	while( timeout <= 10 )   /* �ȴ�Cat1 DTU��������Ҫ�ȴ�5-6s�������� */
	{
			ret = dtu_config_init(DTU_WORKMODE_ONENET);    /*��ʼ��DTU��������*/
			if( ret == 0 )
			{
				dtu_device1.Dtumode_Switch_flag = 1;
				
				printf("Cat1 DTU Init Success \r\n");
				break;
			}
			timeout++;
			HAL_Delay(1000);
	}
	while( timeout > 10 )   /* ��ʱ */
	{
		printf("**************************************************************************\r\n");
		printf("ATK-DTU Init Fail ...\r\n");
		printf("�밴�����²�����м��:\r\n");
		printf("1.ʹ�õ�����λ������������DTU�ܷ񵥶���������\r\n");
		printf("2.���DTU���ڲ�����STM32ͨѶ�Ĵ��ڲ����Ƿ�һ��\r\n");
		printf("3.���DTU��STM32���ڵĽ����Ƿ���ȷ\r\n");
		printf("4.���DTU�����Ƿ�������DTU�Ƽ�ʹ��12V/1A��Դ���磬��Ҫʹ��USB��5V��ģ�鹩�磡��\r\n");
		printf("**************************************************************************\r\n\r\n");
		HAL_Delay(1000);
		DTU_flag = 1;
		break;
	}	
	
  printf("�����ʼ��\r\n");
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
	printf("Error_Handler\r\n");
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
