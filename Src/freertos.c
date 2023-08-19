/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "atk_m750.h"
#include "RingBuffer.h"
#include "wit_c_sdk.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
typedef StaticEventGroup_t osStaticEventGroupDef_t;
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TEST_PRINTF_RINGBUFFER 1
#define TEST_PRINTF_RINGBUFFER 1

#define Wit_dat   ((uint32_t)0x50) //标识符ID：0x50    (标准帧)
#define Wit_Device_ID        ((uint8_t)0x01)
#define XJ1_Device_ID        ((uint8_t)0x02)             //油门踏板
#define GPS_Device_ID        ((uint8_t)0x03)
#define DTU_Device_ID         ((uint8_t)0x04)

/*
*************************************************************************
*               thread interval allocation
*************************************************************************
*/
#define   Wit_Dat_Handle_Delay          30
#define   Can_Rx_Handle_Delay           1
#define   OD_Handle_Delay               200
#define   DTU_Signal_Delay              1000


/*
*************************************************************************
*                event flag
*************************************************************************
*/


#define EVENTBIT_0	(1<<0)				//九轴事件
#define EVENTBIT_1	(1<<1)				//暂未使用
#define EVENTBIT_2	(1<<1)				//4g信号事件

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

CAN_RxHeaderTypeDef RxMessage;

//usart1,2,3
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern uint8_t rx_4g_buffer[128];
extern uint8_t rx_len;
extern uint8_t rx_flag;

//adc
extern ADC_HandleTypeDef hadc1;

//九轴传感器
extern CAN_HandleTypeDef hcan1;
extern RegUpdateCb p_WitRegUpdateCbFunc;      
extern volatile char  s_cDataUpdate;
wit_can_dat wit_dat1 = {{0,0,0},{0,0,0},{0,0,0}};

//can
	uint8_t canrx_dat[8] = {0};

//onenet 
extern ST_Time Timedat;
extern const char sqpa[4][4];
const char ONENET_COM_OFF[]="lc0218";
const char ONENET_COM_ON[] = "lc2001";
RingBuffer *p_uart2_rxbuf;
_dtu_4g_device dtu_device1 = {0,0,0,0,0,{0},{0}};

//oil_displacement

uint8_t displacement_dat = 0;

/* USER CODE END Variables */
/* Definitions for Wit_dat_Task */
osThreadId_t Wit_dat_TaskHandle;
const osThreadAttr_t Wit_dat_Task_attributes = {
  .name = "Wit_dat_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for Can_Rx_Task */
osThreadId_t Can_Rx_TaskHandle;
const osThreadAttr_t Can_Rx_Task_attributes = {
  .name = "Can_Rx_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for RB_Read_Task */
osThreadId_t RB_Read_TaskHandle;
const osThreadAttr_t RB_Read_Task_attributes = {
  .name = "RB_Read_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow4,
};
/* Definitions for OilDisplay_Task */
osThreadId_t OilDisplay_TaskHandle;
const osThreadAttr_t OilDisplay_Task_attributes = {
  .name = "OilDisplay_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow4,
};
/* Definitions for DTU_Init_Task */
osThreadId_t DTU_Init_TaskHandle;
const osThreadAttr_t DTU_Init_Task_attributes = {
  .name = "DTU_Init_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for Signal_4G_Task */
osThreadId_t Signal_4G_TaskHandle;
const osThreadAttr_t Signal_4G_Task_attributes = {
  .name = "Signal_4G_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CanBinarySem */
osSemaphoreId_t CanBinarySemHandle;
osStaticSemaphoreDef_t CanBinarySemControlBlock;
const osSemaphoreAttr_t CanBinarySem_attributes = {
  .name = "CanBinarySem",
  .cb_mem = &CanBinarySemControlBlock,
  .cb_size = sizeof(CanBinarySemControlBlock),
};
/* Definitions for Onenet_tx_BinarySem */
osSemaphoreId_t Onenet_tx_BinarySemHandle;
osStaticSemaphoreDef_t Onenet_tx_BinarySemControlBlock;
const osSemaphoreAttr_t Onenet_tx_BinarySem_attributes = {
  .name = "Onenet_tx_BinarySem",
  .cb_mem = &Onenet_tx_BinarySemControlBlock,
  .cb_size = sizeof(Onenet_tx_BinarySemControlBlock),
};
/* Definitions for Vcu_Event1 */
osEventFlagsId_t Vcu_Event1Handle;
osStaticEventGroupDef_t Vcu_Event1ControlBlock;
const osEventFlagsAttr_t Vcu_Event1_attributes = {
  .name = "Vcu_Event1",
  .cb_mem = &Vcu_Event1ControlBlock,
  .cb_size = sizeof(Vcu_Event1ControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Wit_Dat_Handle(void *argument);
void Can_Rx_Handle(void *argument);
void RingBuffer_Read_Handle(void *argument);
void OilDisplayment_Handle(void *argument);
void DTU_Init_Handle(void *argument);
void Signal_4G_Handle(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of CanBinarySem */
  CanBinarySemHandle = osSemaphoreNew(1, 1, &CanBinarySem_attributes);

  /* creation of Onenet_tx_BinarySem */
  Onenet_tx_BinarySemHandle = osSemaphoreNew(1, 1, &Onenet_tx_BinarySem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Wit_dat_Task */
  Wit_dat_TaskHandle = osThreadNew(Wit_Dat_Handle, NULL, &Wit_dat_Task_attributes);

  /* creation of Can_Rx_Task */
  Can_Rx_TaskHandle = osThreadNew(Can_Rx_Handle, NULL, &Can_Rx_Task_attributes);

  /* creation of RB_Read_Task */
  RB_Read_TaskHandle = osThreadNew(RingBuffer_Read_Handle, NULL, &RB_Read_Task_attributes);

  /* creation of OilDisplay_Task */
  OilDisplay_TaskHandle = osThreadNew(OilDisplayment_Handle, NULL, &OilDisplay_Task_attributes);

  /* creation of DTU_Init_Task */
  DTU_Init_TaskHandle = osThreadNew(DTU_Init_Handle, NULL, &DTU_Init_Task_attributes);

  /* creation of Signal_4G_Task */
  Signal_4G_TaskHandle = osThreadNew(Signal_4G_Handle, NULL, &Signal_4G_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	
	osSemaphoreAcquire(Onenet_tx_BinarySemHandle,osWaitForever);
	osSemaphoreAcquire(CanBinarySemHandle,osWaitForever);
	
	if(NULL != DTU_Init_TaskHandle)/* 创建成功 */
		printf("DTU_Init_TaskHandle任务创建成功!\r\n");
	else
		printf(" DTU_Init_TaskHandle任务创建失败!\r\n");
	if(NULL != Wit_dat_TaskHandle)/* 创建成功 */
		printf("Wit_dat_TaskHandle任务创建成功!\r\n");
	else
		printf("Wit_dat_TaskHandle任务创建失败!\r\n");
	if(NULL != Can_Rx_TaskHandle)/* 创建成功 */
		printf("Can_Rx_TaskHandle任务创建成功!\r\n");
	else
		printf(" Can_Rx_TaskHandle任务创建失败!\r\n");
	if(NULL != RB_Read_TaskHandle)/* 创建成功 */
		printf("RB_Read_TaskHandle任务创建成功!\r\n");
	else
		printf(" RB_Read_TaskHandle任务创建失败!\r\n");
	if(NULL != OilDisplay_TaskHandle)/* 创建成功 */
		printf("OilDisplay_TaskHandle任务创建成功!\r\n");
	else
		printf(" OilDisplay_TaskHandle任务创建失败!\r\n");
	if(NULL != Signal_4G_TaskHandle)/* 创建成功 */
		printf("Signal_4G_TaskHandle任务创建成功!\r\n");
	else
		printf(" Signal_4G_TaskHandle任务创建失败!\r\n");
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of Vcu_Event1 */
  Vcu_Event1Handle = osEventFlagsNew(&Vcu_Event1_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Wit_Dat_Handle */
/**
  * @brief  Function implementing the Wit_dat_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Wit_Dat_Handle */
void Wit_Dat_Handle(void *argument)
{
  /* USER CODE BEGIN Wit_Dat_Handle */
  /* Infinite loop */
	int i;
	uint8_t wit_id = Wit_Device_ID;
	uint8_t Send_date[128] = {0};
  for(;;)
  {
		if(osEventFlagsWait (Vcu_Event1Handle, EVENTBIT_0,osFlagsWaitAny, osWaitForever)&EVENTBIT_0){
			CmdProcess();
			if(s_cDataUpdate)
			{
				for(i = 0; i < 3; i++)
				{
					wit_dat1.fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
					wit_dat1.fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
					wit_dat1.fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
				}
				if(s_cDataUpdate & ACC_UPDATE)
				{
//					printf("acc:%.3f %.3f %.3f\r\n", wit_dat1.fAcc[0], wit_dat1.fAcc[1], wit_dat1.fAcc[2]);
					s_cDataUpdate &= ~ACC_UPDATE;
				}
				if(s_cDataUpdate & GYRO_UPDATE)
				{
//					printf("gyro:%.3f %.3f %.3f\r\n", wit_dat1.fGyro[0], wit_dat1.fGyro[1], wit_dat1.fGyro[2]);
					s_cDataUpdate &= ~GYRO_UPDATE;
				}
				if(s_cDataUpdate & ANGLE_UPDATE)
				{
//					printf("angle:%.3f %.3f %.3f\r\n", wit_dat1.fAngle[0], wit_dat1.fAngle[1], wit_dat1.fAngle[2]);
					s_cDataUpdate &= ~ANGLE_UPDATE;
				}
				if(s_cDataUpdate & MAG_UPDATE)
				{
//					printf("mag:%d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]);
					s_cDataUpdate &= ~MAG_UPDATE;
				}
			}
			if(dtu_device1.dtu_t_threadflag[2] == 1&&dtu_device1.Onenet_Off_flag == 0){
				OneNet_Receive(Send_date,wit_id,sizeof(Send_date));
			}
			vTaskDelay(Wit_Dat_Handle_Delay);
		}
  }
  /* USER CODE END Wit_Dat_Handle */
}

/* USER CODE BEGIN Header_Can_Rx_Handle */
/**
* @brief Function implementing the Can_Rx_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Can_Rx_Handle */
void Can_Rx_Handle(void *argument)
{
  /* USER CODE BEGIN Can_Rx_Handle */
  /* Infinite loop */

  for(;;)
  {
		osSemaphoreAcquire(CanBinarySemHandle,osWaitForever);
    osDelay(1);
  }
  /* USER CODE END Can_Rx_Handle */
}

/* USER CODE BEGIN Header_RingBuffer_Read_Handle */
/**
* @brief Function implementing the RB_Read_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RingBuffer_Read_Handle */
void RingBuffer_Read_Handle(void *argument)
{
  /* USER CODE BEGIN RingBuffer_Read_Handle */
  /* Infinite loop */
	
	uint8_t buf = 0;
	char *token;
	char DTU_Dat[6][64] = {0};
	const char DTU_ATCLK[] = "AT+CLK";
	const char DTU_ATCSQ[] = "AT+CSQ";
	uint8_t i = 0,CSQvalue = 0;
  for(;;)
  {
		if(osEventFlagsWait (Vcu_Event1Handle, EVENTBIT_1,osFlagsNoClear, 1)&EVENTBIT_1)
		{
//			if (RingBuffer_Len(p_uart2_rxbuf) > 0)          /*接收到DTU传送过来的服务器数据*/
//			{
////				RingBuffer_Out(p_uart2_rxbuf, &buf, 1);
//				dtu_device1.dtu_rxbuf[dtu_device1.dtu_rxlen++] = buf;
//				dtu_get_urc_info(buf);                      /*解析DTU上报的URC信息*/
//				if (dtu_device1.dtu_rxlen >= DTU_ONENETDATE_RX_BUF)     /*接收缓存溢出*/
//				{
//						HAL_UART_Transmit(&huart1,dtu_device1.dtu_rxbuf,dtu_device1.dtu_rxlen,0xff);
//						dtu_device1.dtu_rxlen = 0;
//				}
//			}
//			elsedtu_device1.dtu_rxlen
			{
					if (rx_len > 0)
					{
//						printf("%s\r\n",rx_4g_buffer);
							if(strcmp((char *)dtu_device1.dtu_rxbuf,ONENET_COM_OFF)== 0)
							{
	#if TEST_PRINTF_RINGBUFFER					
								printf("data = %s\r\n",dtu_device1.dtu_rxbuf);
	#endif							
								dtu_device1.Onenet_Off_flag = 1;
							}
							else if(strcmp((char *)dtu_device1.dtu_rxbuf,ONENET_COM_ON)== 0)
							{
								dtu_device1.Onenet_Off_flag = 0;				
							}
							/* 获取第一个子字符串 */
							token = strtok((char *)dtu_device1.dtu_rxbuf, sqpa[0]);
							/* 继续获取其他的子字符串 */
							while( token != NULL )
							{
								i++;
								strcpy(DTU_Dat[i],token);
	#if TEST_PRINTF_RINGBUFFER
								printf( "%s\r\n", DTU_Dat[i]);					
	#endif
								token = strtok(NULL, sqpa[0]);
							}
							if(strcmp(DTU_Dat[1],DTU_ATCSQ) == 0)
							{
								CSQvalue = DTU_AT_CSQ_DataAnalyze(DTU_Dat);
								if(CSQvalue > 14){
								dtu_device1.Network_size = 5;}
								else if(CSQvalue > 9&&CSQvalue <= 14){
								dtu_device1.Network_size = 4;}
								else if(CSQvalue > 5&&CSQvalue <= 9){
								dtu_device1.Network_size = 3;}
								else if(CSQvalue > 2&&CSQvalue <= 5){
								dtu_device1.Network_size = 2;}
								else if(CSQvalue > 1&&CSQvalue <= 2){
								dtu_device1.Network_size = 1;}
								else if(CSQvalue == 0){
								dtu_device1.Network_size = 0;}
	#if TEST_PRINTF_RINGBUFFER
								printf("CSQvalue = %d\r\n",CSQvalue);	
	#endif
							}
							else if(strcmp(DTU_Dat[1],DTU_ATCLK) == 0)
							{
								DTU_AT_CLK_DataAnalyze(DTU_Dat);
								dtu_device1.Dtu_At_Clkflag = 1;
	#if TEST_PRINTF_RINGBUFFER
								printf("%d \r\n",Timedat.year);
								printf("%d \r\n",Timedat.month);
								printf("%d \r\n",Timedat.day);
								printf("%d \r\n",Timedat.hour);
								printf("%d \r\n",Timedat.minute);
								printf("%d \r\n",Timedat.second);
								printf("CLKvalue = %s\r\n",DTU_Dat[1]);
	#endif	
							}
							i = 0;//dtu_device1.dtu_rxlen = 0;
							rx_len = 0;
					}	
				}
			}
		vTaskDelay(3);	
		}
  /* USER CODE END RingBuffer_Read_Handle */
}

/* USER CODE BEGIN Header_OilDisplayment_Handle */
/**
* @brief Function implementing the OilDisplay_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OilDisplayment_Handle */
void OilDisplayment_Handle(void *argument)
{
  /* USER CODE BEGIN OilDisplayment_Handle */
  /* Infinite loop */
	extern uint32_t oildisplay_value;
	uint8_t oildisplay[64] = {0};
  for(;;)
  {
		displacement_dat = oildisplay_value*50/4096;
		printf("%d\r\n",displacement_dat);
		if(dtu_device1.dtu_t_threadflag[1] == 1&&dtu_device1.Onenet_Off_flag == 0){
		OneNet_Receive(oildisplay,XJ1_Device_ID,sizeof(oildisplay));
		}
		HAL_ADC_Start_DMA(&hadc1,&oildisplay_value,sizeof(oildisplay_value)/4);
    osDelay(OD_Handle_Delay);
  }
  /* USER CODE END OilDisplayment_Handle */
}

/* USER CODE BEGIN Header_DTU_Init_Handle */
/**
* @brief Function implementing the DTU_Init_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DTU_Init_Handle */
void DTU_Init_Handle(void *argument)
{
  /* USER CODE BEGIN DTU_Init_Handle */
  /* Infinite loop */
	uint16_t timeout = 0;
	int ret;
  for(;;)
  {
		my_mem_init(SRAMIN);
		p_uart2_rxbuf = RingBuffer_Malloc(1024);        /*从内存池中分配1K的内存给串口3接收DTU数据*/
		printf("Wait for Cat1 DTU to start, wait 10s.... \r\n");
		while( timeout <= 10 )   /* 等待Cat1 DTU启动，需要等待5-6s才能启动 */
		{
				ret = dtu_config_init(DTU_WORKMODE_ONENET);    /*初始化DTU工作参数*/
				if( ret == 0 ){
					dtu_device1.dtu_t_threadflag[0] = 1;
					printf("Cat1 DTU Init Success \r\n");
					break;
				}
				timeout++;
				osDelay(1000);
		}
		while( timeout > 10 ){   /* 超时 */
			printf("**************************************************************************\r\n");
			printf("ATK-DTU Init Fail ...\r\n");
			printf("请按照以下步骤进行检查:\r\n");
			printf("1.使用电脑上位机配置软件检查DTU能否单独正常工作\r\n");
			printf("2.检查DTU串口参数与STM32通讯的串口参数是否一致\r\n");
			printf("3.检查DTU与STM32串口的接线是否正确\r\n");
			printf("4.检查DTU供电是否正常，DTU推荐使用12V/1A电源供电，不要使用USB的5V给模块供电！！\r\n");
			printf("**************************************************************************\r\n\r\n");
			osDelay(1000);
			break;
		}
    if(dtu_device1.dtu_t_threadflag[0] == 1){
			send_data_to_dtu("AT+CLK\r\n", strlen("AT+CLK\r\n"));
			osEventFlagsSet(Vcu_Event1Handle, EVENTBIT_1);
			osDelay(500);
			osEventFlagsSet(Vcu_Event1Handle, EVENTBIT_2);
		}
		vTaskDelete(DTU_Init_TaskHandle);
	}	
  /* USER CODE END DTU_Init_Handle */
}

/* USER CODE BEGIN Header_Signal_4G_Handle */
/**
* @brief Function implementing the Signal_4G_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Signal_4G_Handle */
void Signal_4G_Handle(void *argument)
{
  /* USER CODE BEGIN Signal_4G_Handle */
  /* Infinite loop */
	uint8_t j = 0,i = 0;
	int ret;
	uint8_t signal_dat[64] = {0};
  for(;;)
  {
		if(dtu_device1.dtu_t_threadflag[0] == 1){
//			if(osEventFlagsWait (Vcu_Event1Handle, EVENTBIT_2,osFlagsNoClear, osWaitForever)&EVENTBIT_2){
//				while(j<10){
//					osKernelLock ();		
//					for(i=1;i<3;i++){
//						dtu_device1.dtu_t_threadflag[i] = 0;
//					}		
//					osKernelUnlock ();
//					/*1.DTU进入配置状态*/
//				  osKernelLock ();
//			    ret = dtu_enter_configmode();
//			    osKernelUnlock ();
//					if ( ret != 0 ){
//						printf("进入配置失败\r\n");
//						vTaskDelay(10);
//						j++;
//					}
//					else{
//						printf("进入配置成功\r\n");
//						dtu_device1.Dtumode_Switch_flag = 0;
//						send_data_to_dtu("AT+CSQ\r\n", strlen("AT+CSQ\r\n"));
//						break;
//					}
//				}
//				j = 0;
//				osDelay(50);
//				OneNet_Receive(signal_dat,DTU_Device_ID,sizeof(signal_dat));
//				osKernelLock ();			
//				for(i=1;i<3;i++){
//					 dtu_device1.dtu_t_threadflag[i] = 1;
//				}		
//				osKernelUnlock ();
//			}
		}
    osDelay(DTU_Signal_Delay);
  }
  /* USER CODE END Signal_4G_Handle */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t i = 0;
	if(hcan->Instance == CAN1)
	{
		RxMessage.StdId=0x00;
		RxMessage.ExtId=0x00;
		RxMessage.IDE=0;
		RxMessage.DLC=0;
		RxMessage.RTR=0;
		RxMessage.Timestamp=0;
		RxMessage.FilterMatchIndex = 0;
		for(i = 0; i < 8; i++)
		{
			canrx_dat[i]=0x00;
		}
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&RxMessage,canrx_dat);
		if((RxMessage.StdId==Wit_dat) && (RxMessage.IDE==CAN_ID_STD) && (RxMessage.DLC==8))
		{
			WitCanDataIn(canrx_dat, RxMessage.DLC);
			osEventFlagsSet (Vcu_Event1Handle, EVENTBIT_0);
		}
		osSemaphoreRelease(CanBinarySemHandle);
	}
}

static void OneNet_Receive(uint8_t *Send_date,uint8_t device_id,uint8_t dat_len)
{
	uint8_t Sdat[3] = {3,0,0x46};
	int res = 0;
	uint8_t j = 0;
	if(dtu_device1.Dtumode_Switch_flag == 0){             															 //判断是否处于控制阶段 0处于配置状态 1处于透传状态
		while(j<5){
					/*DTU进入透传状态*/
			 osKernelLock ();
			 res = dtu_enter_transfermode();
			 osKernelUnlock ();
			 if( res != 0 ){
				 j++;
					printf("DTU进入透传状态失败\r\n");
				 osDelay(10);
			 }
			 else{
				 dtu_device1.Dtumode_Switch_flag = 1;
				 printf("DTU进入透传状态\r\n");
				 osKernelLock ();
				 OneNet_FillBuf(Send_date,device_id);
				 HAL_UART_Transmit(&huart2,&Sdat[0],sizeof(Sdat[0]),0xff);
				 HAL_UART_Transmit(&huart2,&Sdat[1],sizeof(Sdat[1]),0xff);
				 HAL_UART_Transmit(&huart2,&Sdat[2],sizeof(Sdat[2]),0xff);
				 HAL_UART_Transmit(&huart2,Send_date,dat_len,0xff);
				 osKernelUnlock ();
				 break;
			 }		
		}
	 }
	 else{
			osKernelLock ();
			OneNet_FillBuf(Send_date,device_id);
			HAL_UART_Transmit(&huart2,Send_date,dat_len,0xff);
			osKernelUnlock ();
	 }
}


/**********************************************************************
  * @ 函数名  ： OneNet_FillBuf
  * @ 功能说明： 封装数据
  * @ 参数    ： OneNet_FillBuf(uint8_t *buff,uint8_t devicer_id)
  * @ 返回值  ： 无
  ********************************************************************/
static void OneNet_FillBuf(uint8_t *buff,uint8_t devicer_id)
{
	switch(devicer_id)
	{
		case(Wit_Device_ID):{
			memset(buff, 0, sizeof(buff));
			sprintf((char *)buff, "{'WIT_ACC_AYRO_ANGLE':'\n%.3f,%.3f,%.3f\n%.3f,%.3f,%.3f\n%.3f,%.3f,%.3f\n'}",wit_dat1.fAcc[0], wit_dat1.fAcc[1], wit_dat1.fAcc[2]\
			            ,wit_dat1.fGyro[0], wit_dat1.fGyro[1], wit_dat1.fGyro[2]\
			            ,wit_dat1.fAngle[0], wit_dat1.fAngle[1], wit_dat1.fAngle[2]); //WIT_ACC_AYRO_ANGLE是数据流的一个名称，wit_dat1是数据值;
			break;
		}
		case(XJ1_Device_ID):{
			memset(buff, 0, sizeof(buff));
			sprintf((char *)buff, "{'Oil_Displacement':'%d'}",displacement_dat);
			break;
		}
		case(GPS_Device_ID):{
			break;
		}
		case(DTU_Device_ID):{
			memset(buff, 0, sizeof(buff));
			sprintf((char *)buff, "{'4G_signal':'%d'}",dtu_device1.Network_size);
			break;
		}
		default:break;
	}
}

/* USER CODE END Application */

