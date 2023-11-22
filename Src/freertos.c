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
#include "stdlib.h"
#include "gps.h"
#include "usart.h"
#include "mtspeed.h"
#include "fatfs.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
typedef StaticEventGroup_t osStaticEventGroupDef_t;
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define Q_TOOTH_NUM 24			 //齿数
#define F_TOOTH_NUM 48			 //齿数
#define J_TOOTH_NUM 50			 //减速器齿数
#define WHEEL_RADIUS 0.257 //轮径(单位:m)
#define PI 3.14						 //圆周率
#define TANK_HEIGHT 210		 //油箱高度
#define GEAR_RATIO  3.45    //传动比

#define XJWY_StdId							0x01
#define GPS_StdId								0x02
#define CLK_StdId								0x03
#define CLK_StdId2								0x13
#define DTU_StdId								0x04       //0x04     csq
#define OADC_StdId              0x05
#define Tyre_StdId              0x08
#define Wit_StdId_xy   						0x51      //0x51,0x52  angle_value-xy,z
#define Wit_dat   							((uint32_t)0x50) //标识符ID：0x50    (标准帧)

#define Wit_Device_ID        ((uint8_t)0x01)
#define XJ_Device_ID        ((uint8_t)0x02)             //油门踏板
#define GPS_Device_ID        ((uint8_t)0x03)
#define DTU_Device_ID        ((uint8_t)0x04)
#define TYRE_Device_ID       ((uint8_t)0x05)      

/*
*************************************************************************
*               thread interval allocation
*************************************************************************
*/
#define   Wit_Dat_Handle_Delay          30
#define   Type_Dat_Handle_Delay         200
#define   Can_Rx_Handle_Delay           1
#define   GPS_Handle_Delay              100
#define   OD_Handle_Delay               200
#define   ADC_Handle_Delay              20000
#define   DTU_Signal_Delay              30000
#define   RTC_Delay                     900

/*
*************************************************************************
*                event flag
*************************************************************************
*/


#define EVENTBIT_0	(1<<0)				//九轴事件
#define EVENTBIT_1	(1<<1)				//接受4g反馈信息事件
#define EVENTBIT_2	(1<<2)				//4g信号事件

#define EVENTBIT_3	(1<<3)				//判断是否处于控制阶段 0处于配置状态 1处于透传状态
#define EVENTBIT_4	(1<<4)

#define EVENTBIT_5	(1<<5)				//暂未使用
#define EVENTBIT_6	(1<<6)        //暂未使用
#define EVENTBIT_7	(1<<7)				//暂未使用


#define EVENTBIT_8	(1<<8)				// GPS数据采集时间
#define EVENTBIT_9	(1<<9)				//暂未使用
#define EVENTBIT_10 (1<<10)       //暂未使用


#define MT_EVENTBIT_1	(1<<0)			
#define MT_EVENTBIT_2	(1<<1)			
#define MT_EVENTBIT_3	(1<<2)		  
#define MT_EVENTBIT_4	(1<<3)
#define MT_EVENTBIT_5	(1<<4)

#define SD_EVENTBIT_1	(1<<0)			//sd卡悬架位移事件
#define SD_EVENTBIT_2	(1<<1)			//sd九轴数据事件
#define SD_EVENTBIT_3	(1<<2)		  //sd轮速数据事件
#define SD_EVENTBIT_4	(1<<3)			//gps速度事件

#define CAN_EVENTBIT_1	(1<<0)			
#define CAN_EVENTBIT_2	(1<<1)			
#define CAN_EVENTBIT_3	(1<<2)		  
#define CAN_EVENTBIT_4	(1<<3)
#define CAN_EVENTBIT_5	(1<<4)			
#define CAN_EVENTBIT_6	(1<<5)			
#define CAN_EVENTBIT_7	(1<<6)		  
#define CAN_EVENTBIT_8	(1<<7)		

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

CAN_RxHeaderTypeDef RxMessage;

//IWDG
extern IWDG_HandleTypeDef hiwdg;


//usart1,2,3
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

//timer
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;


extern uint8_t USART3_RX_BUF[USART3_MAX_RECV_LEN]; 				//接收缓冲,最大USART3_MAX_RECV_LEN个字节.
extern uint8_t  USART3_TX_BUF[USART3_MAX_SEND_LEN]; 			//发送缓冲,最大USART3_MAX_SEND_LEN字节
uint8_t USART1_TX_BUF[USART3_MAX_RECV_LEN]; 					//串口1,发送缓存区

__IO uint16_t USART3_RX_STA=0;
extern uint8_t rx_4g_buffer[128];
extern uint8_t rx_len;
extern uint8_t rx_flag;
extern uint8_t usart_rx_char;
//adc
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
__IO uint8_t ADC1_Flag = 0;
__IO uint8_t ADC2_Flag = 0;
extern  uint32_t adc1_value[60];

//九轴传感器
extern CAN_HandleTypeDef hcan1;
extern RegUpdateCb p_WitRegUpdateCbFunc;      
extern volatile char  s_cDataUpdate;
wit_can_dat wit_dat1 = {{0,0,0},{0,0,0},{0,0,0}};

union Anglefloat
{
	float value;
	uint8_t data[4];
}Anglefloat_tx[3];

union Accfloat
{
	float value;
	uint8_t data[4];
}Accfloat_tx[3];

//can
uint8_t canrx_dat[8] = {0};
uint8_t wit_msg[3][8] = {0};
uint8_t xjwy_msg[7] = {0};
uint8_t Clkvalue[6] = {0};
uint8_t oadc_msg[2] = {0};
uint8_t tyre_msg[8] = {0};

//onenet
extern const char sqpa[4][4];
const char ONENET_COM_OFF[]="lc0218";
const char ONENET_COM_ON[] = "lc2001";
RingBuffer *p_uart2_rxbuf;
_dtu_4g_device dtu_device1 = {0,0,0,{0},1,0};

//Tyre_speed


extern uint8_t time3_over1,time3_over2;

mt_rotate mtspeed1 = {0,0,0,0,0,0,0,0,0,0};
mt_rotate mtspeed2 = {0,0,0,0,0,0,0,0,0,0};
mt_rotate mtspeed3 = {0,0,0,0,0,0,0,0,0,0};
mt_rotate mtspeed4 = {0,0,0,0,0,0,0,0,0,0};
mt_rotate mtspeed5 = {0,0,0,0,0,0,0,0,0,0};

//位移传感器结构体

typedef struct
{
	uint8_t displacement_dat;
	uint8_t xjwy1_dat;
	uint8_t xjwy2_dat;
	uint8_t xjwy3_dat;
	uint8_t xjwy4_dat;
	uint8_t oilyy_dat[2];
}_wy_dat;

_wy_dat Wy_dat = {0,0,0,0,0,0};

//GPS

union speed_float
{
	float value;
	uint8_t data[4];
}speedfloat_tx;

uint8_t UpdateTime_flag = 0;

nmea_msg gpsx; 											//GPS信息
const uint8_t *fixmode_tbl[4]={"Fail","Fail"," 2D "," 3D "};	//fix mode字符串 

//SD
extern FATFS SDFatFS;    /* File system object for SD logical drive */
extern FIL SDFile;       /* File object for SD */
extern SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypeDef  SDCardInfo;    

//oil	
extern uint32_t adc2_value[20];
__IO uint8_t Oil_base_dat = 97;                     //油量最终基准值


//RTC
extern RTC_HandleTypeDef hrtc;

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
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow5,
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
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for GPS_Init_Task */
osThreadId_t GPS_Init_TaskHandle;
const osThreadAttr_t GPS_Init_Task_attributes = {
  .name = "GPS_Init_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for GPS_Get_Task */
osThreadId_t GPS_Get_TaskHandle;
const osThreadAttr_t GPS_Get_Task_attributes = {
  .name = "GPS_Get_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow3,
};
/* Definitions for IWDG_Task */
osThreadId_t IWDG_TaskHandle;
const osThreadAttr_t IWDG_Task_attributes = {
  .name = "IWDG_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Otheradc_task */
osThreadId_t Otheradc_taskHandle;
const osThreadAttr_t Otheradc_task_attributes = {
  .name = "Otheradc_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CantxTask */
osThreadId_t CantxTaskHandle;
uint32_t CantxTaskBuffer[ 256 ];
osStaticThreadDef_t CantxTaskControlBlock;
const osThreadAttr_t CantxTask_attributes = {
  .name = "CantxTask",
  .cb_mem = &CantxTaskControlBlock,
  .cb_size = sizeof(CantxTaskControlBlock),
  .stack_mem = &CantxTaskBuffer[0],
  .stack_size = sizeof(CantxTaskBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for TyreSpeedTask */
osThreadId_t TyreSpeedTaskHandle;
const osThreadAttr_t TyreSpeedTask_attributes = {
  .name = "TyreSpeedTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for SD_Init_task */
osThreadId_t SD_Init_taskHandle;
const osThreadAttr_t SD_Init_task_attributes = {
  .name = "SD_Init_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RTCTask */
osThreadId_t RTCTaskHandle;
uint32_t RTCTaskBuffer[ 128 ];
osStaticThreadDef_t RTCTaskControlBlock;
const osThreadAttr_t RTCTask_attributes = {
  .name = "RTCTask",
  .cb_mem = &RTCTaskControlBlock,
  .cb_size = sizeof(RTCTaskControlBlock),
  .stack_mem = &RTCTaskBuffer[0],
  .stack_size = sizeof(RTCTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UsartQueue */
osMessageQueueId_t UsartQueueHandle;
const osMessageQueueAttr_t UsartQueue_attributes = {
  .name = "UsartQueue"
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
/* Definitions for TyreBinarySem */
osSemaphoreId_t TyreBinarySemHandle;
osStaticSemaphoreDef_t TypeBinarySemControlBlock;
const osSemaphoreAttr_t TyreBinarySem_attributes = {
  .name = "TyreBinarySem",
  .cb_mem = &TypeBinarySemControlBlock,
  .cb_size = sizeof(TypeBinarySemControlBlock),
};
/* Definitions for Vcu_Event1 */
osEventFlagsId_t Vcu_Event1Handle;
osStaticEventGroupDef_t Vcu_Event1ControlBlock;
const osEventFlagsAttr_t Vcu_Event1_attributes = {
  .name = "Vcu_Event1",
  .cb_mem = &Vcu_Event1ControlBlock,
  .cb_size = sizeof(Vcu_Event1ControlBlock),
};
/* Definitions for Can_Event */
osEventFlagsId_t Can_EventHandle;
osStaticEventGroupDef_t Can_EventControlBlock;
const osEventFlagsAttr_t Can_Event_attributes = {
  .name = "Can_Event",
  .cb_mem = &Can_EventControlBlock,
  .cb_size = sizeof(Can_EventControlBlock),
};
/* Definitions for MT_Event */
osEventFlagsId_t MT_EventHandle;
osStaticEventGroupDef_t MT_EventControlBlock;
const osEventFlagsAttr_t MT_Event_attributes = {
  .name = "MT_Event",
  .cb_mem = &MT_EventControlBlock,
  .cb_size = sizeof(MT_EventControlBlock),
};
/* Definitions for SD_Event */
osEventFlagsId_t SD_EventHandle;
osStaticEventGroupDef_t SD_EventControlBlock;
const osEventFlagsAttr_t SD_Event_attributes = {
  .name = "SD_Event",
  .cb_mem = &SD_EventControlBlock,
  .cb_size = sizeof(SD_EventControlBlock),
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
void GPS_Init_Handle(void *argument);
void GPS_Get_Handle(void *argument);
void IWDG_Handle(void *argument);
void Oadc_Handle(void *argument);
void Cantx_Handle(void *argument);
void TyreSpeed_Handle(void *argument);
void SD_Init_Handle(void *argument);
void RTC_Handle(void *argument);

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

  /* creation of TyreBinarySem */
  TyreBinarySemHandle = osSemaphoreNew(1, 1, &TyreBinarySem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of UsartQueue */
  UsartQueueHandle = osMessageQueueNew (4, 64, &UsartQueue_attributes);

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

  /* creation of GPS_Init_Task */
  GPS_Init_TaskHandle = osThreadNew(GPS_Init_Handle, NULL, &GPS_Init_Task_attributes);

  /* creation of GPS_Get_Task */
  GPS_Get_TaskHandle = osThreadNew(GPS_Get_Handle, NULL, &GPS_Get_Task_attributes);

  /* creation of IWDG_Task */
  IWDG_TaskHandle = osThreadNew(IWDG_Handle, NULL, &IWDG_Task_attributes);

  /* creation of Otheradc_task */
  Otheradc_taskHandle = osThreadNew(Oadc_Handle, NULL, &Otheradc_task_attributes);

  /* creation of CantxTask */
  CantxTaskHandle = osThreadNew(Cantx_Handle, NULL, &CantxTask_attributes);

  /* creation of TyreSpeedTask */
  TyreSpeedTaskHandle = osThreadNew(TyreSpeed_Handle, NULL, &TyreSpeedTask_attributes);

  /* creation of SD_Init_task */
  SD_Init_taskHandle = osThreadNew(SD_Init_Handle, NULL, &SD_Init_task_attributes);

  /* creation of RTCTask */
  RTCTaskHandle = osThreadNew(RTC_Handle, NULL, &RTCTask_attributes);

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
		printf(" Wit_dat_TaskHandle任务创建失败!\r\n");
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
	if(NULL != GPS_Init_TaskHandle)/* 创建成功 */
		printf("GPS_Init_TaskHandle任务创建成功!\r\n");
	else
		printf(" GPS_Init_TaskHandle任务创建失败!\r\n");
	if(NULL != GPS_Get_TaskHandle)/* 创建成功 */
		printf("GPS_Get_TaskHandle任务创建成功!\r\n");
	else
		printf(" GPS_Get_TaskHandle任务创建失败!\r\n");
	if(NULL != IWDG_TaskHandle)/* 创建成功 */
		printf("IWDG_TaskHandle任务创建成功!\r\n");
	else
		printf(" IWDG_TaskHandle任务创建失败!\r\n");
	if(NULL != Otheradc_taskHandle)/* 创建成功 */
		printf("Otheradc_taskHandle任务创建成功!\r\n");
	else
		printf(" Otheradc_taskHandle任务创建失败!\r\n");
	if(NULL != Cantx_Handle)/* 创建成功 */
		printf("Cantx_Handle任务创建成功!\r\n");
	else
		printf(" Cantx_Handle任务创建失败!\r\n");
	if(NULL != TyreSpeedTaskHandle)/* 创建成功 */
		printf("TyreSpeedTaskHandle任务创建成功!\r\n");
	else
		printf(" TyreSpeedTaskHandle任务创建失败!\r\n");	
	if(NULL != SD_Init_taskHandle)/* 创建成功 */
		printf("SD_Init_taskHandle任务创建成功!\r\n");
	else
		printf(" SD_Init_taskHandle任务创建失败!\r\n");	
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of Vcu_Event1 */
  Vcu_Event1Handle = osEventFlagsNew(&Vcu_Event1_attributes);

  /* creation of Can_Event */
  Can_EventHandle = osEventFlagsNew(&Can_Event_attributes);

  /* creation of MT_Event */
  MT_EventHandle = osEventFlagsNew(&MT_Event_attributes);

  /* creation of SD_Event */
  SD_EventHandle = osEventFlagsNew(&SD_Event_attributes);

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
	uint8_t i,m,n;
	FRESULT res_flash;
	UINT wit_bw;
	uint8_t Wit_date[128] = {0};
	static uint8_t witnum = 0;
	float Sd_wit_dat[10][9]= {0};
	char Sd_wit[128] = {0};
  for(;;)
  {
		if(osEventFlagsWait (Vcu_Event1Handle, EVENTBIT_0,osFlagsWaitAny, osWaitForever)&EVENTBIT_0){
			CmdProcess();
			if(s_cDataUpdate)
			{
				for(i = 0; i < 3; i++){
					wit_dat1.fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
					Accfloat_tx[i].value = wit_dat1.fAcc[i];
					wit_dat1.fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
					wit_dat1.fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
					Anglefloat_tx[i].value = wit_dat1.fAngle[i];
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
			for(i = 0; i < 8; i++){
				if(i<4){
					wit_msg[0][i] = Anglefloat_tx[0].data[i];
				}else{
					wit_msg[0][i] = Anglefloat_tx[1].data[i-4];
				}
			}
			for(i = 0; i < 8; i++){
				if(i<4){
					wit_msg[1][i] = Anglefloat_tx[2].data[i];
				}else{
					wit_msg[1][i] = Accfloat_tx[0].data[i-4];
				}
			}
			for(i = 0; i < 8; i++){
				if(i<4){
					wit_msg[2][i] = Accfloat_tx[1].data[i];
				}else{
					wit_msg[2][i] = Accfloat_tx[2].data[i-4];
				}
			}
			osEventFlagsSet(Can_EventHandle, CAN_EVENTBIT_1);
			if(dtu_device1.Onenet_Off_flag == 0&&dtu_device1.st_flag == 0){
				OneNet_Send(Wit_date,Wit_Device_ID,sizeof(Wit_date));
			}
			for(m = 0;m<3;m++){
				switch(m){
					case(0):{
						for(n = 0;n<3;n++){
							Sd_wit_dat[witnum][n] = wit_dat1.fAcc[n];
						}
						break;
					}
					case(1):{
						for(n = 0;n<3;n++){
							Sd_wit_dat[witnum][n+3] = wit_dat1.fGyro[n];
						}
						break;
					}
					case(2):{
						for(n = 0;n<3;n++){
							Sd_wit_dat[witnum][n+6] = wit_dat1.fAngle[n];
						}
						break;
					}
					default:{
						printf("Sd_wit_dat error!!\r\n");
						break;
					}
				}
		 }
		 if((witnum++)>=10){
				witnum = 0;
				if(osEventFlagsGet (SD_EventHandle)&SD_EVENTBIT_2){
					res_flash = f_open(&SDFile, "0:/FWT_dat/witdat.txt",FA_OPEN_APPEND | FA_WRITE);
					if ( res_flash == FR_OK ){
//						printf("open FWT_dat/witdat.txt success!!\r\n");
						memset(Sd_wit, 0, sizeof(Sd_wit));
						sprintf(Sd_wit,"\r\n%04d/%02d/%02d %02d:%02d:%02d",gpsx.utc.year,gpsx.utc.month,gpsx.utc.date,gpsx.utc.hour,gpsx.utc.min,gpsx.utc.sec);
						f_write(&SDFile,Sd_wit,sizeof(Sd_wit),&wit_bw);
						for(uint8_t k = 0;k<10;k++){
							memset(Sd_wit, 0, sizeof(Sd_wit));
							sprintf(Sd_wit,"\r\nacc:%.3f  %.3f %.3f g\r\ngyro:%.3f %.3f %.3f\r\nangle:%.3f %.3f %.3f"\
									    ,Sd_wit_dat[k][0],Sd_wit_dat[k][1],Sd_wit_dat[k][2]\
											,Sd_wit_dat[k][3],Sd_wit_dat[k][4],Sd_wit_dat[k][5]\
											,Sd_wit_dat[k][6],Sd_wit_dat[k][7],Sd_wit_dat[k][8]);
							f_write(&SDFile,Sd_wit,sizeof(Sd_wit),&wit_bw);
						}
					}else{
						printf("open FWT_dat/witdat.txt fail (%d)!!",res_flash);
					}
					f_close(&SDFile);
				}
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

	uint8_t i = 0,j = 0;
//	char *token;
	char DTU_Csq[5] = {0};
	uint8_t CSQvalue = 0;
	uint8_t receive[35] = {0};
  for(;;)
  {
		if(osEventFlagsGet (Vcu_Event1Handle)&EVENTBIT_1)
		{
			memset(receive, 0, sizeof(receive));
			osMessageQueueGet (UsartQueueHandle, receive, NULL, osWaitForever);
			if(strcmp((char *)receive,ONENET_COM_OFF)== 0)
			{
//			printf("data = %s\r\n",receive);
				dtu_device1.Onenet_Off_flag = 1;
			}
			else if(strcmp((char *)receive,ONENET_COM_ON)== 0)
			{
				dtu_device1.Onenet_Off_flag = 0;				
			}
			else if((strstr((char *)receive, "+CSQ") == ((char *)receive+2)))
			{
				osKernelLock ();
				//\n+CSQ: 17,99
//				printf("%s\r\n",receive);
				for( i = 8;i<10;i++){
					DTU_Csq[j] = receive[i];
					j++;
					if(j >= 2){
					CSQvalue = atoi(DTU_Csq);
//					printf("%d\r\n",CSQvalue);
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
					j = 0;
					}
				}
				osKernelUnlock ();
				osEventFlagsSet(Can_EventHandle, CAN_EVENTBIT_8);
//				printf("Network_size\r\n");
			 }
		}
		 vTaskDelay(50);	
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
	uint32_t oildisplay_value,xjwy_1,xjwy_2,xjwy_3,xjwy_4,oil_yy;
	uint8_t oildisplay[128] = {0};
	uint8_t sdxjwydat[10][7] = {0};
	char sd_xjwy[128] = {0};
	static uint8_t sdxjwynum = 0;
	UINT xjwy_bw;
	FRESULT res_flash;
  for(;;)
  {
		if(ADC1_Flag == 1)
		{
			ADC1_Flag = 0;
      oildisplay_value = 0;
			xjwy_1 = 0; xjwy_2 = 0; xjwy_3 = 0; xjwy_4 = 0; oil_yy = 0;
			for(uint8_t i=0;i<60;){
			  xjwy_1+=adc1_value[i++];                       //左前
			  xjwy_2+=adc1_value[i++];                       //右前
				xjwy_3+=adc1_value[i++];                       //左后
			  xjwy_4+=adc1_value[i++];                       //右后
  			oil_yy+=adc1_value[i++];                       //油压
				oildisplay_value += adc1_value[i++];           //油门线位
			}
			Wy_dat.displacement_dat = oildisplay_value*5/4096;    //oildisplay_value/10*50/4096
			Wy_dat.xjwy1_dat = xjwy_1*22.5/4096;
			Wy_dat.xjwy2_dat = xjwy_2*22.5/4096;
			Wy_dat.xjwy3_dat = xjwy_3*22.5/4096;
			Wy_dat.xjwy4_dat = xjwy_4*22.5/4096;
			Wy_dat.oilyy_dat[0] = (int)(oil_yy*2000/4096)/100;
			Wy_dat.oilyy_dat[1] = (int)(oil_yy*2000/4096)%100;
//			printf("oilyy_dat:%d mv\r\n",oil_yy*3300/40960);
//			printf("Wy_dat.xjwy1_dat:%d mv\r\n",Wy_dat.xjwy1_dat);
//			printf("Wy_dat.xjwy2_dat:%d mv\r\n",Wy_dat.xjwy2_dat);
//			printf("Wy_dat.xjwy3_dat:%d mv\r\n",Wy_dat.xjwy3_dat);
//			printf("Wy_dat.xjwy4_dat:%d mv\r\n",Wy_dat.xjwy4_dat);
//			printf("displacement_dat:%d\r\n",Wy_dat.displacement_dat);
			sdxjwydat[sdxjwynum][0] = xjwy_msg[0] = Wy_dat.xjwy1_dat;
			sdxjwydat[sdxjwynum][1] = xjwy_msg[1] = Wy_dat.xjwy2_dat;
			sdxjwydat[sdxjwynum][2] = xjwy_msg[2] = Wy_dat.xjwy3_dat;
			sdxjwydat[sdxjwynum][3] = xjwy_msg[3] = Wy_dat.xjwy4_dat;
			sdxjwydat[sdxjwynum][4] = xjwy_msg[4] = Wy_dat.displacement_dat;
			sdxjwydat[sdxjwynum][5] = xjwy_msg[5] = Wy_dat.oilyy_dat[0];
			sdxjwydat[sdxjwynum][6] = xjwy_msg[6] = Wy_dat.oilyy_dat[1];
			osEventFlagsSet(Can_EventHandle, CAN_EVENTBIT_6);
			if(dtu_device1.Onenet_Off_flag == 0&&dtu_device1.st_flag == 0){
				OneNet_Send(oildisplay,XJ_Device_ID,sizeof(oildisplay));
			}
			if((sdxjwynum++)>=10){
				sdxjwynum = 0;
				if(osEventFlagsGet (SD_EventHandle)&SD_EVENTBIT_1){
					res_flash = f_open(&SDFile, "0:/FWT_dat/xjwydat.txt",FA_OPEN_APPEND | FA_WRITE);
					if ( res_flash == FR_OK ){
//						printf("open FWT_dat/xjwydat.txt success!!\r\n");
						memset(sd_xjwy, 0, sizeof(sd_xjwy));
						sprintf(sd_xjwy,"\r\n%04d/%02d/%02d %02d:%02d:%02d",gpsx.utc.year,gpsx.utc.month,gpsx.utc.date,gpsx.utc.hour,gpsx.utc.min,gpsx.utc.sec);
						f_write(&SDFile,sd_xjwy,sizeof(sd_xjwy),&xjwy_bw);
						for(uint8_t i = 0;i<10;i++){
							memset(sd_xjwy, 0, sizeof(sd_xjwy));
							sprintf(sd_xjwy,"\r\nxj1 = %d mm ,xj2 = %d mm ,xj3 = %d mm ,xj4 = %d mm ,zd = %2.3lf mpa, ym = %d mm",sdxjwydat[i][1],\
							sdxjwydat[i][2],sdxjwydat[i][3],sdxjwydat[i][4],(sdxjwydat[i][5]*100+sdxjwydat[i][6])*1.0/1000\
							,sdxjwydat[i][0]);
							f_write(&SDFile,sd_xjwy,sizeof(sd_xjwy),&xjwy_bw);						
						}
					}else{
						printf("open FWT_dat/xjwydat.txt fail (%d)!!",res_flash);
					}
					f_close(&SDFile);
				}
			}
			HAL_ADC_Start_DMA(&hadc1,adc1_value,sizeof(adc1_value)/4);
		}
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
		while( timeout <= 10 ){   /* 等待Cat1 DTU启动，需要等待5-6s才能启动 */
				ret = dtu_config_init(DTU_WORKMODE_ONENET);    /*初始化DTU工作参数*/
				if( ret == 0 ){
					osEventFlagsSet(Vcu_Event1Handle, EVENTBIT_3);
					printf("**************************************************************************\r\n");
					printf("Cat1 DTU Init Success \r\n");
					printf("**************************************************************************\r\n\r\n");
					break;
				}
				timeout++;
				osDelay(1000);
		}
		if( timeout > 10 ){   /* 超时 */
			printf("**************************************************************************\r\n");
			printf("ATK-DTU Init Fail ...\r\n");
			printf("**************************************************************************\r\n\r\n");
		}
    if((osEventFlagsGet (Vcu_Event1Handle)&EVENTBIT_3)){
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
	uint8_t j = 0;
	int ret;
  for(;;)
  {
			if(osEventFlagsWait (Vcu_Event1Handle, EVENTBIT_2,osFlagsNoClear, osWaitForever)&EVENTBIT_2){
//				printf("%d\r\n",(osEventFlagsGet(Vcu_Event1Handle)&EVENTBIT_3));
				if(osEventFlagsGet (Vcu_Event1Handle)&EVENTBIT_3){
					dtu_device1.st_flag = 1;
					while(j<5){
						osEventFlagsClear(Vcu_Event1Handle, EVENTBIT_1);
						/*1.DTU进入配置状态*/
						ret = dtu_enter_configmode();
						if ( ret != 0 ){
							printf("DTU enter set mode failed !\r\n");
							vTaskDelay(10);
							j++;
						}
						else{
							printf("DTU enter set mode !\r\n");
							osEventFlagsClear(Vcu_Event1Handle, EVENTBIT_3);
							send_data_to_dtu("AT+CSQ\r\n", strlen("AT+CSQ\r\n"));
							break;
						}
					}
					dtu_device1.st_flag = 0;
					j = 0;
					osEventFlagsSet(Vcu_Event1Handle, EVENTBIT_1);
				}
			}
		osDelay(DTU_Signal_Delay);
  }
  /* USER CODE END Signal_4G_Handle */
}

/* USER CODE BEGIN Header_GPS_Init_Handle */
/**
* @brief Function implementing the GPS_Init thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GPS_Init_Handle */
void GPS_Init_Handle(void *argument)
{
  /* USER CODE BEGIN GPS_Init_Handle */
  /* Infinite loop */
	uint8_t key=0xff;
  for(;;)
  {
		if(Ublox_Cfg_Rate(400,1)!=0)               //设置定位信息更新速度为400ms,顺便判断GPS模块是否在位.
	  {
			printf("WT-GPS_BD Setting...\r\n");
			while((Ublox_Cfg_Rate(400,1)!=0)&&key){	//持续判断,直到可以检查到WT-GPS_BD,且数据保存成功
				Ublox_Cfg_Tp(500000,100000,1);	//设置PPS为1秒钟输出1次,脉冲宽度为100ms
				Ublox_Cfg_Prt(115200);
				key=Ublox_Cfg_Cfg_Save();		//保存配置
			}
			printf("WT-GPS_BD Set Done!!\r\n");
			osDelay(1);
		}
		osEventFlagsSet(Vcu_Event1Handle, EVENTBIT_8);
    vTaskDelete(GPS_Init_TaskHandle);
  }
  /* USER CODE END GPS_Init_Handle */
}

/* USER CODE BEGIN Header_GPS_Get_Handle */
/**
* @brief Function implementing the GPS_Get_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GPS_Get_Handle */
void GPS_Get_Handle(void *argument)
{
  /* USER CODE BEGIN GPS_Get_Handle */
  /* Infinite loop */
  for(;;)
  {
		if(osEventFlagsGet (Vcu_Event1Handle)&EVENTBIT_8){
			if(USART3_RX_STA&0X8000){		//接收到一次数据了
				USART3_RX_STA=0;		   	//启动下一次接收
				GPS_Analysis(&gpsx,(uint8_t*)USART3_RX_BUF);//分析字符串
				Gps_Msg_Show();				//显示信息
			}
		}
		osDelay(GPS_Handle_Delay);
  }
  /* USER CODE END GPS_Get_Handle */
}

/* USER CODE BEGIN Header_IWDG_Handle */
/**
* @brief Function implementing the IWDG_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IWDG_Handle */
void IWDG_Handle(void *argument)
{
  /* USER CODE BEGIN IWDG_Handle */
  /* Infinite loop */
  for(;;)
  {
		__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
    osDelay(100);
  }
  /* USER CODE END IWDG_Handle */
}

/* USER CODE BEGIN Header_Oadc_Handle */
/**
* @brief Function implementing the Otheradc_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Oadc_Handle */
void Oadc_Handle(void *argument)
{
  /* USER CODE BEGIN Oadc_Handle */
  /* Infinite loop */
  uint32_t battery_capacity = 0,oil_capacity = 0;
	uint32_t count_som = 0;
	uint8_t Oil_count[110]= {0};
	uint16_t Oil_dat = 0;                         //ADC油量采集原始数据
	uint16_t High[8] = {0};                       //油量高度数组
	uint8_t Oil = 0;							 							  //油量
	static uint8_t memory_oil = 0;
	static uint8_t High_Count = 0;
	static uint16_t Oil_i = 0,df = 0;
	static uint8_t first_flag = 1;
	static uint8_t af = 0,bf = 0,cf = 0;
  for(;;)
  {
		if(ADC2_Flag == 1)
		{
			ADC2_Flag = 0;
			oil_capacity = 0;battery_capacity = 0;
			for(uint8_t i=0;i<20;){
				oil_capacity += adc2_value[i++];
				battery_capacity += adc2_value[i++];
		  }
			printf("battery_capacity = %d\r\noil_capacity = %d\r\n",battery_capacity*330/4096,oil_capacity*330/4096);
			if(first_flag == 1)             //采样首次油量数据
			{
				oil_first:
				/* 获取各通道ADC采集值 */
				Oil_dat = oil_capacity/10;
				/* 油量高度与检测电压的关系 High(mm) = Volt(mv) / 5 */
				if(Oil_dat > 100 && Oil_dat < 1400)//&& Oil_dat
				{
					printf("adc = %d\r\n",Oil_dat); 
					High[High_Count] = (Oil_dat * 3300*2.1) / 4095 / 5;
					High_Count++;
					if(High_Count >= 100)
					{
						for(uint8_t i = 0; i < High_Count-1; i++){
							for(uint8_t j = i+1; j < High_Count; j++){
								if(High[j] < High[i]){
									uint16_t temp = High[i];
									High[i] = High[j];
									High[j] = temp;
								}
							}
						}
//						printf("High[High_Count / 2] = %d\r\n",High[High_Count / 2]);
						Oil = (High[High_Count / 2] * 100) / TANK_HEIGHT;           //去中位数
//						printf("Oil = %d",Oil);
						High_Count = 0;
//						printf("Oil = %d",Oil);
						if(Oil <= 99 && Oil >=30){
							Oil_base_dat = Oil;
							first_flag = 0;
							memory_oil = Oil_base_dat;
							__HAL_TIM_ENABLE(&htim13);                                     // 使能计数器	
						}
						else if(Oil < 30){
							if(af <= 20){
								af++;
								goto oil_first;
							}
							af = 0;
							Oil_base_dat = 0;
							first_flag = 2;
						}
						else if(Oil > 99){
							Oil_base_dat = 99;
							first_flag = 0;
							memory_oil = Oil_base_dat;
							__HAL_TIM_ENABLE(&htim13);                                     // 使能计数器	
						}										
					}
				}
				else if(af > 20){
					first_flag = 2;
					Oil_base_dat = 0;
					af = 0;
				}
				else{
					af++;
				}
			}
			else if(first_flag == 0)                                             //定时器校验油量数据
			{
					/* 获取各通道ADC采集值 */
				Oil_dat = oil_capacity/10;
				/* 油量高度与检测电压的关系 High(mm) = Volt(mv) / 5 */
				if(Oil_dat > 350)    /* 没测到液体时小于50mv，所以去350 */
				{
					High[High_Count] = (Oil_dat * 3300*2.1) / 4095 / 5;
					High_Count++;
					if(High_Count >= 5)
					{
						for(uint8_t i = 0; i < High_Count-1; i++){
							for(int8_t j = i+1; j < High_Count; j++){
								if(High[j] < High[i]){
									int16_t temp = High[i];
									High[i] = High[j];
									High[j] = temp;
								}
							}
						}
						Oil = (High[High_Count / 2] * 100) / TANK_HEIGHT;
						High_Count = 0;
					}
				}
				if(Oil < memory_oil+10&&Oil > memory_oil-10)
				{
					Oil_count[Oil_i++] = Oil;
				}
				if(Oil_i >= 50)
				{
					count_som = 0;
					for(uint8_t i = 0; i < 50; i++){
						count_som += Oil_count[i];
					}
					if(count_som/50 >= memory_oil){bf++;}
					else if(count_som/50 < memory_oil){df += count_som/50,cf++;}
					if(bf >= 5)                                      //测量值大于Oil_base_dat
					{
						 __HAL_TIM_DISABLE(&htim13);																									// 失能计数器
						 bf = 0;cf = 0;
					}
					else if(cf >= 5)
					{
						memory_oil = df/5;
						__HAL_TIM_SET_COUNTER(&htim13,0);
						__HAL_TIM_ENABLE(&htim13);																									// 使能计数器
						cf = 0;bf = 0;
					}
					Oil_i = 0;
				}
//				printf("volt : %d",(Oil_dat * 3300) / 4095);
//				printf(" oil：%d\r\n ",Oil);
			}	
			battery_capacity = battery_capacity*330/4096;                    //电池电量采样
			if(battery_capacity >3235){
				oadc_msg[0] = 6;
			}
			else if( battery_capacity > 3160&&battery_capacity < 3235){
				oadc_msg[0] = 5;
			}
			else if( battery_capacity > 3080&&battery_capacity < 3160){
				oadc_msg[0] = 4;
			}
			else if( battery_capacity > 2990&&battery_capacity < 3080){
				oadc_msg[0] = 3;
			}
			else if( battery_capacity > 2910&&battery_capacity < 2990){
				oadc_msg[0] = 2;
			}
			else if( battery_capacity > 2840&&battery_capacity < 2910){
				oadc_msg[0] = 1;
			}
			else if(battery_capacity < 2810){
				oadc_msg[0] = 0;
			}
//			printf("battery_capacity = %d\r\n",oadc_msg[0]);
			oadc_msg[1] = Oil_base_dat;
			osEventFlagsSet(Can_EventHandle, CAN_EVENTBIT_7);
//			if((osEventFlagsGet (Vcu_Event1Handle)&EVENTBIT_6) == EVENTBIT_6&&dtu_device1.Onenet_Off_flag == 0&&){
//			OneNet_Send(oildisplay,XJ1_Device_ID,sizeof(oildisplay));
//			}
			HAL_ADC_Start_DMA(&hadc2,adc2_value,sizeof(adc2_value)/4);
		}
		if(first_flag == 1){
			osDelay(10);
		}
		else{
			osDelay(ADC_Handle_Delay);
		}
  }
  /* USER CODE END Oadc_Handle */
}

/* USER CODE BEGIN Header_Cantx_Handle */
/**
* @brief Function implementing the CantxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Cantx_Handle */
void Cantx_Handle(void *argument)
{
  /* USER CODE BEGIN Cantx_Handle */
  /* Infinite loop */
	uint32_t can_tx_mailbox;
	CAN_TxHeaderTypeDef TxMessage;
	TxMessage.ExtId=0;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.TransmitGlobalTime=DISABLE;
  for(;;)
  {
		#define candelay 5
		//九轴数据
		if(osEventFlagsGet (Can_EventHandle)&CAN_EVENTBIT_1){
			osEventFlagsClear (Can_EventHandle, CAN_EVENTBIT_1);
			TxMessage.StdId=Wit_StdId_xy;
			TxMessage.DLC=sizeof(wit_msg[0]);
			while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) < 1);
			if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,wit_msg[0],&can_tx_mailbox) != HAL_OK){
				printf ("数据发送失败！\r\n");
			}
			osDelay(candelay);
			TxMessage.StdId=Wit_StdId_xy+1;
			TxMessage.DLC=sizeof(wit_msg[1]);
			while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) < 1);
			if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,wit_msg[1],&can_tx_mailbox) != HAL_OK){
				printf ("数据发送失败！\r\n");
			}
			osDelay(candelay);
			TxMessage.StdId=Wit_StdId_xy+2;
			TxMessage.DLC=sizeof(wit_msg[2]);
			while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) < 1);
			if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,wit_msg[2],&can_tx_mailbox) != HAL_OK){
				printf ("数据发送失败！\r\n");
			}
			osDelay(candelay);
		}
		//轮速速度数据
		if(osEventFlagsGet (Can_EventHandle)&CAN_EVENTBIT_2){
			osEventFlagsClear (Can_EventHandle, CAN_EVENTBIT_2);
			TxMessage.StdId=Tyre_StdId;
			TxMessage.DLC=sizeof(tyre_msg);
			while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) < 1);
			if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,tyre_msg,&can_tx_mailbox) != HAL_OK){
				printf ("数据发送失败！\r\n");
			}
			osDelay(candelay);	
		}
		//gps速度数据
		if(osEventFlagsGet (Can_EventHandle)&CAN_EVENTBIT_4){
			osEventFlagsClear (Can_EventHandle, CAN_EVENTBIT_4);
			TxMessage.StdId=GPS_StdId;
			TxMessage.DLC=sizeof(speedfloat_tx.data);
			while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) < 1);
			if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,speedfloat_tx.data,&can_tx_mailbox) != HAL_OK){
				printf ("数据发送失败！\r\n");
			}
			osDelay(candelay);			
		}
		//GPS实时时间数据
		if(osEventFlagsGet (Can_EventHandle)&CAN_EVENTBIT_5){
			osEventFlagsClear (Can_EventHandle, CAN_EVENTBIT_5);
			TxMessage.StdId=CLK_StdId;
			TxMessage.DLC=sizeof(Clkvalue);
			while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) < 1);
			if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,Clkvalue,&can_tx_mailbox) != HAL_OK){
				printf ("数据发送失败！\r\n");
			}
			osDelay(candelay);
		}
		//悬架位移数据
		if(osEventFlagsGet (Can_EventHandle)&CAN_EVENTBIT_6){
			osEventFlagsClear (Can_EventHandle, CAN_EVENTBIT_6);
			TxMessage.StdId=XJWY_StdId;
			TxMessage.DLC=sizeof(xjwy_msg);
			while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) < 1);
			if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,xjwy_msg,&can_tx_mailbox) != HAL_OK){
				printf ("数据发送失败！\r\n");
			}		
			osDelay(candelay);			
		}
		//电池电量，油量
		if(osEventFlagsGet (Can_EventHandle)&CAN_EVENTBIT_7){
			osEventFlagsClear (Can_EventHandle, CAN_EVENTBIT_7);
			TxMessage.StdId=OADC_StdId;
			TxMessage.DLC=sizeof(oadc_msg);
			while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) < 1);
			if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,oadc_msg,&can_tx_mailbox) != HAL_OK){
				printf ("数据发送失败！\r\n");
			}
			osDelay(candelay);			
		}
		//4G信号
		if(osEventFlagsGet (Can_EventHandle)&CAN_EVENTBIT_8){
			osEventFlagsClear (Can_EventHandle, CAN_EVENTBIT_8);
			TxMessage.StdId=DTU_StdId;
			TxMessage.DLC=sizeof(dtu_device1.Network_size);
//			printf("dtu_device1.Network_size = %d\r\n",dtu_device1.Network_size);
			while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) < 1);
			if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,&dtu_device1.Network_size,&can_tx_mailbox) != HAL_OK){
				printf ("数据发送失败！\r\n");
			}
			osDelay(candelay);	
		}
		osDelay(1);
	}
  /* USER CODE END Cantx_Handle */
}

/* USER CODE BEGIN Header_TyreSpeed_Handle */
/**
* @brief Function implementing the TyreSpeedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TyreSpeed_Handle */
void TyreSpeed_Handle(void *argument)
{
  /* USER CODE BEGIN TyreSpeed_Handle */
  /* Infinite loop */
	uint8_t tyrespeed[64] = {0};
	uint16_t sdtyrespeed[5][11] = {0};
	static uint8_t sdnum[5] = {0};
	char sd_tyre[64] = {0};
	UINT tyre1_bw,tyre2_bw,tyre3_bw,tyre4_bw,moderator_bw;
	FRESULT res_flash;
  for(;;)
  {
		/* 左前轮 */
		if(osEventFlagsGet (MT_EventHandle)&MT_EVENTBIT_1){
			osEventFlagsClear (MT_EventHandle, MT_EVENTBIT_1);
			if (mtspeed1.Endup_Flag != 0)                         //轮速信号捕获完成
			{
				if(mtspeed1.Total_Time_M1 == 0){
					mtspeed1.Rotate_Speed = 0;
					mtspeed1.Total_Time_M2 = 0;
					mtspeed1.Endup_Flag = 0;
				}
				else{
					mtspeed1.Endup_Flag = 0;
					mtspeed1.Rotate_Speed = (5000.0*60*mtspeed1.Total_Time_M1)/(F_TOOTH_NUM*mtspeed1.Total_Time_M2);    //计算转速(r/min)   M/T算法公式
					sdtyrespeed[0][sdnum[0]++] = mtspeed1.Rotate_Speed;
					if(sdnum[0]>=10){
						sdnum[0] = 0;
						if(osEventFlagsGet (SD_EventHandle)&SD_EVENTBIT_3){
							res_flash = f_open(&SDFile, "0:/FWT_dat/tyrespeed/tyrespeed1.txt",FA_OPEN_APPEND | FA_WRITE);
							if ( res_flash == FR_OK ){
//								printf("FWT_dat/tyrespeed/tyrespeed1.txt open success!");
								memset(sd_tyre, 0, sizeof(sd_tyre));
								sprintf(sd_tyre,"\r\n%04d/%02d/%02d %02d:%02d:%02d",gpsx.utc.year,gpsx.utc.month,gpsx.utc.date,gpsx.utc.hour,gpsx.utc.min,gpsx.utc.sec);
								f_write(&SDFile,sd_tyre,sizeof(sd_tyre),&tyre1_bw);
								memset(sd_tyre, 0, sizeof(sd_tyre));
								sprintf(sd_tyre,"\r\ntyre1 = %d r/min , %d r/min , %d r/min , %d r/min , %d r/min",sdtyrespeed[0][0],sdtyrespeed[0][1],sdtyrespeed[0][2],sdtyrespeed[0][3],sdtyrespeed[0][4]);
								f_write(&SDFile,sd_tyre,sizeof(sd_tyre),&tyre1_bw);
								memset(sd_tyre, 0, sizeof(sd_tyre));
								sprintf(sd_tyre,"\r\ntyre1 = %d r/min , %d r/min , %d r/min , %d r/min , %d r/min",sdtyrespeed[0][5],sdtyrespeed[0][6],sdtyrespeed[0][7],sdtyrespeed[0][8],sdtyrespeed[0][9]);
								f_write(&SDFile,sd_tyre,sizeof(sd_tyre),&tyre1_bw);
							}
							else{
								printf("FWT_dat/tyrespeed/tyrespeed1.txt open fail!(%d)",res_flash);
							}
							f_close(&SDFile);
						}
					}
//          printf(" SPEED1 = %d,M1 = %d,M2 =  %d\r\n ",mtspeed1.Rotate_Speed,mtspeed1.Total_Time_M1,mtspeed1.Total_Time_M2);    					
					mtspeed1.Total_Time_M1 = 0;
					mtspeed1.Total_Time_M2 = 0;
				}
				mtspeed1.startcount = 0;
				HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
			}
		}
		/* 右后轮 */
		if(osEventFlagsGet (MT_EventHandle)&MT_EVENTBIT_2){
			osEventFlagsClear (MT_EventHandle, MT_EVENTBIT_2);
			if (mtspeed2.Endup_Flag != 0)                         //轮速信号捕获完成
			{
				if(mtspeed2.Total_Time_M1 == 0){
					mtspeed2.Rotate_Speed = 0;
					mtspeed2.Total_Time_M2 = 0;
					mtspeed2.Endup_Flag = 0;
				}
				else{
					mtspeed2.Endup_Flag = 0;
					mtspeed2.Rotate_Speed = (5000.0*60*mtspeed2.Total_Time_M1)/(Q_TOOTH_NUM*mtspeed2.Total_Time_M2);    //计算转速(r/min)  M/T算法公式
//  				printf(" SPEED2 = %d,M1 = %d,M2 =  %d\r\n ",mtspeed2.Rotate_Speed,mtspeed2.Total_Time_M1,mtspeed2.Total_Time_M2);
					sdtyrespeed[1][sdnum[1]++] = mtspeed2.Rotate_Speed;
					if(sdnum[1]>=10){
						sdnum[1] = 0;
						if(osEventFlagsGet (SD_EventHandle)&SD_EVENTBIT_3){
							res_flash = f_open(&SDFile, "0:/FWT_dat/tyrespeed/tyrespeed2.txt",FA_OPEN_APPEND | FA_WRITE);
							if ( res_flash == FR_OK ){
//								printf("FWT_dat/tyrespeed/tyrespeed2.txt open success!");
								memset(sd_tyre, 0, sizeof(sd_tyre));
								sprintf(sd_tyre,"\r\n%04d/%02d/%02d %02d:%02d:%02d",gpsx.utc.year,gpsx.utc.month,gpsx.utc.date,gpsx.utc.hour,gpsx.utc.min,gpsx.utc.sec);
								f_write(&SDFile,sd_tyre,sizeof(sd_tyre),&tyre2_bw);
								memset(sd_tyre, 0, sizeof(sd_tyre));
								sprintf(sd_tyre,"\r\ntyre2 = %d r/min , %d r/min , %d r/min , %d r/min , %d r/min",sdtyrespeed[1][0],sdtyrespeed[1][1],sdtyrespeed[1][2],sdtyrespeed[1][3],sdtyrespeed[1][4]);
								f_write(&SDFile,sd_tyre,sizeof(sd_tyre),&tyre2_bw);
								memset(sd_tyre, 0, sizeof(sd_tyre));
								sprintf(sd_tyre,"\r\ntyre2 = %d r/min , %d r/min , %d r/min , %d r/min , %d r/min",sdtyrespeed[1][5],sdtyrespeed[1][6],sdtyrespeed[1][7],sdtyrespeed[1][8],sdtyrespeed[1][9]);
								f_write(&SDFile,sd_tyre,sizeof(sd_tyre),&tyre2_bw);
							}
							else
							{
								printf("FWT_dat/tyrespeed/tyrespeed2.txt open fail!(%d)",res_flash);
							}
							f_close(&SDFile);
						}
					}
					mtspeed2.Total_Time_M1 = 0;
					mtspeed2.Total_Time_M2 = 0;
				}
				mtspeed2.startcount = 0;
				HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
			}
		}
		/* 右前轮 */
		if(osEventFlagsGet (MT_EventHandle)&MT_EVENTBIT_3){
			osEventFlagsClear (MT_EventHandle, MT_EVENTBIT_3);
			if (mtspeed3.Endup_Flag != 0)                         //轮速信号捕获完成
			{
				if(mtspeed3.Total_Time_M1 == 0){
					mtspeed3.Rotate_Speed = 0;
					mtspeed3.Total_Time_M2 = 0;
					mtspeed3.Endup_Flag = 0;
				}
				else{
					mtspeed3.Endup_Flag = 0;
					mtspeed3.Rotate_Speed = (5000.0*60*mtspeed3.Total_Time_M1)/(F_TOOTH_NUM*mtspeed3.Total_Time_M2);    //计算转速(r/min)    
					sdtyrespeed[2][sdnum[2]++] = mtspeed3.Rotate_Speed;
					if(sdnum[2]>=10){
						sdnum[2] = 0;
						if(osEventFlagsGet (SD_EventHandle)&SD_EVENTBIT_3){
							res_flash = f_open(&SDFile, "0:/FWT_dat/tyrespeed/tyrespeed3.txt",FA_OPEN_APPEND | FA_WRITE);
							if ( res_flash == FR_OK ){
//								printf("FWT_dat/tyrespeed/tyrespeed3.txt open success!");
								memset(sd_tyre, 0, sizeof(sd_tyre));
								sprintf(sd_tyre,"\r\n%04d/%02d/%02d %02d:%02d:%02d",gpsx.utc.year,gpsx.utc.month,gpsx.utc.date,gpsx.utc.hour,gpsx.utc.min,gpsx.utc.sec);
								f_write(&SDFile,sd_tyre,sizeof(sd_tyre),&tyre3_bw);
								memset(sd_tyre, 0, sizeof(sd_tyre));
								sprintf(sd_tyre,"\r\ntyre3 = %d r/min , %d r/min , %d r/min , %d r/min , %d r/min",sdtyrespeed[2][0],sdtyrespeed[2][1],sdtyrespeed[2][2],sdtyrespeed[2][3],sdtyrespeed[2][4]);
								f_write(&SDFile,sd_tyre,sizeof(sd_tyre),&tyre3_bw);
								memset(sd_tyre, 0, sizeof(sd_tyre));
								sprintf(sd_tyre,"\r\ntyre3 = %d r/min , %d r/min , %d r/min , %d r/min , %d r/min",sdtyrespeed[2][5],sdtyrespeed[2][6],sdtyrespeed[2][7],sdtyrespeed[2][8],sdtyrespeed[2][9]);
								f_write(&SDFile,sd_tyre,sizeof(sd_tyre),&tyre3_bw);
							}
							else
							{
								printf("FWT_dat/tyrespeed/tyrespeed3.txt open fail!(%d)",res_flash);
							}
							f_close(&SDFile);
						}
					}
//					printf(" SPEED3 = %d,M1 = %d,M2 =  %d\r\n ",mtspeed3.Rotate_Speed,mtspeed3.Total_Time_M1,mtspeed3.Total_Time_M2);
					mtspeed3.Total_Time_M1 = 0;
					mtspeed3.Total_Time_M2 = 0;
				}
				mtspeed3.startcount = 0;
				HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_1);
//				printf("TIM_CHANNEL_2\r\n");
			}
		}
		/* 左后轮 */
		if(osEventFlagsGet (MT_EventHandle)&MT_EVENTBIT_4){
			osEventFlagsClear (MT_EventHandle, MT_EVENTBIT_4);
			if (mtspeed4.Endup_Flag != 0)                         //轮速信号捕获完成
			{
				if(mtspeed4.Total_Time_M1 == 0){
					mtspeed4.Rotate_Speed = 0;
					mtspeed4.Total_Time_M2 = 0;
					mtspeed4.Endup_Flag = 0;
				}
				else{
					mtspeed4.Endup_Flag = 0;
					if(time3_over1 == 0){
					mtspeed4.Rotate_Speed = (5000.0*60*mtspeed4.Total_Time_M1)/(Q_TOOTH_NUM*mtspeed4.Total_Time_M2);    //计算转速(r/min)
//					printf(" SPEED4 = %d,M1 = %d,M2 =  %d\r\n ",mtspeed4.Rotate_Speed,mtspeed4.Total_Time_M1,mtspeed4.Total_Time_M2);
					}else{
						time3_over1 = 0;
						mtspeed4.memory_value = 0;
					}
					sdtyrespeed[3][sdnum[3]++] = mtspeed4.Rotate_Speed;
					if(sdnum[3]>=10){
						sdnum[3] = 0;
						if(osEventFlagsGet (SD_EventHandle)&SD_EVENTBIT_3){
							res_flash = f_open(&SDFile, "0:/FWT_dat/tyrespeed/tyrespeed4.txt",FA_OPEN_APPEND | FA_WRITE);
							if ( res_flash == FR_OK ){
//								printf("FWT_dat/tyrespeed/tyrespeed4.txt open success!");
								memset(sd_tyre, 0, sizeof(sd_tyre));
								sprintf(sd_tyre,"\r\n%04d/%02d/%02d %02d:%02d:%02d",gpsx.utc.year,gpsx.utc.month,gpsx.utc.date,gpsx.utc.hour,gpsx.utc.min,gpsx.utc.sec);
								f_write(&SDFile,sd_tyre,sizeof(sd_tyre),&tyre4_bw);
								memset(sd_tyre, 0, sizeof(sd_tyre));
								sprintf(sd_tyre,"\r\ntyre4 = %d r/min , %d r/min , %d r/min , %d r/min , %d r/min",sdtyrespeed[3][0],sdtyrespeed[3][1],sdtyrespeed[3][2],sdtyrespeed[3][3],sdtyrespeed[3][4]);
								f_write(&SDFile,sd_tyre,sizeof(sd_tyre),&tyre4_bw);
								memset(sd_tyre, 0, sizeof(sd_tyre));
								sprintf(sd_tyre,"\r\ntyre4 = %d r/min , %d r/min , %d r/min , %d r/min , %d r/min",sdtyrespeed[3][5],sdtyrespeed[3][6],sdtyrespeed[3][7],sdtyrespeed[3][8],sdtyrespeed[3][9]);
								f_write(&SDFile,sd_tyre,sizeof(sd_tyre),&tyre4_bw);
							}
							else
							{
								printf("FWT_dat/tyrespeed/tyrespeed4.txt open fail!(%d)",res_flash);
							}
							f_close(&SDFile);
						}
					}
//					printf(" SPEED4 = %d,M1 = %d,M2 =  %d\r\n ",mtspeed4.Rotate_Speed,mtspeed4.Total_Time_M1,mtspeed4.Total_Time_M2);
					mtspeed4.Total_Time_M1 = 0;
					mtspeed4.Total_Time_M2 = 0;
				}
				mtspeed4.startcount = 0;
				HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
			}
		}
		/* 减速器机械测速 */
		if(osEventFlagsGet (MT_EventHandle)&MT_EVENTBIT_5){
			osEventFlagsClear (MT_EventHandle, MT_EVENTBIT_5);
			if (mtspeed5.Endup_Flag != 0)                         //轮速信号捕获完成
			{
				if(mtspeed5.Total_Time_M1 == 0){
					mtspeed5.Rotate_Speed = 0;
					mtspeed5.Total_Time_M2 = 0;
					mtspeed5.Endup_Flag = 0;
				}
				else{
					mtspeed5.Endup_Flag = 0;
					if(time3_over2 == 0){
						mtspeed5.Rotate_Speed = (((5000.0*60*mtspeed5.Total_Time_M1)/(J_TOOTH_NUM*mtspeed5.Total_Time_M2)) * 2 * PI * WHEEL_RADIUS /GEAR_RATIO)*3.0/50;    //计算速度(km/h)	
						printf(" SPEED5 = %d,M1 = %d,M2 =  %d\r\n",mtspeed5.Rotate_Speed,mtspeed5.Total_Time_M1,mtspeed5.Total_Time_M2);
					}
					else{
						time3_over2 = 0;
						mtspeed5.memory_value = 0;
					}
					sdtyrespeed[4][sdnum[4]++] = mtspeed5.Rotate_Speed;
					if(sdnum[4]>=10){
						sdnum[4] = 0;
						if(osEventFlagsGet (SD_EventHandle)&SD_EVENTBIT_3){
							res_flash = f_open(&SDFile, "0:/FWT_dat/tyrespeed/reducerspeed.txt",FA_OPEN_APPEND | FA_WRITE);
							if ( res_flash == FR_OK ){
//								printf("FWT_dat/tyrespeed/tyrespeed4.txt open success!");
								memset(sd_tyre, 0, sizeof(sd_tyre));
								sprintf(sd_tyre,"\r\n%04d/%02d/%02d %02d:%02d:%02d",gpsx.utc.year,gpsx.utc.month,gpsx.utc.date,gpsx.utc.hour,gpsx.utc.min,gpsx.utc.sec);
								f_write(&SDFile,sd_tyre,sizeof(sd_tyre),&moderator_bw);
								memset(sd_tyre, 0, sizeof(sd_tyre));
								sprintf(sd_tyre,"\r\nmoderator = %d km/h , %d km/h , %d km/h , %d km/h , %d km/h",sdtyrespeed[4][0],sdtyrespeed[4][1],sdtyrespeed[4][2],sdtyrespeed[4][3],sdtyrespeed[4][4]);
								f_write(&SDFile,sd_tyre,sizeof(sd_tyre),&moderator_bw);
								memset(sd_tyre, 0, sizeof(sd_tyre));
								sprintf(sd_tyre,"\r\nmoderator = %d km/h , %d km/h , %d km/h , %d km/h , %d km/h",sdtyrespeed[4][5],sdtyrespeed[4][6],sdtyrespeed[4][7],sdtyrespeed[4][8],sdtyrespeed[4][9]);
								f_write(&SDFile,sd_tyre,sizeof(sd_tyre),&moderator_bw);
							}
							else
							{
								printf("FWT_dat/tyrespeed/tyrespeed4.txt open fail!(%d)",res_flash);
							}
							f_close(&SDFile);
						}
					}
//					printf(" SPEED4 = %d,M1 = %d,M2 =  %d\r\n ",mtspeed4.Rotate_Speed,mtspeed4.Total_Time_M1,mtspeed4.Total_Time_M2);
					mtspeed5.Total_Time_M1 = 0;
					mtspeed5.Total_Time_M2 = 0;
				}
				mtspeed5.startcount = 0;
				HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
			}
		}
		//左前
		tyre_msg[0] = (mtspeed1.Rotate_Speed >> 8);
		tyre_msg[1] = mtspeed1.Rotate_Speed &0xff;
		//左后
		tyre_msg[2] = (mtspeed3.Rotate_Speed >> 8);
		tyre_msg[3] = mtspeed3.Rotate_Speed &0xff;
		//右前
		tyre_msg[4] = (mtspeed4.Rotate_Speed >> 8);
		tyre_msg[5] = mtspeed4.Rotate_Speed &0xff;
		//右后
		tyre_msg[6] = (mtspeed2.Rotate_Speed >> 8);
		tyre_msg[7] = mtspeed2.Rotate_Speed &0xff;
		osEventFlagsSet (Can_EventHandle,CAN_EVENTBIT_2);
		if(dtu_device1.Onenet_Off_flag == 0&&dtu_device1.st_flag == 0){
//			printf("EVENTBIT_10\r\n");
			OneNet_Send(tyrespeed,TYRE_Device_ID,sizeof(tyrespeed));
		}
		osDelay(Type_Dat_Handle_Delay);
	}
  /* USER CODE END TyreSpeed_Handle */
}

/* USER CODE BEGIN Header_SD_Init_Handle */
/**
* @brief Function implementing the SD_Init_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SD_Init_Handle */
void SD_Init_Handle(void *argument)
{
  /* USER CODE BEGIN SD_Init_Handle */
  /* Infinite loop */
  for(;;)
  {
		FRESULT res_flash;
		//挂载文件系统
		res_flash = f_mount(&SDFatFS,"0:",1);
		if(res_flash!=FR_OK)
		{
			printf("！！mount error！！(%d)\r\n",res_flash);
			osEventFlagsClear (SD_EventHandle,SD_EVENTBIT_1|SD_EVENTBIT_2|SD_EVENTBIT_3|SD_EVENTBIT_4);
		}
		else
		{
			printf("mount sucess！！\r\n");
			printf_sdcard_info();
			osEventFlagsSet (SD_EventHandle,SD_EVENTBIT_1|SD_EVENTBIT_2|SD_EVENTBIT_3|SD_EVENTBIT_4);
		}
    vTaskDelete(SD_Init_taskHandle);
  }
  /* USER CODE END SD_Init_Handle */
}

/* USER CODE BEGIN Header_RTC_Handle */
/**
* @brief Function implementing the RTCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RTC_Handle */
void RTC_Handle(void *argument)
{
  /* USER CODE BEGIN RTC_Handle */
  /* Infinite loop */
	RTC_TimeTypeDef rtcTime;
  RTC_DateTypeDef GetData;   //获取日期结构	
  for(;;)
  {
		HAL_RTC_GetTime(&hrtc,&rtcTime,RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc,&GetData,RTC_FORMAT_BIN);
//		printf("      %02d-%02d-%02d\r\n",rtcTime.Hours,rtcTime.Minutes,rtcTime.Seconds);
    osDelay(RTC_Delay);
  }
  /* USER CODE END RTC_Handle */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


/**********************************************************************
  * @ 函数名  ： HAL_CAN_RxFifo0MsgPendingCallback()
  * @ 功能说明： CAN数据接受回调函数
  * @ 参数    ： CAN_HandleTypeDef *hcan
  * @ 返回值  ： 无
  ********************************************************************/
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
		for(i = 0; i < 8; i++){
			canrx_dat[i]=0x00;
		}
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&RxMessage,canrx_dat);		
		switch(RxMessage.StdId)
		{
			case(CLK_StdId2):{
				if((RxMessage.IDE==CAN_ID_STD) && (RxMessage.DLC==1)){
					UpdateTime_flag = canrx_dat[0];
				}
				break;
			}
			case(Wit_dat):{
				if((RxMessage.IDE==CAN_ID_STD) && (RxMessage.DLC==8)){
					WitCanDataIn(canrx_dat, RxMessage.DLC);
					osEventFlagsSet (Vcu_Event1Handle, EVENTBIT_0);					
				}
				break;
			}
			default:
				break;
		}
		osSemaphoreRelease(CanBinarySemHandle);
	}
}
/**********************************************************************
  * @ 函数名  ： OneNet_Send()
	* @ 功能说明： 4g_DTU的数据发送函数
  * @ 参数    ： uint8_t *Send_date,uint8_t device_id,uint8_t dat_len
  * @ 返回值  ： 无
  ********************************************************************/
static void OneNet_Send(uint8_t *Send_date,uint8_t device_id,uint8_t dat_len)
{
	uint8_t Sdat[3] = {3,0,0x46};
	int res = 0;
	uint8_t j = 0;
//	osEventFlagsSet(Vcu_Event1Handle, EVENTBIT_3);
//	printf("%d",osEventFlagsGet (Vcu_Event1Handle)&EVENTBIT_3);
	if((osEventFlagsGet(Vcu_Event1Handle)&EVENTBIT_3) == 0){   //判断是否处于控制阶段 0处于配置状态 1处于透传状态
		dtu_device1.st_flag = 1;
		while(j<5){
					/*DTU进入透传状态*/
			 res = dtu_enter_transfermode();
			 if( res != 0 ){
				 j++;
					printf("DTU enter transparent mode failed !\r\n");
				 osDelay(10);
			 }
			 else{
				 osEventFlagsSet(Vcu_Event1Handle, EVENTBIT_3);	//			 dtu_device1.Dtumode_Switch_flag = 1;
				 printf("DTU enter transparent mode !\r\n");
				 OneNet_FillBuf(Send_date,device_id);
				 HAL_UART_Transmit(&huart2,&Sdat[0],sizeof(Sdat[0]),0xff);
				 HAL_UART_Transmit(&huart2,&Sdat[1],sizeof(Sdat[1]),0xff);
				 HAL_UART_Transmit(&huart2,&Sdat[2],sizeof(Sdat[2]),0xff);
				 HAL_UART_Transmit(&huart2,Send_date,dat_len,0xff);
				 break;
			 }		
		}
		dtu_device1.st_flag = 0;
		j = 0;
	}
	else{
			OneNet_FillBuf(Send_date,device_id);
			HAL_UART_Transmit(&huart2,Send_date,dat_len,0xff);
	 }
}


/**********************************************************************
  * @ 函数名  ： OneNet_FillBuf
  * @ 功能说明： 封装数据
  * @ 参数    ： uint8_t *buff,uint8_t devicer_id
  * @ 返回值  ： 无
  ********************************************************************/
static void OneNet_FillBuf(uint8_t *buff,uint8_t devicer_id)
{
	osKernelLock ();
	switch(devicer_id)
	{
		case(Wit_Device_ID):{
			memset(buff, 0, sizeof(buff));
			sprintf((char *)buff, "{'WIT_ACC_AYRO_ANGLE':'\n%.3f,%.3f,%.3f\n%.3f,%.3f,%.3f\n%.3f,%.3f,%.3f\n'}"\
									,wit_dat1.fAcc[0], wit_dat1.fAcc[1], wit_dat1.fAcc[2]\
			            ,wit_dat1.fGyro[0], wit_dat1.fGyro[1], wit_dat1.fGyro[2]\
			            ,wit_dat1.fAngle[0], wit_dat1.fAngle[1], wit_dat1.fAngle[2]); //WIT_ACC_AYRO_ANGLE是数据流的一个名称，wit_dat1是数据值;
			break;
		}
		case(TYRE_Device_ID):{
			memset(buff, 0, sizeof(buff));
			sprintf((char *)buff, "{'Tyre_speed':'%d,%d,%d,%d'}",mtspeed1.Rotate_Speed,mtspeed2.Rotate_Speed,mtspeed3.Rotate_Speed,mtspeed4.Rotate_Speed);
			break;
		}
		case(XJ_Device_ID):{
			memset(buff, 0, sizeof(buff));
			sprintf((char *)buff, "{'XJWY_Dat':'%d,%d,%d,%d,%d'}",Wy_dat.displacement_dat,Wy_dat.xjwy1_dat,Wy_dat.xjwy2_dat,Wy_dat.xjwy3_dat,Wy_dat.xjwy4_dat);
			break;
		}
		case(GPS_Device_ID):{
			memset(buff, 0, sizeof(buff));
			sprintf((char *)buff, "{'Speed_dat':'%.1fm'}",(float)(gpsx.speed/=1000));
			break;
		}
		case(DTU_Device_ID):{
			memset(buff, 0, sizeof(buff));
			sprintf((char *)buff, "{'4G_signal':'%d'}",dtu_device1.Network_size);
			break;
		}
		default:break;
	}
	osKernelUnlock ();
}

/**********************************************************************
  * @ 函数名  ： HAL_UART_RxCpltCallback()
  * @ 功能说明： 用于处理接受gps模块数据
  * @ 参数    ： UART_HandleTypeDef *huart
  * @ 返回值  ： 无
  ********************************************************************/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3){
		if((USART3_RX_STA&(1<<15))==0)//接收完的一批数据,还没有被处理,则不再接收其他数据
		{
			if(USART3_RX_STA<USART3_MAX_RECV_LEN)	//还可以接收数据
			{
				__HAL_TIM_SetCounter(&htim7,0);
				if(USART3_RX_STA==0) 
				{
					__HAL_TIM_ENABLE(&htim7);
				}
				USART3_RX_BUF[USART3_RX_STA++]=usart_rx_char;	//记录接收到的值	 
			}else 
			{
				USART3_RX_STA|=1<<15;				//强制标记接收完成
			}
		}
		HAL_UART_Receive_IT(&huart3,&usart_rx_char,1);
	}
}

/**********************************************************************
  * @ 函数名  ： Gps_Msg_Show()
  * @ 功能说明： 显示GPS定位信息
  * @ 参数    ： 无
  * @ 返回值  ： 无
  ********************************************************************/
static void Gps_Msg_Show(void)
{
// 	float tp;
	uint8_t Speed_date[64] = {0};
	static uint8_t Sd_gps_num = 0;
	char sd_gpsspeed[200] = {0};
	float sd_gpsspeeddat[10] = {0};
	FRESULT res_flash;
	UINT gps_bw;
//	tp=gpsx.longitude;	
////	printf("Longitude:%.5f %1c\r\n",tp/=100000,gpsx.ewhemi);//得到经度字符串 	   
//	tp=gpsx.latitude;	
////	printf("Latitude:%.5f %1c \r\n",tp/=100000,gpsx.nshemi);//得到纬度字符串
//	tp=gpsx.altitude;	   
////	printf("Altitude:%.1fm \r\n",tp/=10);//得到高度字符串
//	speedfloat_tx.value = gpsx.speed/1000;
	speedfloat_tx.value = mtspeed5.Rotate_Speed;
	printf("Speed:%.3fkm/h \r\n",speedfloat_tx.value);//得到速度字符串
	sd_gpsspeeddat[Sd_gps_num] = speedfloat_tx.value;
//	if(gpsx.fixmode<=3)														//定位状态
//	{  
//		printf("Fix Mode:%s\r\n",fixmode_tbl[gpsx.fixmode]);  
//	}
//	printf("Valid satellite:%02d\r\n",gpsx.posslnum);//用于定位的卫星数
//	printf("Visible satellite:%02d\r\n",gpsx.svnum%100);//可见卫星数		
//	printf("UTC Date:%04d/%02d/%02d   \r\n",gpsx.utc.year,gpsx.utc.month,gpsx.utc.date); //显示UTC日期
//	printf("UTC Time:%02d:%02d:%02d   \r\n",gpsx.utc.hour,gpsx.utc.min,gpsx.utc.sec);//显示UTC时间
	if(UpdateTime_flag != 1&&gpsx.utc.year > 2022){
		if(gpsx.utc.hour > 0&&gpsx.utc.min > 0&&gpsx.utc.sec > 0){
			Clkvalue[0] = gpsx.utc.year - 2000;
			Clkvalue[1] = gpsx.utc.month;
			Clkvalue[2] = gpsx.utc.date;
			Clkvalue[3] = gpsx.utc.hour + 8;
			Clkvalue[4] = gpsx.utc.min;
			Clkvalue[5] = gpsx.utc.sec;
			for(uint8_t k = 0;k<6;k++){
				printf("%d\r\n",Clkvalue[k]);
			}
			osEventFlagsSet(Can_EventHandle, CAN_EVENTBIT_5);
			osDelay(20);
		}
	}
 osEventFlagsSet(Can_EventHandle, CAN_EVENTBIT_4);
	if(dtu_device1.Onenet_Off_flag == 0&&dtu_device1.st_flag == 0){
		OneNet_Send(Speed_date,GPS_Device_ID,sizeof(Speed_date));
	}
	if(Sd_gps_num++>=10){
		Sd_gps_num = 0;
		if(osEventFlagsGet (SD_EventHandle)&SD_EVENTBIT_4){
			res_flash = f_open(&SDFile, "0:/FWT_dat/gpsspeed.txt",FA_OPEN_APPEND | FA_WRITE);
			if ( res_flash == FR_OK ){
				memset(sd_gpsspeed, 0, sizeof(sd_gpsspeed));
				sprintf(sd_gpsspeed,"\r\n%04d/%02d/%02d %02d:%02d:%02d",gpsx.utc.year,gpsx.utc.month,gpsx.utc.date,gpsx.utc.hour,gpsx.utc.min,gpsx.utc.sec);
				f_write(&SDFile,sd_gpsspeed,sizeof(sd_gpsspeed),&gps_bw);
				memset(sd_gpsspeed, 0, sizeof(sd_gpsspeed));
				sprintf(sd_gpsspeed,"\r\ngps_speed = %.3f km/h , %.3f km/h , %.3f km/h , %.3f km/h , %.3f km/h\r\ngps_speed = %.3f km/h , %.3f km/h , %.3f km/h , %.3f km/h , %.3f km/h"\
							,sd_gpsspeeddat[0],sd_gpsspeeddat[1],sd_gpsspeeddat[2],sd_gpsspeeddat[3],sd_gpsspeeddat[4],sd_gpsspeeddat[5],sd_gpsspeeddat[6],sd_gpsspeeddat[7],sd_gpsspeeddat[8]\
							,sd_gpsspeeddat[9]);
				f_write(&SDFile,sd_gpsspeed,sizeof(sd_gpsspeed),&gps_bw);
			}
			else
			{
				printf("FWT_dat/tyrespeed/tyrespeed4.txt open fail!(%d)",res_flash);
			}
			f_close(&SDFile);
		}
	}
}

/**********************************************************************
  * @ 函数名  ： HAL_TIM_IC_CaptureCallback()
  * @ 功能说明： 定时器输入捕获中断返回处理
  * @ 参数    ： TIM_HandleTypeDef *htim
  * @ 返回值  ： 无
  ********************************************************************/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	static uint32_t Pulse_Count1 = 1,Pulse_Count2 = 1,Pulse_Count3 = 1,Pulse_Count4 = 1,Pulse_Count5 = 1;
	if(htim->Instance == TIM2)
	{
		if(mtspeed1.Startup_Flag == 1){
			Pulse_Count1++;
		}
		if(mtspeed2.Startup_Flag == 1){
			Pulse_Count2++;
		}
		//左前
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1) == 1)
		{
			mtspeed1.speed_zero = 0;
			if(mtspeed1.Endup_Flag != 0){                                                       //采样周期结束捕获第二次上升沿
//				printf("Capture2\r\n");
				HAL_TIM_IC_Stop_IT(htim,TIM_CHANNEL_2);																					 //禁止捕获通道1中断
				mtspeed1.Total_Time_M1 = Pulse_Count1;                                 					 //获取采样周期内的M1
				mtspeed1.Total_Time_M2 = __HAL_TIM_GetCounter(htim) - 	mtspeed1.memory_value;		 //获取高频时钟周期M2
				Pulse_Count1 = 1;                                                                //脉冲置1
				mtspeed1.Startup_Flag = 0;                                                       //首次标志清零
				osEventFlagsSet (MT_EventHandle, MT_EVENTBIT_1);
			}else{
//				printf("Capture1\r\n");
				if(mtspeed1.startcount == 0){
					mtspeed1.startcount = 1;
					mtspeed1.Startup_Flag = 1;							                                //标记首次捕获上升沿
				  mtspeed1.memory_value = __HAL_TIM_GetCounter(&htim2);
				  mtspeed1.timecountflag = 1;
				}
			}
		}
		//右后
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5) == 1)
		{
			mtspeed2.speed_zero = 0;
			if(mtspeed2.Endup_Flag != 0){                                                       //采样周期结束捕获第二次上升沿
//				printf("tim2_channel_Capture1\r\n");
				HAL_TIM_IC_Stop_IT(htim,TIM_CHANNEL_1);																					 //禁止捕获通道1中断
				mtspeed2.Total_Time_M1 = Pulse_Count2;                                 					 //获取采样周期内的M1
				mtspeed2.Total_Time_M2 = __HAL_TIM_GetCounter(htim) - 	mtspeed2.memory_value;		 //获取高频时钟周期M2
				Pulse_Count2 = 1;                                                                //脉冲置1
				mtspeed2.Startup_Flag = 0;                                                       //首次标志清零                                                    //实际采样结束标志
				osEventFlagsSet (MT_EventHandle, MT_EVENTBIT_2);
			}else{
//				printf("tim2_channel_Capture2\r\n");
				if(mtspeed2.startcount == 0){
					mtspeed2.startcount = 1;
					mtspeed2.Startup_Flag = 1;							                                //标记首次捕获上升沿
				  mtspeed2.memory_value = __HAL_TIM_GetCounter(&htim2);
				  mtspeed2.timecountflag = 1;
				}				                             					                            //标记首次捕获上升沿
			}
		}
	}
	if(htim->Instance == TIM5)
	{
		if(mtspeed3.Startup_Flag == 1){
			Pulse_Count3++;
		}
		//右前
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == 1)
		{
			mtspeed3.speed_zero = 0;
			if(mtspeed3.Endup_Flag != 0){                                                       //采样周期结束捕获第二次上升沿
				HAL_TIM_IC_Stop_IT(&htim5,TIM_CHANNEL_1);																					 //禁止捕获通道1中断
				mtspeed3.Total_Time_M1 = Pulse_Count3;                                 					 //获取采样周期内的M1
				mtspeed3.Total_Time_M2 = __HAL_TIM_GetCounter(&htim5) - 	mtspeed3.memory_value;		 //获取高频时钟周期M2
				Pulse_Count3 = 1;                                                                //脉冲置1
				mtspeed3.Startup_Flag = 0;                                                       //首次标志清零                                                    //实际采样结束标志
				osEventFlagsSet (MT_EventHandle, MT_EVENTBIT_3);
			}else{
				if(mtspeed3.startcount == 0){
					mtspeed3.startcount = 1;
					mtspeed3.Startup_Flag = 1;							                                //标记首次捕获上升沿
				  mtspeed3.memory_value = __HAL_TIM_GetCounter(&htim5);
				  mtspeed3.timecountflag = 1;
				}				                             					                            //标记首次捕获上升沿
			}
		}		
	}
	if(htim->Instance == TIM3)
	{
		if(mtspeed4.Startup_Flag == 1){
			Pulse_Count4++;
		}
		if(mtspeed5.Startup_Flag == 1){
			Pulse_Count5++;
		}
		//左后
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7) == 1)
		{
			mtspeed4.speed_zero = 0;                                                            
			if(mtspeed4.Endup_Flag != 0){                                                       //采样周期结束捕获第二次上升沿
//				printf("tim3_channel_Capture1\r\n");
				HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_2);																					 //禁止捕获通道1中断
				mtspeed4.Total_Time_M1 = Pulse_Count4;                                 					 //获取采样周期内的M1
				mtspeed4.Total_Time_M2 = __HAL_TIM_GetCounter(&htim3) - mtspeed4.memory_value;		 											//获取高频时钟周期M2
				Pulse_Count4 = 1;                                                                //脉冲置1
				mtspeed4.Startup_Flag = 0;                                                       //首次标志清零                                                     //实际采样结束标志
				osEventFlagsSet (MT_EventHandle, MT_EVENTBIT_4);
			}else{
//				printf("tim3_channel_Capture2\r\n");
				if(mtspeed4.startcount == 0){
					mtspeed4.startcount = 1;
					mtspeed4.Startup_Flag = 1;							                                //标记首次捕获上升沿
					mtspeed4.memory_value = __HAL_TIM_GetCounter(&htim3);
				  mtspeed4.timecountflag = 1;
				}				                             					                            //标记首次捕获上升沿
			}
		}
		//减速器		
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6) == 1)
		{
			mtspeed5.speed_zero = 0;                                                            
			if(mtspeed5.Endup_Flag != 0){                                                       //采样周期结束捕获第二次上升沿
				HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_1);																					//禁止捕获通道1中断
				mtspeed5.Total_Time_M1 = Pulse_Count5;                                 					  //获取采样周期内的M1
				mtspeed5.Total_Time_M2 = __HAL_TIM_GetCounter(&htim3) - mtspeed5.memory_value;	  //获取高频时钟周期M2
				Pulse_Count5 = 1;                                                                 //脉冲置1
				mtspeed5.Startup_Flag = 0;                                                        //首次标志清零                                                     //实际采样结束标志
				osEventFlagsSet (MT_EventHandle, MT_EVENTBIT_5);
			}else{
				if(mtspeed5.startcount == 0){
					mtspeed5.startcount = 1;
					mtspeed5.Startup_Flag = 1;							                                //标记首次捕获上升沿
					mtspeed5.memory_value = __HAL_TIM_GetCounter(&htim3);
				  mtspeed5.timecountflag = 1;
				}				                             					                            //标记首次捕获上升沿
			}
		}
	}
}

/**********************************************************************
  * @ 函数名  ： printf_sdcard_info()
  * @ 功能说明： 读取SD卡基本信息
  * @ 参数    ： 无
  * @ 返回值  ： 无
  ********************************************************************/
inline static void printf_sdcard_info(void)
{
	uint64_t CardCap;      	//SD卡容量
	HAL_SD_CardCIDTypeDef SDCard_CID; 

	HAL_SD_GetCardCID(&hsd,&SDCard_CID);	//获取CID
	HAL_SD_GetCardInfo(&hsd,&SDCardInfo);                    //获取SD卡信息
	CardCap=(uint64_t)(SDCardInfo.LogBlockNbr)*(uint64_t)(SDCardInfo.LogBlockSize);	//计算SD卡容量
	switch(SDCardInfo.CardType)
	{
		case CARD_SDSC:
		{
			if(SDCardInfo.CardVersion == CARD_V1_X)
				printf("Card Type:SDSC V1\r\n");
			else if(SDCardInfo.CardVersion == CARD_V2_X)
				printf("Card Type:SDSC V2\r\n");
		}
		break;
		case CARD_SDHC_SDXC:printf("Card Type:SDHC\r\n");break;
		default:break;
	}	
  printf("Card ManufacturerID: %d \r\n",SDCard_CID.ManufacturerID);				//制造商ID	
 	printf("CardVersion:         %d \r\n",(uint32_t)(SDCardInfo.CardVersion));		//卡版本号
	printf("Class:               %d \r\n",(uint32_t)(SDCardInfo.Class));		    //
 	printf("Card RCA(RelCardAdd):%d \r\n",SDCardInfo.RelCardAdd);					//卡相对地址
	printf("Card BlockNbr:       %d \r\n",SDCardInfo.BlockNbr);						//块数量
 	printf("Card BlockSize:      %d \r\n",SDCardInfo.BlockSize);					//块大小
	printf("LogBlockNbr:         %d \r\n",(uint32_t)(SDCardInfo.LogBlockNbr));		//逻辑块数量
	printf("LogBlockSize:        %d \r\n",(uint32_t)(SDCardInfo.LogBlockSize));		//逻辑块大小
	printf("Card Capacity:       %d MB\r\n",(uint32_t)(CardCap>>20));				//卡容量
}

/* USER CODE END Application */

