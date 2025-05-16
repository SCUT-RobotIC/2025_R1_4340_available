/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "fdcan.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID_MODEL.h"
#include "rtwtypes.h"
#include <math.h>
#include "bsp_fdcan.h"
#include "motorctrl.h"
#include "stdio.h"
#include "math.h"
#include "dm_motor_drv.h"
#include "dm_motor_ctrl.h"
#include "UPPER_LOCATION.h"
#include "stdio.h"
#include "string.h"
#include "sbus.h"
#include "r1_shoot.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern motor_t motor[num];
extern uint8_t CAN_RECEIVE[3];
extern TGT_COOR TC;
extern REAL_COOR RC;
extern double deadband;
extern motor_measure_t *motor_data_can1[8];
extern motor_measure_t *motor_data_can2[8];
extern motor_measure_t *motor_data_can3[8];

extern IWDG_HandleTypeDef hiwdg1;
extern int Vel_Deadband[3];
extern double tor_output[3*8];
extern shoot_t shoot_data;
extern SBUS_CH_Struct SBUS_CH;
extern uint8_t DribblingStatus;
#define CUR      0
#define VEL      1
#define ANG      2

#define PI 3.1415926
#define m6020_2_1_init 5848
#define m6020_2_2_init 3021
#define m6020_2_3_init	3055
uint8_t active_can[10];
uint8_t shoot_flag[10]={0};
uint8_t dri_flag=0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

int theta_test[10];

uint8_t uart_index[2] = {0};
uint8_t buffer[100];
char UART2_TX_BUF[100];
uint8_t UART7_RX_BUF[100];
uint8_t UART2_RX_BUF[100];
uint8_t rx_datatemp[8];
int cnt[10];
int PinState=0;
uint8_t usart2_rec=0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_TIM6_Init();
  MX_FDCAN2_Init();
  MX_USART2_UART_Init();
  MX_FDCAN3_Init();
  MX_USART3_UART_Init();
  MX_FDCAN1_Init();
  MX_UART7_Init();
 // MX_IWDG1_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);

	HAL_UART_Receive_IT(&huart2, buffer+1, 1);
	HAL_UART_Receive_IT(&huart7, buffer, 1);
//  can_filter_init();
//	bsp_can_init();
  fdcan_filter_init(&hfdcan1);
	fdcan_filter_init(&hfdcan2);
  fdcan_filter_init(&hfdcan3);

	PID_MODEL_initialize();
	PID_Speed_Para_Init(1, 1, 10 , 3 , 0.01);
	PID_Speed_Para_Init(1, 2, 10 , 3 , 0.01);
	PID_Speed_Para_Init(1, 3, 10 , 3 , 0.01);
	PID_Speed_Para_Init(1, 4, 10 , 3 , 0.01);
	PID_Speed_Para_Init(1, 5, 70 , 50 , 0.01);

//	
	PID_Speed_Para_Init(2, 1, 20 , 8 , 0.01);
	PID_Speed_Para_Init(2, 2, 20 , 8 , 0.01);
	PID_Speed_Para_Init(2, 3, 20 , 8 , 0.01);

	PID_Speed_Para_Init(2, 5, 70 , 8 , 0.01);
	PID_Speed_Para_Init(2, 6, 70 , 8 , 0.01);
	PID_Speed_Para_Init(2, 7, 70 , 8 , 0.01);
	
	PID_Speed_Para_Init(3, 1, 10 , 3 , 0.01);
	PID_Speed_Para_Init(3, 2, 10 , 3 , 0.01);
	PID_Speed_Para_Init(3, 3, 10 , 3 , 0.01);
	PID_Speed_Para_Init(3, 4, 10 , 3 , 0.01);
	PID_Speed_Para_Init(3, 5, 10 , 3 , 0.01);
	PID_Speed_Para_Init(3, 6, 10 , 3 , 0.01);
	PID_Speed_Para_Init(3, 7, 10 , 3 , 0.01);
	
	PID_Angle_S_Para_Init(1, 1 , 10 , 3 , 0.01);
  PID_Angle_A_Para_Init(1, 1 , 0.3 , 0 , 0);
	
	PID_Angle_S_Para_Init(1, 2 , 10 , 3 , 0.01);
  PID_Angle_A_Para_Init(1, 2 , 0.3 , 0 , 0);
	
	PID_Angle_S_Para_Init(1, 3 , 10 , 3 , 0.01);
  PID_Angle_A_Para_Init(1, 3 , 0.3 , 0 , 0);
	

	
	PID_Angle_S_Para_Init(1, 5 , 4 , 0.5 , 0.1); 
  PID_Angle_A_Para_Init(1, 5 , 0.5 , 0.07 , 0.01); 
  PID_Angle_S_Para_Init(1, 6 , 4 , 0.5 , 0.1);
  PID_Angle_A_Para_Init(1, 6 , 0.5 , 0.07 , 0.01);
	
  PID_Angle_S_Para_Init(1, 7 , 10 , 3 , 0.01);
  PID_Angle_A_Para_Init(1, 7 , 0.3 , 0 , 0);
	
	PID_Angle_S_Para_Init(2, 1 , 10 , 3 , 0.01);
  PID_Angle_A_Para_Init(2, 1 , 0.3 , 0 , 0);
	
	PID_Angle_S_Para_Init(2, 3 , 5 , 3 , 0.01);
  PID_Angle_A_Para_Init(2, 3 , 1.5 , 1 , 0.1);
	
	PID_Angle_S_Para_Init(2, 5 , 70 , 30 , 0.1);
  PID_Angle_A_Para_Init(2, 5 , 0.5 , 0 , 0);
	PID_Angle_S_Para_Init(2, 6 , 70 , 30 , 0.1);
  PID_Angle_A_Para_Init(2, 6 , 0.5 , 0 , 0);
	PID_Angle_S_Para_Init(2, 7 , 70 , 30 , 0.1);
  PID_Angle_A_Para_Init(2, 7 , 0.5 , 0 , 0);
	
	PID_Angle_S_Para_Init(3, 1 , 10 , 3 , 0.01);
  PID_Angle_A_Para_Init(3, 1 , 0.3 , 0 , 0);
	
	PID_Angle_S_Para_Init(3, 5 , 4 , 0.5 , 0.1); //这个是夹爪的2006
  PID_Angle_A_Para_Init(3, 5 , 0.5 , 0.07 , 0.01); 
  PID_Angle_S_Para_Init(3, 6 , 4 , 0.5 , 0.1);//升降�?3508
  PID_Angle_A_Para_Init(3, 6 , 0.5 , 0.07 , 0.01);
												
  PID_Angle_S_Para_Init(3, 7 , 10 , 3 , 0.01);//上弹的时候转的那�?3508
  PID_Angle_A_Para_Init(3, 7 , 0.3 , 0 , 0);
	
	Set_6020_Mode( 0 );
	set_mode( VEL, VEL, ANG, VEL, VEL, ANG, ANG,
            VEL, VEL, VEL, VEL, ANG, ANG, ANG,
					  ANG, VEL, VEL, VEL, ANG, ANG, ANG ); 
	dm_motor_init();
	for(int i=0;i<10;i++){
		dm_motor_enable(&hfdcan1,&motor[Motor1]);
		dm_motor_enable(&hfdcan1,&motor[Motor2]);
		dm_motor_enable(&hfdcan1,&motor[Motor3]);
		HAL_Delay(10);
	}

	memset(UART2_TX_BUF,0,sizeof(UART2_TX_BUF));
	rtU.target_CH1_5=1000;
//	while(1){
//		active_can[0]=motor_data_can2[4]->activate;
//		if(active_can[0])
//			break;
//	}

//		rtU.target_CH2_5=m6020_2_1_init;
//	while(1){
//		active_can[1]=motor_data_can2[5]->activate;
//		if(active_can[1])
//			break;
//	}

//		rtU.target_CH2_6=m6020_2_2_init;
//	while(1){
//		active_can[2]=motor_data_can2[6]->activate;
//		if(active_can[2])
//			break;
//	}

//		rtU.target_CH2_7=m6020_2_3_init;

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//		if(RC.action==1)
//		{ 
//			int buff_len;
//			memset(UART2_TX_BUF,0,sizeof(UART2_TX_BUF));
//			buff_len = sprintf(UART2_TX_BUF,"%f\r\n",(float)deadband);
//			HAL_UART_Transmit_DMA(&huart2,(uint8_t *)UART2_TX_BUF,buff_len);
//		}
		
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	
		 if (huart->Instance == USART2)
    {
				if(uart_index[1] == 0 && buffer[1]!=0xA5){
					HAL_UART_Receive_IT(&huart2, buffer+1, 1);
			
				}else{
        UART2_RX_BUF[uart_index[1]++] = buffer[1];

        if (uart_index[1] == 16)
        {
            uart_index[1] = 0;
            if (UART2_RX_BUF[0] == 0xA5 && UART2_RX_BUF[15] == 0x5A) 
            {
								Receive();

								
            }
        }

        HAL_UART_Receive_IT(&huart2, buffer+1, 1);
			}
				
    }
		
    if (huart->Instance == UART7)
    {
			
			if(uart_index[0] == 0 && buffer[0]!=0x0F){
					HAL_UART_Receive_IT(&huart7, buffer, 1);
					
				}else
        UART7_RX_BUF[uart_index[0]++] = buffer[0];

        if (uart_index[0] == 24)
        {
            uart_index[0] = 0;
            if (UART7_RX_BUF[0] == 0x0F && UART7_RX_BUF[24] == 0x00) 
            {
								update_sbus(UART7_RX_BUF);
            }
        }

        HAL_UART_Receive_IT(&huart7, buffer, 1);
				
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART7)
  {
        HAL_UART_Receive_IT(&huart7, buffer+1, 1);

  }
  if (huart->Instance == USART2)
  {
    HAL_UART_Receive_IT(&huart2, buffer, 1);
  }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
  if (htim->Instance == TIM6)
  {		

		dm_motor_ctrl_send(&hfdcan1,&motor[Motor1]);
		
		float pos = 5.0*sin(3.14*cnt[0]/1000);
		motor[Motor1].ctrl.pos_set=pos;
		
		
    cnt[0]++;
		HAL_IWDG_Refresh(&hiwdg1);//feed the IWG


		Reach_TGT();
		get_msgn();
		assign_output();
    motor_state_update();
		set_reset_status();
		PID_MODEL_step();
		cnt[1]++;
		if(cnt[1]==500){
			HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_7);
			cnt[1]=0;
		}

  }
	
	 if (htim->Instance == TIM7)
	{
		reload();
		rtU.status_CH1_1=CUR;
		rtU.status_CH1_2=CUR;
		tor_output[CH1_1]=-shoot_data.output_stg;
		tor_output[CH1_2]=shoot_data.output_stg;
		SetServo(&htim4, TIM_CHANNEL_1, shoot_data.output_servo0, 20000, 20);
		SetServo(&htim4, TIM_CHANNEL_2, shoot_data.output_servo1, 20000, 20);
		
		
				if(SBUS_CH.ConnectState==1&&SBUS_CH.CH6<800){
			if(SBUS_CH.CH5>600&&SBUS_CH.CH5<1200&&shoot_flag[0]==0){
				shoot_flag[0]=1;
				shoot_data.shoot_ctl=1;
			}
			if(SBUS_CH.CH5<600&&shoot_flag[1]==0){
				shoot_flag[1]=1;
				shoot_data.shoot_ctl=4;
			}
			if(SBUS_CH.CH5>1200)
			{
				shoot_flag[0]=0;
				shoot_flag[1]=0;

			}
		}
		
		else if(SBUS_CH.ConnectState==1&&SBUS_CH.CH6<1200&&SBUS_CH.CH6>800){

			if(SBUS_CH.CH5<1200&&dri_flag==0){
				dri_flag=1;
				DribblingStatus=1;
			}
			if(SBUS_CH.CH5>1200)
			{
				dri_flag=0;
			}
		}
			cnt[2]++;
			
		
		}
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
