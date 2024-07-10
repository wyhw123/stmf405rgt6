/* USER CODE BEGIN Header */

//************************����оδ��******************************************/
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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "ina226.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/******************************************PID��������************************************************/
float kp=20;
float ki=0.2;
float kd=0.5;

float input=0.0f;
float detpwm=0;
float integral;//����
float setcur,nowcur=0;
float error=0.0;
float lasterror=0.0;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t buffer[20];
uint16_t  k[4];

float batteryVolt=0.0,batteryCur=0.0,cupVolt=0.0,cupCur=0.0;
int Num1=0,Num2=0,Num3=0,Num4=0,Num5=0,Num6=0,STEP=0,Beep1=1;
int E1=0,E2=0,E3=0;
int SUP_Error=0;
int PWM=0,a=0;
float status=0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


  double PID_Calcu(float setcur, float input) //����ʽPID���㺯��
	{
		//�����ڻ�PID���������
		detpwm=0;
    error = setcur-input;
    integral += error;
    lasterror = error;
		float derivative = error - lasterror;//΢��
    float detpwm = kp * error + ki * integral + kd * derivative;//p׷��i׷�ĸ��죬d���ٶ���
	  if(detpwm>=20||detpwm<=-20)//�����޷�
		{detpwm=10;}
		return detpwm;
  }
	
	
	void Check(void)//�����⺯��
{
	if(batteryVolt>=23)
	{
		SUP_Error=1;
	}
	else if(batteryCur>=3)
	{
		SUP_Error=2;
	}
	else if(batteryVolt*batteryCur>=100)
	{
		SUP_Error=3;
	}
	else
	{
		SUP_Error=0;
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
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
	MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	
	/******************************************LED�Ʋ���************************************************/

//	HAL_GPIO_WritePin (LED1_GPIO_Port ,LED1_Pin ,GPIO_PIN_SET );
//	HAL_GPIO_WritePin (LED2_GPIO_Port ,LED2_Pin ,GPIO_PIN_SET );
//	HAL_GPIO_WritePin (LED3_GPIO_Port ,LED3_Pin ,GPIO_PIN_SET );

/******************************************���밴ť����*********************************************/

//	k[0]=HAL_GPIO_ReadPin (TRIG1_GPIO_Port ,TRIG1_Pin );  //���밴ť����ͨ��ťһ��ʹ�ã��ϲ���0���²���1
//	k[1]=HAL_GPIO_ReadPin (TRIG2_GPIO_Port ,TRIG2_Pin );
//	k[2]=HAL_GPIO_ReadPin (TRIG3_GPIO_Port ,TRIG3_Pin );
//	k[3]=HAL_GPIO_ReadPin (TRIG4_GPIO_Port ,TRIG4_Pin );

/******************************************OLED��ʾ������************************************************/

//	OLED_ShowString(0,0,(uint8_t *)"SuperCap:",16,1);        //(spi��oled��ʾ��ע�⣡��ʾ֮����ҪRefreshˢ����Ļ
//	OLED_ShowNum(0,17,k[0],1,16,1);
//  OLED_Refresh();//������ʾ


/******************************************CANͨѶ����************************************************/
//��ʼ�����룬while���շ�ѭ������
////	uint8_t Send_Data=0;//�ٷ�can.c�л������can���õĿ⺯���������пƴ�canͨѶ���롷
//  CAN_Init (&hcan1);
//	CAN_Filter_Mask_Config (&hcan1 ,CAN_FILTER (13)|CAN_FIFO_1|CAN_STDID |CAN_DATA_TYPE ,0x114,0x7ff );  //����canͨѶ��fifo

////  CAN_Send_Data (&hcan1 ,0x114,Send_Data,1); //�����ã�while����ѭ��


/******************************************INA226������ѹ���ģ�����************************************************/

//  float status=0;                               //INA226��ʼ��  
//	uint16_t configWord = 0x45C7;   //0x4527	 //address_in �����õ�ַ
//	status =INA226_setConfig(&hi2c1, INA226_ADDRESS_IN1,configWord);//INA226_MODE_CONT_SHUNT_AND_BUS|INA226_VSH_140uS|INA226_VBUS_140uS|INA226_AVG_1|INA226_RESET_ACTIVE���� INA226 ����/��ѹ���оƬ�ļĴ�����
//	INA226_setCalibrationReg(&hi2c1, INA226_ADDRESS_IN1,0x0A00);//����INA226оƬ��У׼�Ĵ���0x0A000x4127
////	INA226_setMaskEnable(&hi2c1, INA226_ADDRESS_OUT, INA226_RESET_INACTIVE);//����INA226оƬ������Ĵ���
////	INA226_setAlertLimit(&hi2c1, INA226_ADDRESS_OUT, INA226_RESET_INACTIVE);//����INA226оƬ�ľ������ƼĴ���


/******************************************USART����ģ�����************************************************/
	// __HAL_UART_ENABLE_IT (&huart1,UART_IT_IDLE );          //�ж�ʹ��
	// HAL_UART_Receive_DMA (&huart1,buffer,sizeof(buffer));  //cubemx����dma����ͨѶ
	
/******************************************����PWM����************************************************/	
//	HAL_TIM_Base_Start (&htim2 );
//	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
//	
//	
//  HAL_TIM_Base_Start(&htim1);  //����PWM�������ռ�ձ�480
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
//  HAL_TIMEx_PWMN_Start (&htim1,TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
//  HAL_TIMEx_PWMN_Start (&htim1,TIM_CHANNEL_2);
//	
//	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
//	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);

	
	HAL_GPIO_WritePin (LED1_GPIO_Port ,LED1_Pin ,GPIO_PIN_SET );//���뿪ʼ��ʼ����־��
	HAL_GPIO_WritePin (LED2_GPIO_Port ,LED2_Pin ,GPIO_PIN_SET );
	HAL_GPIO_WritePin (LED3_GPIO_Port ,LED3_Pin ,GPIO_PIN_SET );
	
	OLED_Init();//OLED��ʼ��
	HAL_Delay (10);
	OLED_Clear();
	OLED_ShowString(0,0,(uint8_t *)"SuperCap:",16,1);//��ʾ֮����ҪRefreshˢ����Ļ
  OLED_Refresh();//������ʾ
	
	__HAL_UART_ENABLE_IT (&huart1,UART_IT_IDLE );          //�ж�ʹ��
	HAL_UART_Receive_DMA (&huart1,buffer,sizeof(buffer));  //cubemx����dma����ͨѶ

	uint16_t configWord = 0x45C7;//0x4527	//INA226��ʼ��   //address_in �����õ�ַ
	status =INA226_setConfig(&hi2c1, INA226_ADDRESS_IN2,configWord);//INA226_MODE_CONT_SHUNT_AND_BUS|INA226_VSH_140uS|INA226_VBUS_140uS|INA226_AVG_1|INA226_RESET_ACTIVE���� INA226 ����/��ѹ���оƬ�ļĴ�����
	INA226_setCalibrationReg(&hi2c1, INA226_ADDRESS_IN2,0x0A00);//����INA226оƬ��У׼�Ĵ���0x0A000x4127
	status =INA226_setConfig(&hi2c1, INA226_ADDRESS_IN1,configWord);//INA226_MODE_CONT_SHUNT_AND_BUS|INA226_VSH_140uS|INA226_VBUS_140uS|INA226_AVG_1|INA226_RESET_ACTIVE���� INA226 ����/��ѹ���оƬ�ļĴ�����
	INA226_setCalibrationReg(&hi2c1, INA226_ADDRESS_IN1,0x0A00);//����INA226оƬ��У׼�Ĵ���0x0A000x4127

	k[0]=HAL_GPIO_ReadPin (TRIG1_GPIO_Port ,TRIG1_Pin );  //���밴ťʹ�ã��ϲ���0���²���1
	k[1]=HAL_GPIO_ReadPin (TRIG2_GPIO_Port ,TRIG2_Pin );
	k[2]=HAL_GPIO_ReadPin (TRIG3_GPIO_Port ,TRIG3_Pin );
	k[3]=HAL_GPIO_ReadPin (TRIG4_GPIO_Port ,TRIG4_Pin );
  OLED_ShowNum(0,17,k[0],1,16,1);
	OLED_ShowNum(10,17,k[1],1,16,1);
	OLED_ShowNum(20,17,k[2],1,16,1);
	OLED_ShowNum(30,17,k[3],1,16,1);
	OLED_Refresh();//������ʾ
	
	HAL_TIM_Base_Start_IT (&htim3 );//tim3 ������ʱ���ж�
	HAL_TIM_Base_Start_IT (&htim7 );//tim7 ������ʱ���ж�
  HAL_TIM_Base_Start (&htim2 );
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	
  HAL_TIM_Base_Start(&htim1);  //����PWM�������ռ�ձ�480
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start (&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start (&htim1,TIM_CHANNEL_2);
	
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
	
	HAL_GPIO_WritePin (LED1_GPIO_Port ,LED1_Pin ,GPIO_PIN_RESET );//�����ʼ����ɱ�־��
	HAL_GPIO_WritePin (LED2_GPIO_Port ,LED2_Pin ,GPIO_PIN_RESET );
	HAL_GPIO_WritePin (LED3_GPIO_Port ,LED3_Pin ,GPIO_PIN_RESET );

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,PWM);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,PWM);
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
/************************************************�����ж�****************************************************/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	switch(GPIO_Pin ) 
	{
		 case KEY1_Pin:setcur =0.5; break;
		 case KEY2_Pin:setcur =1.0; break;

	 }
	
}


/******************************************��ʱ���ж�TIM3,TIM7************************************************/
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
		{
			
			if(htim->Instance  == htim3 .Instance )
			{
				Num1++;
				Num2++;
				if(Num1 ==500)//һ��Ƶ��
				{		
					HAL_GPIO_TogglePin(LED1_GPIO_Port ,LED1_Pin);
					if(PWM<450)//�޷�
					{
					PWM+=PID_Calcu(setcur ,cupCur);//����ʽpid���ֵ
					}
					if(PWM<0)
					{
					PWM=0;
					}
					Num1=0;
					
				}
				if(Num2 ==10)  //������ѹ������
				{
					
		 			batteryVolt=INA226_getBusV(&hi2c1, INA226_ADDRESS_IN2);
		      batteryCur=INA226_getCurrent(&hi2c1, INA226_ADDRESS_IN2);
					cupVolt=INA226_getBusV(&hi2c1, INA226_ADDRESS_IN1);
		      cupCur=INA226_getCurrent(&hi2c1, INA226_ADDRESS_IN1);
					Num2 = 0;
		
				}
				
			}
			
			
			if(htim->Instance  == htim7 .Instance )
			{
				E1++;
				E2++;
				if(E1 ==500)//һ��Ƶ��
				{	
					if(SUP_Error==0)//�ޱ���
					{
						__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,0);
					}
					if(SUP_Error==3)//���������
					{
						__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,1250);
					}
					if(SUP_Error==1)//���ٷ�����
					{
					Beep1 =~Beep1;
						
					if(Beep1==1)
					__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,1250);	
					else 
					__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,0);
				  }
					E1=0;
				}
				
				if(E2 ==100)//���ٷ�����
				{	
					if(SUP_Error==2)
					{
					Beep1 =~Beep1;
					if(Beep1==1)
					__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,1250);	
					else 
					__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,0);		
				  }					
					E2=0;
				}


			}		
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
