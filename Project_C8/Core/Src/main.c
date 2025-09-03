/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "./HAL/key/key.h"
#include "./HAL/OLED/OLED_NEW.H"
#include "./HAL/delay/delay.h"
#include "./HAL/AliESP8266/AliESP8266.h"
#include "./HAL/ds18b20/ds18b20.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void Key_function(void);								//按键函数
void Monitor_function(void);						//监测函数
void Display_function(void);						//显示函数
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
                                                                                                                                                                        
uint8_t USART2_TX_BUF[255];
#define u2_printf(...)  HAL_UART_Transmit(&huart2,USART2_TX_BUF,sprintf((char *)USART2_TX_BUF,__VA_ARGS__),0xffff)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t key_num,flag_display;        	  //按键与显示变量
uint16_t time_1ms,time_500ms;         	//计时变量1ms,500ms

uint8_t Motor_Status;									//步进电机状态变量
uint16_t Motor_Num;										//步进电机计数变量
uint16_t Motor_Time;									//步进电机计时变量
uint8_t const Motor_Buf[8] = {0x01, 0x03, 0x02, 0x06, 0x04, 0x0c, 0x08, 0x09};//步进电机步数数组
uint8_t flag_moto;										//标志位
uint16_t body_temp;										//体温
uint16_t adc_value,humi;				      //ADC值，湿度变量

uint8_t mode,hot_flag,fan_flag;
uint8_t crib_flag;                      //婴儿床标志位
uint8_t beep_temp,beep_humi;

uint16_t time_1,flag_1,time_2,flag_2;         			//计时变量


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/****
*******按键设置函数
*****/
void Key_function(void)
{
	key_num = Chiclet_Keyboard_Scan();		//按键扫描
	if(key_num != 0)			                //有按键按下
	{
	 
	 
	}
}

/****
*******监测函数
*****/
void Monitor_function(void)
{
  uint16_t temp_init;
	if(time_500ms == 1)
	{
		time_500ms = 0;
    if(body_temp < 1000)
      temp_init = body_temp;
		body_temp=Ds18b20_Read_Temp();
    if(body_temp > 1000)
      body_temp = temp_init;
     
	}
  
   
  
  if(voice == 0 || (crib_flag == 1 && mode == 1)) //检测到哭声，或手动控制婴儿床的标志位为1，婴儿床自动摇篮
  {
    Motor_Status |= 0x81;
  }
  
  if(mode == 0)                         //自动模式时
  {
     hot_flag = 1;
      fan_flag = 0;
      beep_temp = 1;
  }
  else
  {
    beep_temp = 0;
  }
 
  
  
  relay_fan(fan_flag);
  relay_hot(hot_flag);
  
}


/****
*******显示函数
*****/
void Display_function(void)						//显示体温、模式、是否尿床、是否听到哭声
{
	Oled_ShowCHinese(0,0,(uint8_t *)"模式");
	Oled_ShowString(32,0,(uint8_t *)":");
	if(mode == 0)
		Oled_ShowCHinese(40,0,(uint8_t *)"自动");
	else
		Oled_ShowCHinese(40,0,(uint8_t *)"手动");
	 
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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_UART_Receive_IT(&Huart_wifi, &uartwifi_value, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   
  
  while (1)
  {
    Key_function();											//按键函数
		Monitor_function();									//监测函数
		Display_function();									//显示函数
    Ali_MQTT_Recevie();                 //接收数据
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim1.Instance)		//定时器1触发中断
	{
		time_1ms++;
		if(time_1ms>= 500)
		{
			time_1ms= 0;
			time_500ms = 1;
		}
	
		time_1++;
		if(time_1>= 1000)
		{
			time_1= 0;
			flag_1 = 1;											//1s标志位
		}
		time_2++;
		if(time_2>= 1400)
		{
			time_2= 0;
			flag_2 = 1;											//1.4s标志位
		}
		
    //1ms定时器中
    if(Motor_Status & 0x80)						//步进电机转动
		{
			if(Motor_Status & 0x01)
				Motor_Num++;
      else
				Motor_Num--;
			if(Motor_Num >= 256*8)
			{
				Motor_Status &= ~0x80;
        flag_moto=0;        
			}
			GPIOB->ODR &= 0xf0ff;           //引脚状态
			GPIOB->ODR |= Motor_Buf[Motor_Num%8] << 8;
		}
    if(Motor_Status & 0x01)//延时
		{
			Motor_Time++;
			if(Motor_Time >= 1500)
			{
				Motor_Time = 0;
        Motor_Status &= ~0x01;
				Motor_Status |= 0x80;
				
			}
		}
		

	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == Huart_wifi.Instance)//串口触发中断
	{
    if(huart->Instance == Huart_wifi.Instance)//串口触发
    {
      HAL_UART_Receive_IT(&Huart_wifi, &uartwifi_value, 1);
      if(ESP8266_cnt >= sizeof(ESP8266_buf))	ESP8266_cnt = 0; //防止串口被刷爆
      ESP8266_buf[ESP8266_cnt++] = uartwifi_value;	
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
