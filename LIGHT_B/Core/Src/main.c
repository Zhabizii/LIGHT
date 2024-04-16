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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "oled.h"
#include "string.h"
#include "stdlib.h"
#include "esp8266.h"
#include "mqtt.h"

//////////////////////////////下面是自行修改参数的地方////////////////////////////////////////
#define ServerIP "183.230.40.39" //传统的mqtt协议ip
#define Port "6002"

#define ClientID "1203150932"    //需要定义为用户自己的参数 设备id
#define Username   "628799"		//需要定义为用户自己的参数 产品id
#define Password   "smlight"  //需要定义为用户自己的参数 鉴权信息

#define Topic   "$dp" //需要定义为用户自己的参数 不用改
#define TopicPost   "$dp" //需要定义为用户自己的参数 不用改

#define SSID "esp"
#define WIFIPassword "12345678"
///////////////////////////////////////////////////////////////

#define RXBUFFERSIZE  256 
   //最大接收字节数
uint16_t RxBuffer[RXBUFFERSIZE];   //接收数据
uint16_t aRxBuffer;			//接收中断缓冲
uint16_t Uart1_Rx_Cnt = 0;		//接收缓冲计数


#define RXBUFFERSIZE2  256  

//最大接收字节数
uint8_t RxBuffer2[RXBUFFERSIZE2];   //接收数据
uint8_t  RxBuffer3[RXBUFFERSIZE2];   //接收数据

char aRxBuffer2;			//接收中断缓冲
uint8_t Uart1_Rx_Cnt2 = 0;		//接收缓冲计数
uint16_t R_success = 0;

int light_pwm = 0;
int light_targe = 400,light_error = 0;;
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
   char  * CSTXREVData[3];
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
unsigned char servernotok=1;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t res=1;	
	uint8_t j=0;
	char *light_C;
	char *infrared_C;
		
	uint16_t infrared;
	uint16_t light;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
		OLED_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);  //开启PWM通道1
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
		USART_Interupt_Enable();
  /* USER CODE END 2 */
		while(res)
		{
			res=WIFI_Dect((uint8_t *)SSID,(uint8_t *)WIFIPassword);
			HAL_Delay(200);
		}
		res = 1;
		while(servernotok)
		{
			res=ESP8266_CONNECT_SERVER((uint8_t *)ServerIP,(uint8_t *)Port);
			HAL_Delay(1000);
		}
			servernotok=1;
			while(servernotok)
			{	
				res=mqtt_Connection((char *)Username,(char *)Password,(char *)ClientID);  //连接服务器
				if(servernotok == 0){					
					USART2_printf("Enter MQTT OK!\r\n");
				}
				else{		
					USART2_printf("Enter MQTT Error!\r\n");
				}	
			}
			servernotok=1;
			while(servernotok)
			{	
				res=mqtt_subscribe((char *)Topic);  //订阅消息
				if(servernotok == 0){					
					USART2_printf("Subscription succeeded!\r\n");
				}
				else{		
					USART2_printf("Subscription failed!\r\n");
				}	
			}
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	
		
			if(USART2_RX_STA == REC_OK)
			{
				CSTXREVData[0] = strtok((char *)USART2_RX_BUF,",");//A,18.1,62.5
				CSTXREVData[1] = strtok(NULL,",");		
				CSTXREVData[2] = strtok(NULL,",");
		
				infrared_C = CSTXREVData[0];
				light_C = CSTXREVData[1];
				
				infrared = atoi(infrared_C);
				light = atoi(light_C);		
				
				if(light != 0)
				{
					
					ESP8266_Send_data((char *)TopicPost,CSTXREVData[0],CSTXREVData[1]);  //发送数据			
					if( light > 100 && infrared == 1)//
					{
						HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
						light_error = light - light_targe;
						light_pwm = 1.4*light_error;
						light_pwm = abs(light_pwm) < 500 ? light_pwm : 500;
						light_pwm = 500 - light_pwm;
						
						__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, light_pwm); 						
						
					}
					else 
					{
							HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_RESET);
					}

					

					OLED_ShowString(1,1,"           ");
					OLED_ShowString(1,1,"infrard:");
					OLED_ShowNum(1,9,infrared,1);
					OLED_ShowString(2,1,"light:");
					OLED_ShowNum(2,7,light,4);			
					OLED_ShowSignedNum(3,1,light_pwm,5);
				}

					
				memset(USART2_RX_BUF,0,USART_REC_LEN);

				USART2_RX_STA = 0;				
		}
			

		
		HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_RESET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_SET);
		HAL_Delay(800);
		

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
}

/* USER CODE BEGIN 4 */
 

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
