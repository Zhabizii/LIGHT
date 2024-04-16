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
#include "adc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>

#define RXBUFFERSIZE  256
#define RXBUFFERSIZE  256  
char RxBuffer[RXBUFFERSIZE]; 
   //最大接收字节数
char RxBuffer[RXBUFFERSIZE];   //接收数据
uint8_t aRxBuffer;			//接收中断缓冲
uint8_t Uart1_Rx_Cnt = 0;		//接收缓冲计数

#define RXBUFFERSIZE2  256
#define RXBUFFERSIZE2  256  
char RxBuffer2[RXBUFFERSIZE2]; 
   //最大接收字节数
char RxBuffer2[RXBUFFERSIZE2];   //接收数据
uint8_t aRxBuffer2;			//接收中断缓冲
uint8_t Uart1_Rx_Cnt2 = 0;		//接收缓冲计数

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t data[20]= "100";
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint16_t light_data = 0;
	float light_vu = 0;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer, 1);
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer2, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	
		
		HAL_ADC_Start(&hadc1);     //启动ADC转换
		HAL_ADC_PollForConversion(&hadc1, 50);   //等待转换完成，时间为50ms
		if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
	  {
			light_data = HAL_ADC_GetValue(&hadc1);   //获取AD值
			light_vu = light_data/3.3;
	  }
		
		printf("红外感应：%d , 光照：%d \n",HAL_GPIO_ReadPin(infrared_GPIO_Port,infrared_Pin),(uint16_t)light_vu);
		
		
		USART2_printf("%d,%d",HAL_GPIO_ReadPin(infrared_GPIO_Port,infrared_Pin),(uint16_t)light_vu);
	
		
		HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_RESET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_SET);
		HAL_Delay(500);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
	if(huart->Instance == USART1)
	{
		if(Uart1_Rx_Cnt >= 255)  //溢出判断
		{
			Uart1_Rx_Cnt = 0;
			memset(RxBuffer,0x00,sizeof(RxBuffer));
			HAL_UART_Transmit(&huart1, (uint8_t *)"数据溢出", 10,0xFFFF); 	
					
		}
		else
		{
			RxBuffer[Uart1_Rx_Cnt++] = aRxBuffer;   //接收数据转存
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);		
				HAL_UART_Transmit(&huart1, (uint8_t *)&RxBuffer, Uart1_Rx_Cnt,0xFFFF); //将收到的信息发送出去
							while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);//检测UART发送结束
				Uart1_Rx_Cnt = 0;
				memset(RxBuffer,0x00,sizeof(RxBuffer)); //清空数组
		}	
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);	
		HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer, 1);   //再开启接收中断		
	}
	else if(huart->Instance == USART2)
	{
		if(Uart1_Rx_Cnt2 >= 255)  //溢出判断
		{
				Uart1_Rx_Cnt2 = 0;
				memset(RxBuffer2,0x00,sizeof(RxBuffer2));
				HAL_UART_Transmit(&huart2, (uint8_t *)"数据溢出", 10,0xFFFF); 	

		}
		else
		{
				RxBuffer2[Uart1_Rx_Cnt2++] = aRxBuffer2;   //接收数据转存
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
				HAL_UART_Transmit(&huart1, (uint8_t *)&RxBuffer2, Uart1_Rx_Cnt2,0xFFFF); //将收到的信息发送出去
						while(HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX);//检测UART发送结束
				Uart1_Rx_Cnt2 = 0;
				memset(RxBuffer2,0x00,sizeof(RxBuffer2)); //清空数组
		}
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
		HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer2, 1);   //再开启接收中断
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
