/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdarg.h>
#define USART_REC_LEN 256
#define REC_OK		1	   //接收完成标志
#define REC_WAIT	0	   //接收未完成标志
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */
extern uint8_t 	rxConut;         			 			  //数据长度	
extern uint8_t 	regConut;         			 			  //数据长度	
extern uint8_t  USART_RX_BUF[USART_REC_LEN]; 	//接收缓冲,最大USART_REC_LEN个字节
extern uint8_t  USART2_RX_BUF[USART_REC_LEN];
extern uint16_t USART_RX_STA;         			  //接收状态标记	
extern uint16_t USART2_RX_STA;
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void USART2_printf(char *fmt,...);  //usart2发送函数
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

