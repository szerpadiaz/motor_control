/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "hw_config.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef USART_H;


/* USART1 init function */

void MX_USARTx_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_H.Instance = USARTx;
  USART_H.Init.BaudRate = 9600;
  USART_H.Init.WordLength = UART_WORDLENGTH_8B;
  USART_H.Init.StopBits = UART_STOPBITS_1;
  USART_H.Init.Parity = UART_PARITY_NONE;
  USART_H.Init.Mode = UART_MODE_TX_RX;
  USART_H.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  USART_H.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&USART_H) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USARTx)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
	USARTx_CLK_ENABLE();
	USARTx_GPIO_ENABLE();


    /**USART1 GPIO Configuration
    PA2     ------> USART1_TX
    PA3     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = USARTx_AF;
    HAL_GPIO_Init(USART_GPIO_Port, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USARTx_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(USARTx_IRQ);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USARTx)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
	 USARTx_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA2     ------> USART1_TX
    PA3     ------> USART1_RX
    */
    HAL_GPIO_DeInit(USART_GPIO_Port, USART_TX_Pin|USART_RX_Pin);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USARTx_IRQ);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
int __io_putchar(int ch) {
HAL_UART_Transmit(&USART_H, (uint8_t*)&ch, 1, 0xffff);
return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
	HAL_UART_Receive_IT(&USART_H, (uint8_t *)Serial2RxBuffer, 1);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
