/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "structs.h"
#include "usart.h"
#include "fsm.h"
#include "spi.h"
#include "gpio.h"
#include "adc.h"
#include "foc.h"
#include "can.h"
#include "position_sensor.h"
#include "hw_config.h"
#include "user_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 RX0 interrupt.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&CAN_H);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  HAL_StatusTypeDef status = HAL_CAN_GetRxMessage(&CAN_H, CAN_RX_FIFO0, &can_rx.rx_header, can_rx.data);
  if(status == HAL_OK)
  {
	  printf("\r\n Got CAN msg: '%d', '%d' ... \n\r", can_rx.data[0], can_rx.data[1]);
  }


  //uint32_t TxMailbox;
  //pack_reply(&can_tx, CAN_ID,  comm_encoder.angle_multiturn[0]/GR, comm_encoder.velocity/GR, controller.i_q_filt*KT*GR);	// Pack response
  //HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data, &TxMailbox);	// Send response

  /* Check for special Commands */
  /*
  if(((can_rx.data[0]==0xFF) & (can_rx.data[1]==0xFF) & (can_rx.data[2]==0xFF) & (can_rx.data[3]==0xFF) & (can_rx.data[4]==0xFF) & (can_rx.data[5]==0xFF) & (can_rx.data[6]==0xFF) & (can_rx.data[7]==0xFC))){
	  update_fsm(&state, MOTOR_CMD);
      }
  else if(((can_rx.data[0]==0xFF) & (can_rx.data[1]==0xFF) & (can_rx.data[2]==0xFF) & (can_rx.data[3]==0xFF) * (can_rx.data[4]==0xFF) & (can_rx.data[5]==0xFF) & (can_rx.data[6]==0xFF) & (can_rx.data[7]==0xFD))){
      update_fsm(&state, MENU_CMD);
      }
  else if(((can_rx.data[0]==0xFF) & (can_rx.data[1]==0xFF) & (can_rx.data[2]==0xFF) & (can_rx.data[3]==0xFF) * (can_rx.data[4]==0xFF) & (can_rx.data[5]==0xFF) & (can_rx.data[6]==0xFF) & (can_rx.data[7]==0xFE))){
	  update_fsm(&state, ZERO_CMD);
      }
  else{
	  unpack_cmd(can_rx, controller.commands);	// Unpack commands
	  controller.timeout = 0;					// Reset timeout counter
  }
  */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
	//HAL_GPIO_WritePin(LED, GPIO_PIN_SET );	// Useful for timing
	//printf("\r\n TIM1 ... \n\r");
	/* Sample ADCs */
	//analog_sample(&controller);

	/* Sample position sensor */
	//ps_sample(&comm_encoder, DT);

	/* Run Finite State Machine */
	run_fsm(&state);

	/* Check for CAN messages */
	//can_tx_rx();

	/* increment loop count */
	//controller.loop_count++;
	//HAL_GPIO_WritePin(LED, GPIO_PIN_RESET );

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	HAL_UART_IRQHandler(&huart);

	char c = Serial2RxBuffer[0];
	printf("\r\n Got cmd '%c', updating FSM ... \n\r", c);

	//can_tx_rx();

	update_fsm(&state, c);

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart);
  /* USER CODE BEGIN USART1_IRQn 1 */
  /* USER CODE END USART1_IRQn 1 */
}


/* USER CODE BEGIN 1 */

void can_tx_rx(void){

	uint32_t TxMailbox;
	can_tx.data[0] = CAN_ID;
	can_tx.data[1] = 1;
	can_tx.data[2] = 2;
	can_tx.data[3] = 3;
	can_tx.data[4] = 4;
	can_tx.data[5] = 5;
	HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data, &TxMailbox);
	printf("\r\n Sending CAN msg, status = %d ... \n\r", status);

	uint32_t tx_level = HAL_CAN_GetTxMailboxesFreeLevel(&CAN_H);
	uint32_t rx_level = HAL_CAN_GetRxFifoFillLevel(&CAN_H, CAN_RX_FIFO0);

	printf("\r\n tx_level = %ld and rx_level = %ld ... \n\r", tx_level, rx_level);

	/*
	int no_mesage = HAL_CAN_GetRxMessage(&CAN_H, CAN_RX_FIFO0, &can_rx.rx_header, can_rx.data);	// Read CAN
	if(!no_mesage){
		uint32_t TxMailbox;
		pack_reply(&can_tx, CAN_ID,  comm_encoder.angle_multiturn[0]/GR, comm_encoder.velocity/GR, controller.i_q_filt*KT*GR);	// Pack response
		HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data, &TxMailbox);	// Send response

		// Check for special Commands
		if(((can_rx.data[0]==0xFF) & (can_rx.data[1]==0xFF) & (can_rx.data[2]==0xFF) & (can_rx.data[3]==0xFF) & (can_rx.data[4]==0xFF) & (can_rx.data[5]==0xFF) & (can_rx.data[6]==0xFF) & (can_rx.data[7]==0xFC))){
			  update_fsm(&state, MOTOR_CMD);
			}
		else if(((can_rx.data[0]==0xFF) & (can_rx.data[1]==0xFF) & (can_rx.data[2]==0xFF) & (can_rx.data[3]==0xFF) * (can_rx.data[4]==0xFF) & (can_rx.data[5]==0xFF) & (can_rx.data[6]==0xFF) & (can_rx.data[7]==0xFD))){
			update_fsm(&state, MENU_CMD);
			}
		else if(((can_rx.data[0]==0xFF) & (can_rx.data[1]==0xFF) & (can_rx.data[2]==0xFF) & (can_rx.data[3]==0xFF) * (can_rx.data[4]==0xFF) & (can_rx.data[5]==0xFF) & (can_rx.data[6]==0xFF) & (can_rx.data[7]==0xFE))){
			  update_fsm(&state, ZERO_CMD);
			}
		else{
			  unpack_cmd(can_rx, controller.commands);	// Unpack commands
			  controller.timeout = 0;					// Reset timeout counter
		}
	}
	*/
}
/* USER CODE END 1 */
