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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "display.h"
#include "CAN.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

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
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim14;
DMA_HandleTypeDef hdma_tim2_ch1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader; //global??
uint32_t TxMailbox;
uint8_t TxData[8];
uint8_t RxData[8];

Lcd_HandleTypeDef lcd;

char lcdbuffer[32]; // buffer to hold the data we are displaying
char lcdbuffer2[32];
int batt_volt;
volatile int sixty = 70;
uint8_t rxflag = 0;
uint32_t volt = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN2_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART6_UART_Init(void);
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

	// UART parameters to display voltage
//	char voltage_buf[20];
//	char oil_buf[20];
//	char the_header[50];





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
  MX_TIM14_Init();
  MX_USART2_UART_Init();
  MX_CAN2_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);


  // timer is started
  HAL_TIM_Base_Start_IT(&htim14); // starts the timer interrupt
  //HAL_TIM_Base_Start_IT(&htim7);

//
  Lcd_PortType ports[] = {
		  D4_GPIO_Port, D5_GPIO_Port, D6_GPIO_Port, D7_GPIO_Port
  };

  Lcd_PinType pins[] = {D4_Pin, D5_Pin, D6_Pin, D7_Pin};

 lcd = Lcd_create(ports, pins, RS_GPIO_Port, RS_Pin, EN_GPIO_Port, EN_Pin, LCD_4_BIT_MODE);
 Lcd_clear(&lcd);
batt_volt = 5;
//  Lcd_string(&lcd, "RPM: 420 BV: 12");
//
//  Lcd_cursor(&lcd, 1,1);
//  Lcd_int(&lcd, -500);

  // basic 16x2 test
//  char buffer[16]; // Adjust size as needed
//


//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  while (1)
  {

//
	// int rpm = ((RxData[0] << 8) | RxData[1]);
	//  int rpm = ((RxData[0] | RxData[1]) << 8) * 6;

	  int rpm = (RxData[1] + (256 * RxData[0]));
	  int coolant = RxData[2];
	  int oil = RxData[3];
	  int bv = RxData[5];

	 if (rxflag){



		 rxflag = 0;

		 if (rpm > 0 || coolant > 0 || oil > 0 || bv > 0){

			 // int voltvalue = RxData[5];

		        //Lcd_clear(&lcd);
			 	Lcd_cursor(&lcd, 0, 0);
		        sprintf(lcdbuffer, "BV:%2d.%1d RPM:%5d", bv / 10, bv % 10, rpm);
		    	Lcd_string(&lcd, lcdbuffer);
//		        HAL_Delay(100);
		    //	Lcd_clear(&lcd);

		    	// condition 2:

		    	Lcd_cursor(&lcd, 1, 0);
		    	 sprintf(lcdbuffer, "OIL: %2d CT: %2d  ", oil, coolant);
		    	 Lcd_string(&lcd, lcdbuffer);
				    //	Lcd_clear(&lcd);
		    	 HAL_Delay(100);

		 }
//		 unsigned int uart_buffer_expression = sprintf(lcdbuffer2, "%u\r\n", RxData[5]);
//
//
//				         	  HAL_UART_Transmit(&huart2, (uint8_t *)lcdbuffer2, uart_buffer_expression, HAL_MAX_DELAY);

	//	 }
	 }

	 else {
		// HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);

//		 sprintf(lcdbuffer, "BV: 0");
//		 		    	Lcd_string(&lcd, lcdbuffer);



		 		    		// This is all the UART Code that Goes to the Arduino For Shift Lights
//		 		    		char arduino_msg[20];  // Create a buffer to hold the converted integer message
//
//		 		    	    // Convert the integer value to a string
//		 		    	    sprintf(arduino_msg, "%d\n", 200);
//
//		 		    	    // Transmit the string over UART6
//		 		    	    HAL_UART_Transmit(&huart6, (uint8_t*)arduino_msg, strlen(arduino_msg), HAL_MAX_DELAY);
//
//
		 Lcd_cursor(&lcd, 0, 0);
		     Lcd_string(&lcd, "BV: 0.0 RPM:  0 ");

		     Lcd_cursor(&lcd, 1, 0);
		     Lcd_string(&lcd, "OIL:  0 CT:  0  ");

		 		    	    HAL_Delay(80);

		 		    	  //  Lcd_clear(&lcd);



	 }


		char arduino_msg[20];  // Create a buffer to hold the converted integer message

	    // Convert the integer value to a string
	 //   sprintf(arduino_msg, "%d\n", RxData[5]);
		sprintf(arduino_msg, "%d\n", rpm);

	    // Transmit the string over UART6
	    HAL_UART_Transmit(&huart6, (uint8_t*)arduino_msg, strlen(arduino_msg), HAL_MAX_DELAY);






//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // Turn on decimal point


//	  TxData[0] = 0x23;
//      TxData[1] = 0x49;
//      TxData[2] = 0x69;
//


//


//	 if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
//		 //uint32_t can_error = HAL_CAN_GetError(&hcan2); // Can potentially use for debugging
//
////		 Error_Handler(); // Cause of UART Failure
//	 }


	 // ----------------- UART Code: -----------------------------


	 //sprintf(oil_buf, "Oil Temp: %u", oil_temp);
	// 	sprintf(voltage_buf, "Voltage: %u", raw_voltage);
	 // display the oil_temp through UART
//	 HAL_UART_Transmit(&huart2, (uint8_t*)oil_buf, strlen(oil_buf), HAL_MAX_DELAY);
//
	// HAL_UART_Transmit(&huart6, (uint8_t*)"Hello8\r\n", strlen("Hello8\r\n"), HAL_MAX_DELAY);

	 // display voltage
	// HAL_UART_Transmit(&huart2, (uint8_t*)voltage_buf, strlen(voltage_buf), HAL_MAX_DELAY);

	// HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

//
//
//

//
	   HAL_Delay(300); // delay of 1ms // was 500


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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  	  	  	  CAN_FilterTypeDef  canonefilter;


    		  canonefilter.FilterBank = 1; //change to 1 if CAN stops working
    		  canonefilter.FilterMode = CAN_FILTERMODE_IDMASK;
    		  canonefilter.FilterScale = CAN_FILTERSCALE_32BIT;
    		  canonefilter.FilterIdHigh = 0x0000;
    		  canonefilter.FilterIdLow = 0x0000;
    		  canonefilter.FilterMaskIdHigh = 0x0000;
    		  canonefilter.FilterMaskIdLow = 0x0000;
    		  canonefilter.FilterFIFOAssignment = CAN_RX_FIFO0;
    		  canonefilter.FilterActivation = CAN_FILTER_ENABLE;
    		  canonefilter.SlaveStartFilterBank = 1; // meaningless in our context


    		  	  HAL_CAN_ConfigFilter(&hcan1, &canonefilter);
    		  	  HAL_CAN_Start(&hcan1);

    		  	  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // potentially not needed

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */



  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 9;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  CAN_FilterTypeDef  canfilterconfig;


  		  canfilterconfig.FilterBank = 1; //change to 1 if CAN stops working
  		  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  		  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  		  canfilterconfig.FilterIdHigh = 0x0000;
  		  canfilterconfig.FilterIdLow = 0x0000;
  		  canfilterconfig.FilterMaskIdHigh = 0x0000;
  		  canfilterconfig.FilterMaskIdLow = 0x0000;
  		  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  		  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  		  canfilterconfig.SlaveStartFilterBank = 14; // meaningless in our context


  		  	  HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig);
  		  	  HAL_CAN_Start(&hcan2);

  		  	  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);



  	  	    TxHeader.StdId = 0x0446;  // ID 2 (to match H7's filter)
 	        TxHeader.IDE = CAN_ID_STD;  // Standard ID
 	        TxHeader.RTR = CAN_RTR_DATA;  // Data frame
 	        TxHeader.DLC = 8;  // Length of data (3 bytes)
 	        TxHeader.TransmitGlobalTime = DISABLE;

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 112;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 112.5 - 1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC0 PC1
                           PC2 PC3 PC5 PC8
                           PC9 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA5 PA8 PA9
                           PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB13 PB14 PB15 PB3
                           PB4 PB5 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	// changed values hcan->Instance = CAN2 and inside get hcan -> &hcan2

	if (hcan->Instance == CAN2) {
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {

    /* Reception Error */
    Error_Handler();
  	  }


  	//  batt_volt = RxData[5];
    	rxflag = 1;

    //	  int rpm2 = (RxData[1] + (256 * RxData[0]));

//    	int rpm2 = (RxData[1] + (256 * RxData[0]));
//
//
//      //uint32_t voltage = RxData[5];
//	  	//  char uart_buffer[20];
//	    //  unsigned int uart_buffer_size = sprintf(uart_buffer, "StdId: 0x%3X\r\n", (unsigned int) voltage);
//
//  	//  if (RxHeader.StdId == 0x623){
//	  char uart_buffer[20];
////
////	  // HAVING THE UART DEBUG STATEMENTS IN HERE WILL THROW OFF THE DISPLAY CODE
////
//////	   //Format as decimal
//	  unsigned int uart_buffer_expression = sprintf(uart_buffer, "Voltage: %d\n%d\r\n", rpm2, RxData[2]);
////	//  unsigned int uart_buffer_expression = sprintf(lcd_buffer, "Voltage: %u\r\n", batt_volt);
////
////
//	  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buffer, uart_buffer_expression, HAL_MAX_DELAY);

 // 	  }

  //if (RxHeader.StdId == 0x651){
	    	  //char uart_buffer[20];
	  	      //unsigned int uart_buffer_size = sprintf(uart_buffer, "StdId: 0x%3X\r\n", (unsigned int) RxHeader.StdId);

	  	    //  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buffer, uart_buffer_size, HAL_MAX_DELAY);
  //}


	}

}


// The function below this is the timer interrupt callback (Code within the specific timers constantly executes)

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//    static uint32_t timer_val = 0;  // Persistent timer value
//    static uint8_t led_state = 0;  // 0: LED off, 1: LED on
//
//
////
////
    if (htim == &htim14) { // Check if this is TIM14 interrupt

    	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

    }

////    // set a priority between the interrupts since the time at which they refresh doesn't matter as they will eventually run into each other
    if (htim->Instance == TIM7) {
//    	DisplayRxData(oil_temp);

    //	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

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
