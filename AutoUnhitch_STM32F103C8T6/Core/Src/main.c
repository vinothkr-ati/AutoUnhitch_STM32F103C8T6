/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

volatile uint8_t aa = 0;
volatile uint8_t bb = 0;
volatile uint8_t cc = 0;
volatile uint8_t dd = 0;

//***************        ADC          *********************

volatile uint32_t ADC_VAL[4];

///////////      CAN1       ////////////////////////////

CAN_RxHeaderTypeDef CAN1RX_Header;
volatile uint8_t    CAN1_Queue_TX[12];
volatile uint8_t    CAN1RX_DATA[8];
volatile uint8_t    CAN1TX_DATA[8];
volatile uint32_t   CAN1RX_ID_Queue = 0;
volatile uint32_t   CAN1RX_IDE_DATA = 0;
volatile uint32_t   CAN1RX_Std_ID_DATA = 0;
volatile uint32_t   CAN1RX_Ext_ID_DATA = 0;


///**************  GPIO READ
volatile uint8_t DIGITAL_SENSE1 = 0;
volatile uint8_t DIGITAL_SENSE2 = 0;
volatile uint8_t nFAULT         = 0;

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
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* Definitions for Read_Sensors */
osThreadId_t Read_SensorsHandle;
const osThreadAttr_t Read_Sensors_attributes = {
  .name = "Read_Sensors",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CANRX_Data_Proc */
osThreadId_t CANRX_Data_ProcHandle;
const osThreadAttr_t CANRX_Data_Proc_attributes = {
  .name = "CANRX_Data_Proc",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CAN1RX_Data_Queue */
osMessageQueueId_t CAN1RX_Data_QueueHandle;
const osMessageQueueAttr_t CAN1RX_Data_Queue_attributes = {
  .name = "CAN1RX_Data_Queue"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
void Read_Sensors_Func(void *argument);
void CANRX_Data_Process_Func(void *argument);
void StartTask03(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void Delay_Micro_Seconds (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim3,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim3) < us);  // wait for the counter to reach the us input in the parameter
}

HAL_StatusTypeDef CAN_TransmitMessage_Std_ID(CAN_HandleTypeDef *hcan,uint32_t Std_ID, uint8_t *data, uint8_t len) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox = 0;
    HAL_StatusTypeDef status;

    // Configure TxHeadervolatile uint16_t  Left_End_Stop_Steering  = 0;
    volatile uint16_t  Right_End_Stop_Steering = 0;
    TxHeader.StdId = Std_ID;           // Standard Identifier

    TxHeader.IDE = CAN_ID_STD;        // Standard ID

    TxHeader.RTR = CAN_RTR_DATA;      // Data frame
    TxHeader.DLC = len;               // Data Length Code
    TxHeader.TransmitGlobalTime = DISABLE;

    // Request transmission
    status = HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &TxMailbox);

    if (status != HAL_OK) {
        // Transmission request failed
        return status;
    }

    // Wait until the transmission is complete
    while (HAL_CAN_IsTxMessagePending(hcan, TxMailbox));

    // Transmission successful, mailbox is freed automatically

    return HAL_OK;
}




/*
 * STD ID
 */
HAL_StatusTypeDef CAN_TransmitMessage_Ext_ID(CAN_HandleTypeDef *hcan, uint32_t Ext_ID, uint8_t *data, uint8_t len) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox = 0;
    HAL_StatusTypeDef status;

    // Configure TxHeader
            // Standard Identifier
    TxHeader.ExtId = Ext_ID;            // Extended Identifier (not used in this case)

    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.RTR = CAN_RTR_DATA;      // Data frame
    TxHeader.DLC = len;               // Data Length Code
    TxHeader.TransmitGlobalTime = DISABLE;

    // Request transmission
    status = HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &TxMailbox);

    if (status != HAL_OK) {
        // Transmission request failed
        return status;
    }

    // Wait until the transmission is complete
    while (HAL_CAN_IsTxMessagePending(hcan, TxMailbox));

    // Transmission successful, mailbox is freed automatically
    return HAL_OK;
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{


    if (hcan->Instance == CAN1)
       {
            HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN1RX_Header, CAN1RX_DATA);
            // Process CAN1 Data here
            CAN1_Queue_TX[0] = CAN1RX_DATA[0];
			CAN1_Queue_TX[1] = CAN1RX_DATA[1];
			CAN1_Queue_TX[2] = CAN1RX_DATA[2];
			CAN1_Queue_TX[3] = CAN1RX_DATA[3];

			CAN1_Queue_TX[4] = CAN1RX_DATA[4];
			CAN1_Queue_TX[5] = CAN1RX_DATA[5];
			CAN1_Queue_TX[6] = CAN1RX_DATA[6];
			CAN1_Queue_TX[7] = CAN1RX_DATA[7];

			//		CAN1_RX_DLC = RxHeader.DLC;

			CAN1RX_IDE_DATA = CAN1RX_Header.IDE;
			if(CAN1RX_IDE_DATA == 0) {
				CAN1RX_Std_ID_DATA = CAN1RX_Header.StdId;
				CAN1_Queue_TX[8]  =  (CAN1RX_Std_ID_DATA >> 0)  & 0xFF;
				CAN1_Queue_TX[9]  =  (CAN1RX_Std_ID_DATA >> 8)  & 0xFF;
				CAN1_Queue_TX[10] =  (CAN1RX_Std_ID_DATA >> 16) & 0xFF;
				CAN1_Queue_TX[11] =  (CAN1RX_Std_ID_DATA >> 24) & 0xFF;
			}else {
				CAN1RX_Ext_ID_DATA = CAN1RX_Header.ExtId;
				CAN1_Queue_TX[8] =  (CAN1RX_Ext_ID_DATA >> 0)  & 0xFF;
				CAN1_Queue_TX[9] = (CAN1RX_Ext_ID_DATA >> 8)  & 0xFF;
				CAN1_Queue_TX[10] = (CAN1RX_Ext_ID_DATA >> 16) & 0xFF;
				CAN1_Queue_TX[11] = (CAN1RX_Ext_ID_DATA >> 24) & 0xFF;
			}
			 // Attempt to send data to the queue in a non-blocking way
			osStatus_t status = osMessageQueuePut(CAN1RX_Data_QueueHandle, CAN1_Queue_TX, 0, 0);  // No timeout, no blocking
			if (status != osOK)
			{
			  // Queue full, data not sent; could handle this or log it
			}


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
  MX_CAN_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	HAL_Delay(100);// just normal
	//Timer PWM Start for uS delay generation
		HAL_TIM_Base_Start(&htim3);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

	// Must have Delay
	Delay_Micro_Seconds(15);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_Delay(5);
	 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);


	  TIM1->CCR1 = 250;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
	//CAN Start
	HAL_CAN_Start(&hcan);

	// Activate the notification
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);



  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of CAN1RX_Data_Queue */
  CAN1RX_Data_QueueHandle = osMessageQueueNew (20, sizeof(CAN1_Queue_TX), &CAN1RX_Data_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Read_Sensors */
  Read_SensorsHandle = osThreadNew(Read_Sensors_Func, NULL, &Read_Sensors_attributes);

  /* creation of CANRX_Data_Proc */
  CANRX_Data_ProcHandle = osThreadNew(CANRX_Data_Process_Func, NULL, &CANRX_Data_Proc_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  CAN_FilterTypeDef can1FilterConfig;
  can1FilterConfig.FilterBank = 0;                           // Bank 0 for CAN1
  can1FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  can1FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  can1FilterConfig.FilterIdHigh = 0x0000;
  can1FilterConfig.FilterIdLow = 0x0000;
  can1FilterConfig.FilterMaskIdHigh = 0x0000;
  can1FilterConfig.FilterMaskIdLow = 0x0000;
  can1FilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;  // ⬅️ FIFO0 for CAN1
  can1FilterConfig.FilterActivation = ENABLE;
  can1FilterConfig.SlaveStartFilterBank = 0;           // doesn't matter in single can controllers
  HAL_CAN_ConfigFilter(&hcan, &can1FilterConfig);



  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 500;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


//
//ADC_Auto_Unhitch_Read(ADC_VAL);
//ADC_Current_Sensor_1_Read(ADC_VAL);
//ADC_Current_Sensor_2_Read(ADC_VAL);
//ADC_24V_Read(ADC_VAL);
//
//
//DIGITAL_SENSE1 = HAL_GPIO_ReadPin(GPIOB ,GPIO_PIN_15);
//DIGITAL_SENSE2 = HAL_GPIO_ReadPin(GPIOB ,GPIO_PIN_14);
//nFAULT = HAL_GPIO_ReadPin(GPIOA ,GPIO_PIN_10);
//
//aa++;
//osDelay(100);
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Read_Sensors_Func */
/**
  * @brief  Function implementing the Read_Sensors thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Read_Sensors_Func */
void Read_Sensors_Func(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

	ADC_Auto_Unhitch_Read(ADC_VAL);
	ADC_Current_Sensor_1_Read(ADC_VAL);
	ADC_Current_Sensor_2_Read(ADC_VAL);
	ADC_24V_Read(ADC_VAL);


	DIGITAL_SENSE1 = HAL_GPIO_ReadPin(GPIOB ,GPIO_PIN_15);
	DIGITAL_SENSE2 = HAL_GPIO_ReadPin(GPIOB ,GPIO_PIN_14);
	nFAULT = HAL_GPIO_ReadPin(GPIOA ,GPIO_PIN_10);

	aa++;
	osDelay(100);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_CANRX_Data_Process_Func */
/**
* @brief Function implementing the CANRX_Data_Proc thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CANRX_Data_Process_Func */
void CANRX_Data_Process_Func(void *argument)
{
  /* USER CODE BEGIN CANRX_Data_Process_Func */

	uint8_t Received_CAN1RX[12];
		osStatus_t Status_CAN1;
  /* Infinite loop */
  for(;;)
  {

	  // Wait for and receive data from the queue
	 			  Status_CAN1 = osMessageQueueGet(CAN1RX_Data_QueueHandle, Received_CAN1RX, NULL, osWaitForever);
	 			  if (Status_CAN1 == osOK)  {
	 				 // Process Received_CAN1RX data
	 				 // Indicate processing by toggling an LED, etc.

	 				 // Reconstruct uint32_t from bb (Little-endian format)
	 				CAN1RX_ID_Queue = 	( Received_CAN1RX[8]  <<  0 ) |
	 									( Received_CAN1RX[9]  <<  8 ) |
	 									( Received_CAN1RX[10] << 16 ) |
	 									( Received_CAN1RX[11] << 24 );

	 			//	switch (CAN1RX_ID_Queue) {

//	 				case BrakeRX_ID:
//
//
//	 					//Reconstruct aa from bb[0] and bb[1] (little endian)
//	 					uint16_t Left_brake   = Received_CAN1RX[0] | (Received_CAN1RX[1] << 8);
//	 					uint16_t Right_brake  = Received_CAN1RX[2] | (Received_CAN1RX[3] << 8);
//
//	 					Brake1_Percentage = (float) Left_brake / 10;
//	 					Brake2_Percentage = (float) Right_brake / 10;
	 					//break;
//
//	 				default:
//
//	 					break;
//	 				}
	 			 }
    osDelay(1);
  }
  /* USER CODE END CANRX_Data_Process_Func */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  { cc++;
    osDelay(10);
  }
  /* USER CODE END StartTask03 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
#ifdef USE_FULL_ASSERT
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
