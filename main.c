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
#include <stdio.h>

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

//PID Timer
int change =0;

// Buffer for UART TX data
uint16_t TxData[20];
char send[20];
int ReadyToSend = 1;

// Buffer for UART RX data
int NewCmd = 0;
#define RxBufferSize 40
uint8_t RxData[RxBufferSize];

//Common to motor A and B
volatile uint32_t currentMillis = 0;
volatile uint32_t PrevPID = 0;

#define Count_BUFFER_SIZE 10
int Count_Error[Count_BUFFER_SIZE];

//_________________CHECK STOPS_____________________________________


//_________________ODOMETRY_____________________________________
int DirMutA = 1;
int DirMutB = 1;
volatile float tencountR = 0;
volatile float tencountL = 0;

volatile float NL = 0;
volatile float NR = 0;

//___________________________________________________________________________________________

//MOTOR A
int DirA = 1;

int SetRPM_A = 0;
volatile uint32_t PWM_A = 0;
volatile int RPMA = 0;

#define IN1_PIN GPIO_PIN_7
#define IN1_GPIO GPIOA

#define IN2_PIN GPIO_PIN_6
#define IN2_GPIO GPIOA

//ENCODER A
volatile uint32_t preViousMillis_A = 0;
volatile uint32_t PrevChckStop = 0;

int counterValue_A, pastCounterValue_A = 0;
float angleValue_A = 0;

//PID MOTOR_A
float P_GainA = 0.1;
float I_GainA = 0.01;
float D_GainA = 0.4;
int Prev_ErrorA = 0;

uint32_t IntegralErrorA = 0;
#define BUFFER_SIZE 10
int MotorA_Error[BUFFER_SIZE];

//_____________________________________________________________________________________________________________________

//MOTOR B
int DirB = 1;
int SetRPM_B = 0;
volatile uint32_t PWM_B = 0;
volatile int RPMB = 0;

#define IN3_PIN GPIO_PIN_2
#define IN3_GPIO GPIOB

#define IN4_PIN GPIO_PIN_10
#define IN4_GPIO GPIOB

//ENCODER A
volatile uint32_t preViousMillis_B = 0;
int counterValue_B, pastCounterValue_B = 0;
float angleValue_B = 0;

//PID MOTOR_B
float P_GainB = 0.1;
float I_GainB = 0.01;
float D_GainB = 0.4;
int Prev_ErrorB = 0;

uint32_t IntegralError_B = 0;
int MotorB_Error[BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_UART_Receive_DMA(&huart1, RxData, 6);

	DirA = (RxData[0] - 48);
	SetRPM_A = (((RxData[2] - 48) + ((RxData[1] - 48) * 10))) * 2;

	DirB = (RxData[3] - 48);
	SetRPM_B = (((RxData[5] - 48) + ((RxData[4] - 48) * 10))) * 2;

	TIM4->CNT = 20000;
	TIM2->CNT = 20000;

	NR = 0;
	NL = 0;


	NewCmd = 1;
	change=1;

	for (int i = 0; i < RxBufferSize; i++) {
		RxData[i] = 0;
	}

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	ReadyToSend = 1;

}

void motorA_RUN(int pwm) {

	if (pwm < 0) {
		pwm = 0;
	}
	if (pwm >= 255) {
		pwm = 255;
	}

	if (DirA == 1) {
		HAL_GPIO_WritePin(IN1_GPIO, IN1_PIN, 0);
		HAL_GPIO_WritePin(IN2_GPIO, IN2_PIN, 1);

		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwm);

	} else if (DirA == 2) {
		HAL_GPIO_WritePin(IN1_GPIO, IN1_PIN, 1);
		HAL_GPIO_WritePin(IN2_GPIO, IN2_PIN, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm);

	} else {
		HAL_GPIO_WritePin(IN1_GPIO, IN1_PIN, 0);
		HAL_GPIO_WritePin(IN2_GPIO, IN2_PIN, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	}

}

void motorB_RUN(int pwm) {

	if (pwm < 0) {
		pwm = 0;
	}
	if (pwm >= 255) {
		pwm = 255;
	}

	if (DirB == 1) {
		HAL_GPIO_WritePin(IN3_GPIO, IN3_PIN, 0);
		HAL_GPIO_WritePin(IN4_GPIO, IN4_PIN, 1);

		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwm);

	} else if (DirB == 2) {
		HAL_GPIO_WritePin(IN3_GPIO, IN3_PIN, 1);
		HAL_GPIO_WritePin(IN4_GPIO, IN4_PIN, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm);

	} else {
		HAL_GPIO_WritePin(IN3_GPIO, IN3_PIN, 0);
		HAL_GPIO_WritePin(IN4_GPIO, IN4_PIN, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	}

}
int MovingAvarageFilter_A(int NewError) {
	// Shift all elements to the right
	for (int i = (BUFFER_SIZE - 1); i > 0; i--) {
		MotorA_Error[i] = MotorA_Error[i - 1];
	}
	// Insert new error at the beginning
	MotorA_Error[0] = NewError;

	// Calculate the sum of the buffer
	int sum = 0;
	for (int i = 0; i < BUFFER_SIZE; i++) {
		sum += MotorA_Error[i];
	}
	return sum;

}

int MovingAvarageFilter_B(int NewError) {
	// Shift all elements to the right
	for (int i = (BUFFER_SIZE - 1); i > 0; i--) {
		MotorB_Error[i] = MotorB_Error[i - 1];
	}
	// Insert new error at the beginning
	MotorB_Error[0] = NewError;

	// Calculate the sum of the buffer
	int sum = 0;
	for (int i = 0; i < BUFFER_SIZE; i++) {
		sum += MotorB_Error[i];
	}
	return sum;

}






int PID_A(int rpma) {
	int Error = SetRPM_A - rpma;
	float P = ((float) Error) * P_GainA;
	float I = ((float) (MovingAvarageFilter_A(Error))) * I_GainA;
	float D = ((float) (Error - Prev_ErrorA)) * D_GainA;
	int PID = (int) (P + I + D);
	Prev_ErrorA = Error;
	return PID;

}

int PID_B(int rpma) {
	int Error = SetRPM_B - rpma;
	float P = ((float) Error) * P_GainB;
	float I = ((float) (MovingAvarageFilter_B(Error))) * I_GainB;
	float D = ((float) (Error - Prev_ErrorB)) * D_GainB;
	int PID = (int) (P + I + D);
	Prev_ErrorB = Error;
	return PID;

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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

	//UART RECEIVE DMA
	HAL_UART_Receive_DMA(&huart1, RxData, 6);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); //PB1
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); //PB0

	//START ENCODER MODES
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	//CHECK STOPS
	float PrevNR=0;
	float PrevNL=0;

	//START TIMING THE MOTORS

	TIM4->CNT = 20000;
	TIM2->CNT = 20000;
	preViousMillis_A = HAL_GetTick();
	preViousMillis_B = HAL_GetTick();

	//TIME PID CALCULATE INTERVALS
	PrevPID = HAL_GetTick();
	PrevChckStop = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		currentMillis = HAL_GetTick();
		counterValue_A = TIM4->CNT;
		counterValue_B = TIM2->CNT;

		if (counterValue_A >= 20200 && DirA == 1) {
			int time = currentMillis - preViousMillis_A;
			float speedy = ((float) 60000 / (float) time)
					* ((float)((TIM4->CNT)-20000) / (float) 2000);
			RPMA = (int) speedy;
			tencountR = tencountR + 1;
			preViousMillis_A = currentMillis;
			TIM4->CNT = 20000;
			change=1;
		}

		if (counterValue_B >= 20200 && DirB == 1) {
			int time = currentMillis - preViousMillis_B;
			float speedy = ((float) 60000 / (float) time)
					* ((float)((TIM2->CNT)-20000) / (float) 2000);
			RPMB = (int) speedy;
			tencountL = tencountL + 1;
			preViousMillis_B = currentMillis;
			TIM2->CNT = 20000;
			change=1;
		}

		if (counterValue_A <= 19800 && DirA == 2) {
			int time = currentMillis - preViousMillis_A;
			float speedy = ((float) 60000 / (float) time)
					* ((float)(20000 - (TIM4->CNT)) / (float) 2000);
			RPMA = (int) speedy;
			tencountR = tencountR + 1;
			preViousMillis_A = currentMillis;
			TIM4->CNT = 20000;
			change=1;
		}

		if (counterValue_B <= 19800 && DirB == 2) {
			int time = currentMillis - preViousMillis_B;
			float speedy = ((float) 60000 / (float) time)
					* ((float) (20000 - (TIM2->CNT)) / (float) 2000);
			RPMB = (int) speedy;
			tencountL = tencountL + 1;
			preViousMillis_B = currentMillis;
			TIM2->CNT = 20000;
			change=1;
		}

		if((currentMillis- PrevChckStop )>500){
		if(NL==PrevNL&& RPMB>20){
			RPMB = 0;

		}
		if(NR==PrevNR&& RPMB>20){
			RPMA = 0;
		}

		PrevNL=NL;
		PrevNR=NR;
		PrevChckStop=currentMillis;

		}

		if (SetRPM_B == 0) {
			motorB_RUN(0);
			TIM2->CNT = 20000;
			RPMB = 0;
		}

		if (SetRPM_A == 0) {
			motorA_RUN(0);
			TIM4->CNT = 20000;
			RPMA = 0;
		}

		if ((tencountR >= 10) || (tencountL >= 10)) {

			NR = NR + (tencountR / 10);
			NL = NL + (tencountL / 10);
			tencountL = 0;
			tencountR = 0;
		}

		if (DirA == 1) {
			DirMutA = 1;
		}
		if (DirB == 1) {
			DirMutB = 1;
		}
		if (DirA == 2) {
			DirMutA = -1;
		}
		if (DirB == 2) {
			DirMutB = -1;
		}



		if ((currentMillis - PrevPID) >= 50 && NewCmd == 0&& change==1) {

			PWM_A = PWM_A + PID_A(RPMA);
			PWM_B = PWM_B + PID_B(RPMB);

			motorA_RUN(PWM_A);
			motorB_RUN(PWM_B);
			PrevPID = currentMillis;

			if (ReadyToSend == 1) {
				sprintf(send, "%d,%d,%d,%d \n", RPMA, RPMB,((int) NR * DirMutA), ((int) NL * DirMutB));
				HAL_UART_Transmit_IT(&huart1, (uint8_t*) send, strlen(send));
				ReadyToSend = 0;
			}


		}

		if (NewCmd == 1) {
			NewCmd = 0;
			if (DirA == 0) {
				NR = 0;
				tencountR = 0;
			}
			if (DirB == 0) {
				NL = 0;
				tencountL = 0;
			}

		}

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 3;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 3;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 3;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
	while (1) {
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
