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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

typedef enum {FORWARD, BACWARD, LEFT, RIGHT, STOP} moveDir;

typedef struct {
	TIM_HandleTypeDef * wheelPWMTimer;
	uint32_t wheelPWMChannel;
	int8_t currentSpeed_forPWM, settedSpeed_forPWM;
	int8_t wheelMaxSpeed_forPWM, wheelMinSpeed_forPWM;
	GPIO_TypeDef* dirPort;
	uint16_t dirPin;

} robotWheel;

typedef struct{
	TIM_HandleTypeDef * R_timer, *G_timer, *B_timer;
	uint32_t R_channel, G_channel, B_channel;
	uint8_t currentBrightness_R, settetBrightness_R;
	uint8_t currentBrightness_G, settetBrightness_G;
	uint8_t currentBrightness_B, settetBrightness_B;
	uint8_t brightnessChangeStep_R, brightnessChangeStep_G, brightnessChangeStep_B;
	uint8_t ledMaxBrightness, ledMinBrightness;
}RGB_led;

typedef struct {
	robotWheel *leftWheel, *rightWheel;
	RGB_led *mLED;
	int8_t acceleration_forPWM;
	moveDir mDir;

	TIM_HandleTypeDef * IRQ_timer;
} robot;

robot mRobot;

void __robot_init(){

	//assign perferials to the struct members
	mRobot.leftWheel->wheelPWMTimer = &htim8;
	mRobot.leftWheel->wheelPWMChannel = TIM_CHANNEL_1;
	mRobot.leftWheel->dirPort = MOTOR_L_EN_GPIO_Port;
	mRobot.leftWheel->dirPin = MOTOR_L_EN_Pin;
	mRobot.leftWheel->currentSpeed_forPWM = 0;
	mRobot.leftWheel->settedSpeed_forPWM = 0;
	mRobot.leftWheel->wheelMaxSpeed_forPWM = 99;
	mRobot.leftWheel->wheelMinSpeed_forPWM = -99;

	mRobot.rightWheel->wheelPWMTimer = &htim8;
	mRobot.rightWheel->wheelPWMChannel = TIM_CHANNEL_3;
	mRobot.rightWheel->dirPort = MOTOR_R_EN_GPIO_Port;
	mRobot.rightWheel->dirPin = MOTOR_R_EN_Pin;
	mRobot.rightWheel->currentSpeed_forPWM = 0;
	mRobot.rightWheel->settedSpeed_forPWM = 0;
	mRobot.rightWheel->wheelMaxSpeed_forPWM = 99;
	mRobot.rightWheel->wheelMinSpeed_forPWM = -99;

	mRobot.acceleration_forPWM = 1;
	mRobot.mDir = STOP;

	//default move forward
	HAL_GPIO_WritePin(mRobot.leftWheel->dirPort, mRobot.leftWheel->dirPin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(mRobot.rightWheel->dirPort, mRobot.rightWheel->dirPin, GPIO_PIN_RESET);

	//start PWM
	HAL_TIM_PWM_Start(mRobot.leftWheel->wheelPWMTimer,  mRobot.leftWheel->wheelPWMChannel);
	HAL_TIM_PWM_Start(mRobot.rightWheel->wheelPWMTimer,  mRobot.rightWheel->wheelPWMChannel);

	//reset speeds
	__HAL_TIM_SET_COMPARE(mRobot.leftWheel->wheelPWMTimer, mRobot.leftWheel->wheelPWMChannel, 0);
	__HAL_TIM_SET_COMPARE(mRobot.rightWheel->wheelPWMTimer, mRobot.rightWheel->wheelPWMChannel, 0);

	//assign perferials to struct members
	mRobot.mLED->R_timer = &htim2;
	mRobot.mLED->G_timer = &htim2;
	mRobot.mLED->B_timer = &htim2;
	mRobot.mLED->R_channel = TIM_CHANNEL_2;
	mRobot.mLED->G_channel = TIM_CHANNEL_3;
	mRobot.mLED->B_channel = TIM_CHANNEL_4;

	mRobot.mLED->currentBrightness_R = 0;
	mRobot.mLED->currentBrightness_G = 0;
	mRobot.mLED->currentBrightness_B = 0;
	mRobot.mLED->settetBrightness_R = 0;
	mRobot.mLED->settetBrightness_G = 0;
	mRobot.mLED->settetBrightness_B = 0;
	mRobot.mLED->brightnessChangeStep_R = 1;
	mRobot.mLED->brightnessChangeStep_G = 1;
	mRobot.mLED->brightnessChangeStep_B = 1;

	mRobot.mLED->ledMaxBrightness = 99;
	mRobot.mLED->ledMinBrightness = 0;

	//start timers
	HAL_TIM_PWM_Start(mRobot.mLED->R_timer,  mRobot.mLED->R_channel);
	HAL_TIM_PWM_Start(mRobot.mLED->G_timer,  mRobot.mLED->G_channel);
	HAL_TIM_PWM_Start(mRobot.mLED->B_timer,  mRobot.mLED->B_channel);

	//reset brightness
	__HAL_TIM_SET_COMPARE(mRobot.mLED->R_timer, mRobot.mLED->R_channel, 0);
	__HAL_TIM_SET_COMPARE(mRobot.mLED->G_timer, mRobot.mLED->G_channel, 0);
	__HAL_TIM_SET_COMPARE(mRobot.mLED->B_timer, mRobot.mLED->B_channel, 0);

	mRobot.IRQ_timer = &htim2;
	//start interrupt timer
	HAL_TIM_Base_Start_IT(mRobot.IRQ_timer);

}

void __robot_update_wheel_speed(robotWheel *wheel){
	if(mRobot.mDir == STOP){
		wheel->settedSpeed_forPWM = 0;
	}

	if(wheel->currentSpeed_forPWM > wheel->settedSpeed_forPWM){
		if(wheel->currentSpeed_forPWM - mRobot.acceleration_forPWM < wheel->wheelMinSpeed_forPWM){
			wheel->currentSpeed_forPWM = wheel->wheelMinSpeed_forPWM;
		}
		else{
			wheel->currentSpeed_forPWM -= mRobot.acceleration_forPWM;
		}
	}
	else if(wheel->currentSpeed_forPWM < wheel->settedSpeed_forPWM){
		if(wheel->currentSpeed_forPWM + mRobot.acceleration_forPWM > wheel->wheelMaxSpeed_forPWM){
			wheel->currentSpeed_forPWM = wheel->wheelMaxSpeed_forPWM;
		}
		else{
			wheel->currentSpeed_forPWM += mRobot.acceleration_forPWM;
		}
	}

	if(wheel->currentSpeed_forPWM <= 0){
		HAL_GPIO_WritePin(wheel->dirPort, wheel->dirPin, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(wheel->dirPort, wheel->dirPin, GPIO_PIN_RESET);
	}

	__HAL_TIM_SET_COMPARE(wheel->wheelPWMTimer, wheel->wheelPWMChannel, abs(wheel->currentSpeed_forPWM));
}

void __robot_update_led_light(){

	//update brightness vaules in the structure
	if(mRobot.mLED->currentBrightness_R > mRobot.mLED->settetBrightness_R ){
		//check boundary conditions
		if(mRobot.mLED->currentBrightness_R - mRobot.mLED->brightnessChangeStep_R < mRobot.mLED->ledMinBrightness){
			mRobot.mLED->currentBrightness_R = mRobot.mLED->ledMinBrightness;
		}
		else{
			mRobot.mLED->currentBrightness_R -= mRobot.mLED->brightnessChangeStep_R;
		}
	}
	else if(mRobot.mLED->currentBrightness_R < mRobot.mLED->settetBrightness_R){
		if(mRobot.mLED->currentBrightness_R + mRobot.mLED->brightnessChangeStep_R > mRobot.mLED->ledMinBrightness){
			mRobot.mLED->currentBrightness_R = mRobot.mLED->ledMaxBrightness;
		}
		else{
			mRobot.mLED->currentBrightness_R += mRobot.mLED->brightnessChangeStep_R;
		}
	}


	if(mRobot.mLED->currentBrightness_G > mRobot.mLED->settetBrightness_G ){
		if(mRobot.mLED->currentBrightness_G - mRobot.mLED->brightnessChangeStep_G < mRobot.mLED->ledMinBrightness){
			mRobot.mLED->currentBrightness_G = mRobot.mLED->ledMinBrightness;
		}
		else{
			mRobot.mLED->currentBrightness_G -= mRobot.mLED->brightnessChangeStep_G;
		}
	}
	else if(mRobot.mLED->currentBrightness_G < mRobot.mLED->settetBrightness_G){
		if(mRobot.mLED->currentBrightness_G + mRobot.mLED->brightnessChangeStep_G > mRobot.mLED->ledMinBrightness){
			mRobot.mLED->currentBrightness_G = mRobot.mLED->ledMaxBrightness;
		}
		else{
			mRobot.mLED->currentBrightness_G += mRobot.mLED->brightnessChangeStep_G;
		}
	}


	if(mRobot.mLED->currentBrightness_B > mRobot.mLED->settetBrightness_B ){
		if(mRobot.mLED->currentBrightness_B - mRobot.mLED->brightnessChangeStep_B < mRobot.mLED->ledMinBrightness){
			mRobot.mLED->currentBrightness_B = mRobot.mLED->ledMinBrightness;
		}
		else{
			mRobot.mLED->currentBrightness_B -= mRobot.mLED->brightnessChangeStep_B;
		}
	}
	else if(mRobot.mLED->currentBrightness_B < mRobot.mLED->settetBrightness_B){
		if(mRobot.mLED->currentBrightness_B + mRobot.mLED->brightnessChangeStep_B > mRobot.mLED->ledMinBrightness){
			mRobot.mLED->currentBrightness_B = mRobot.mLED->ledMaxBrightness;
		}
		else{
			mRobot.mLED->currentBrightness_B += mRobot.mLED->brightnessChangeStep_B;
		}
	}

	//set updated values in the timer
	__HAL_TIM_SET_COMPARE(mRobot.mLED->R_timer, mRobot.mLED->R_channel, mRobot.mLED->currentBrightness_B);
	__HAL_TIM_SET_COMPARE(mRobot.mLED->G_timer, mRobot.mLED->G_channel, mRobot.mLED->currentBrightness_G);
	__HAL_TIM_SET_COMPARE(mRobot.mLED->B_timer, mRobot.mLED->B_channel, mRobot.mLED->currentBrightness_B);
}

void __robot_Move(moveDir md, int8_t L_power, int8_t R_power){

	switch(md){
	case FORWARD:
		mRobot.mDir = FORWARD;
		break;
	case BACWARD:
		mRobot.mDir = BACWARD;
		break;
	case LEFT:
		mRobot.mDir = LEFT;
		break;
	case RIGHT:
		mRobot.mDir = RIGHT;
		break;
	case STOP:
		mRobot.mDir = STOP;
		HAL_GPIO_WritePin(mRobot.leftWheel->dirPort, mRobot.leftWheel->dirPin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mRobot.rightWheel->dirPort, mRobot.rightWheel->dirPin, GPIO_PIN_RESET);
		mRobot.leftWheel->settedSpeed_forPWM = 0;
		mRobot.rightWheel->settedSpeed_forPWM = 0;
		return;
	}

	mRobot.leftWheel->settedSpeed_forPWM = L_power;
	mRobot.rightWheel->settedSpeed_forPWM = R_power;
}

void __robot_set_led_light(uint8_t R, uint8_t G, uint8_t B){
	mRobot.mLED->settetBrightness_R = R;
	mRobot.mLED->settetBrightness_G = G;
	mRobot.mLED->settetBrightness_B = B;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == mRobot.IRQ_timer)
  {
	  __robot_update_led_light();
	  __robot_update_wheel_speed(mRobot.leftWheel);
	  __robot_update_wheel_speed(mRobot.rightWheel);
  }
}


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
  MX_TIM8_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

	robotWheel rw, lw;
	RGB_led led;
	mRobot.leftWheel = &lw;
	mRobot.rightWheel = &rw;
	mRobot.mLED = &led;

	__robot_init();

	uint32_t motorTime, ledTime, currTime;
	motorTime = ledTime = currTime = HAL_GetTick();

	uint8_t motorState = 0, ledState = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		currTime = HAL_GetTick();

		if(currTime - motorTime >= 3000 && motorState == 0){
			__robot_Move(FORWARD, 99, 99);
			motorState = 1;
			motorTime = currTime;
		}

		if(currTime - motorTime >= 3000 && motorState == 1){
			__robot_Move(BACWARD, -99, -99);
			motorState = 0;
			motorTime = currTime;
		}

		if(currTime - ledTime >= 2000 && ledState == 0){
			__robot_set_led_light(99,0,0);
			ledState = 1;
			ledTime = currTime;
		}

		if(currTime - ledTime >= 2000 && ledState == 1){
			__robot_set_led_light(0,99,0);
			ledState = 2;
			ledTime = currTime;
		}

		if(currTime - ledTime >= 2000 && ledState == 2){
			__robot_set_led_light(0,0,99);
			ledState = 0;
			ledTime = currTime;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 159;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
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
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 9;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 159;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 99;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MOTOR_R_PH_Pin|MOTOR_L_PH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOTOR_R_PH_Pin MOTOR_L_PH_Pin */
  GPIO_InitStruct.Pin = MOTOR_R_PH_Pin|MOTOR_L_PH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
