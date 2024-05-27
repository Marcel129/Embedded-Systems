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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "robot.h"
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

extern robot mRobot;
bool timeToUpdateStates = true;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == mRobot.IRQ_timer)
	{
		timeToUpdateStates = true;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == mRobot.mSonar->echoPin) {
		if(HAL_GPIO_ReadPin(mRobot.mSonar->echoPort, mRobot.mSonar->echoPin) == GPIO_PIN_SET && mRobot.mSonar->isMeasureReady == 0){
			mRobot.mSonar->prevTime_us = 0;
			mRobot.timeBaseTimer_1us->Instance->CNT = 0;
		}
		else{
			mRobot.mSonar->currentResult_us = mRobot.timeBaseTimer_1us->Instance->CNT - mRobot.mSonar->prevTime_us;
			mRobot.mSonar->distance_m = (((float)mRobot.mSonar->currentResult_us/1000000.0) * 340.0)/2.0;
			mRobot.mSonar->isMeasureReady = 1;
		}
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
  MX_TIM7_Init();
  MX_UART5_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

	robotWheel rw, lw;
	RGB_led led;
	sonar son;
	temperatureSensor ts;
	mRobot.leftWheel = &lw;
	mRobot.rightWheel = &rw;
	mRobot.mLED = &led;
	mRobot.mSonar = &son;
	mRobot.mtempSens = &ts;

	__robot_init(&htim1, &htim2, &htim5, &htim7, &htim8, &huart5, &hadc1);

	uint32_t motorTime, ledTime, sonarTime, currTime, tempSensTime;
	motorTime = ledTime = currTime = sonarTime = tempSensTime = HAL_GetTick();

	uint8_t motorState = 0, ledState = 0;
	__robot_Move(BACWARD, 99, 99);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

		currTime = HAL_GetTick();

		if(timeToUpdateStates){
			timeToUpdateStates = false;
			__robot_update_led_light();
			__measureTemperature();
			__robot_update_wheel_speed(mRobot.leftWheel);
			__robot_update_wheel_speed(mRobot.rightWheel);
		}

//		if(currTime - motorTime >= 3000 && motorState == 0){
//			__robot_Move(FORWARD, -99, -99);
//			motorState = 1;
//			motorTime = currTime;
//		}
//
//		if(currTime - motorTime >= 3000 && motorState == 1){
//			__robot_Move(BACWARD, 99, 99);
//			motorState = 0;
//			motorTime = currTime;
//		}

//		if(currTime - ledTime >= 2000 && ledState == 0){
//			__robot_set_led_light(99,0,0);
//			ledState = 1;
//			ledTime = currTime;
//		}
//
//		if(currTime - ledTime >= 2000 && ledState == 1){
//			__robot_set_led_light(0,99,0);
//			ledState = 2;
//			ledTime = currTime;
//		}
//
//		if(currTime - ledTime >= 2000 && ledState == 2){
//			__robot_set_led_light(0,0,99);
//			ledState = 0;
//			ledTime = currTime;
//		}

		if(currTime - sonarTime >= 100){
			sonarTime = currTime;
			__startDistanceMeasure();
		}

		if(mRobot.mSonar->isMeasureReady == 1){
			mRobot.mSonar->isMeasureReady = 0;
			printf("Current distance value: %f m\n", mRobot.mSonar->distance_m);

			if(mRobot.mSonar->distance_m < 0.3){//go left if an obstacle is detected
				__robot_Move(FORWARD, 0, 99);
			}
			else{//if not go ahead
				__robot_Move(FORWARD, 99, 99);
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
  RCC_OscInitStruct.PLL.PLLN = 64;
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
