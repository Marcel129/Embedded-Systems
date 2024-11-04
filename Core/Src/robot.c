/*
 * robot.c
 *
 *  Created on: May 27, 2024
 *      Author: Marcel
 */

#include "robot.h"

robot mRobot;

int __io_putchar(int ch)
{
	HAL_UART_Transmit(mRobot.COM_UART_handler, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
	return 1;
}

void __robot_init(TIM_HandleTypeDef * xxx,
		TIM_HandleTypeDef * ledTim,
		TIM_HandleTypeDef * tim_1us,
		TIM_HandleTypeDef * stdIRQTim,
		TIM_HandleTypeDef * wheelTim,
		UART_HandleTypeDef * uartHandler,
		ADC_HandleTypeDef * innerTempSensADCHandler){

	//assign perferials to the struct members
	mRobot.leftWheel->wheelPWMTimer = wheelTim;
	mRobot.leftWheel->wheelPWMChannel = TIM_CHANNEL_1;
	mRobot.leftWheel->dirPort = MOTOR_R_PH_GPIO_Port;
	mRobot.leftWheel->dirPin = MOTOR_R_PH_Pin;
	mRobot.leftWheel->currentSpeed_forPWM = 0;
	mRobot.leftWheel->settedSpeed_forPWM = 0;
	mRobot.leftWheel->wheelMaxSpeed_forPWM = 99;
	mRobot.leftWheel->wheelMinSpeed_forPWM = -99;
	mRobot.leftWheel->name = LEFT_WHEEL;

	mRobot.rightWheel->wheelPWMTimer = wheelTim;
	mRobot.rightWheel->wheelPWMChannel = TIM_CHANNEL_3;
	mRobot.rightWheel->dirPort = MOTOR_L_PH_GPIO_Port;
	mRobot.rightWheel->dirPin = MOTOR_L_PH_Pin;
	mRobot.rightWheel->currentSpeed_forPWM = 0;
	mRobot.rightWheel->settedSpeed_forPWM = 0;
	mRobot.rightWheel->wheelMaxSpeed_forPWM = 99;
	mRobot.rightWheel->wheelMinSpeed_forPWM = -99;
	mRobot.rightWheel->name = RIGHT_WHEEL;

	mRobot.acceleration_forPWM = 1;
	mRobot.mDir = STOP;

	HAL_GPIO_WritePin(mRobot.leftWheel->dirPort, mRobot.leftWheel->dirPin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(mRobot.rightWheel->dirPort, mRobot.rightWheel->dirPin, GPIO_PIN_RESET);

	//start PWM
	HAL_TIM_PWM_Start(mRobot.leftWheel->wheelPWMTimer,  mRobot.leftWheel->wheelPWMChannel);
	HAL_TIM_PWM_Start(mRobot.rightWheel->wheelPWMTimer,  mRobot.rightWheel->wheelPWMChannel);

	//reset speeds
	__HAL_TIM_SET_COMPARE(mRobot.leftWheel->wheelPWMTimer, mRobot.leftWheel->wheelPWMChannel, 0);
	__HAL_TIM_SET_COMPARE(mRobot.rightWheel->wheelPWMTimer, mRobot.rightWheel->wheelPWMChannel, 0);

	//assign perferials to the struct members
	mRobot.mLED->R_timer = ledTim;
	mRobot.mLED->G_timer = ledTim;
	mRobot.mLED->B_timer = ledTim;
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

	//assign perferials to the struct members
	mRobot.mSonar->echoPort = SONAR_ECHO_GPIO_Port;
	mRobot.mSonar->enablePort = SONAR_ENABLE_GPIO_Port;
	mRobot.mSonar->triggerPort = SONAR_TRIGGER_GPIO_Port;
	mRobot.mSonar->echoPin = SONAR_ECHO_Pin;
	mRobot.mSonar->enablePin = SONAR_ENABLE_Pin;
	mRobot.mSonar->triggerPin = SONAR_TRIGGER_Pin;

	HAL_GPIO_WritePin(mRobot.mSonar->enablePort, mRobot.mSonar->enablePin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(mRobot.mSonar->triggerPort, mRobot.mSonar->triggerPin, GPIO_PIN_RESET);

	mRobot.mSonar->currentResult_us = 0;
	mRobot.mSonar->prevTime_us = 0;
	mRobot.mSonar->isMeasureReady = 0;
	mRobot.mSonar->distance_m = 0;

	//assign uart handler for communication with PC
	mRobot.COM_UART_handler = uartHandler;

	//assign and start the interrupt timer
	//this timer is used to generate interrupt for periodicly update state of the robot
	mRobot.IRQ_timer = stdIRQTim;
	HAL_TIM_Base_Start_IT(mRobot.IRQ_timer);

	//assign and start the time base timer
	//this timer is required to precisely measure the proximity using sonar
	mRobot.timeBaseTimer_1us = tim_1us;
	HAL_TIM_Base_Start(mRobot.timeBaseTimer_1us);

	mRobot.mtempSens->adc_handler = &hadc1;
	mRobot.mtempSens->V_25 = 0.76;//V
	mRobot.mtempSens->V_ref = 3.3;
	mRobot.mtempSens->AvgSlope = 0.0025;//V/C
	mRobot.mtempSens->result = 0;
	mRobot.mtempSens->V_sense = 0;
	mRobot.mtempSens->result = 0;

}

void __measureTemperature(){
	HAL_ADC_Start(mRobot.mtempSens->adc_handler);
	HAL_ADC_PollForConversion(mRobot.mtempSens->adc_handler, 100);
	mRobot.mtempSens->result = HAL_ADC_GetValue(mRobot.mtempSens->adc_handler);

	mRobot.mtempSens->V_sense = (float)mRobot.mtempSens->result*mRobot.mtempSens->V_ref/4096.0;//convert ADC value to voltage
	mRobot.mtempSens->temperature = ((mRobot.mtempSens->V_sense - mRobot.mtempSens->V_25)/mRobot.mtempSens->AvgSlope) + 25.0; //((V_sense - V_25) / AvgSlope) + 25
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
		if(wheel->name == LEFT_WHEEL){
			HAL_GPIO_WritePin(wheel->dirPort, wheel->dirPin, GPIO_PIN_SET);
		}
		else if(wheel->name == RIGHT_WHEEL){
			HAL_GPIO_WritePin(wheel->dirPort, wheel->dirPin, GPIO_PIN_RESET);
		}
	}
	else{
		if(wheel->name == LEFT_WHEEL){
			HAL_GPIO_WritePin(wheel->dirPort, wheel->dirPin, GPIO_PIN_RESET);
		}
		else if(wheel->name == RIGHT_WHEEL){
			HAL_GPIO_WritePin(wheel->dirPort, wheel->dirPin, GPIO_PIN_SET);

		}
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
		if(mRobot.mLED->currentBrightness_R + mRobot.mLED->brightnessChangeStep_R > mRobot.mLED->ledMaxBrightness){
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
		if(mRobot.mLED->currentBrightness_G + mRobot.mLED->brightnessChangeStep_G > mRobot.mLED->ledMaxBrightness){
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
		if(mRobot.mLED->currentBrightness_B + mRobot.mLED->brightnessChangeStep_B > mRobot.mLED->ledMaxBrightness){
			mRobot.mLED->currentBrightness_B = mRobot.mLED->ledMaxBrightness;
		}
		else{
			mRobot.mLED->currentBrightness_B += mRobot.mLED->brightnessChangeStep_B;
		}
	}

	//assign updated values to the timer
	__HAL_TIM_SET_COMPARE(mRobot.mLED->R_timer, mRobot.mLED->R_channel, mRobot.mLED->currentBrightness_R);
	__HAL_TIM_SET_COMPARE(mRobot.mLED->G_timer, mRobot.mLED->G_channel, mRobot.mLED->currentBrightness_G);
	__HAL_TIM_SET_COMPARE(mRobot.mLED->B_timer, mRobot.mLED->B_channel, mRobot.mLED->currentBrightness_B);
}

void __robot_Move(moveDir md, int8_t R_power, int8_t L_power){

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

void __startDistanceMeasure(){
	mRobot.mSonar->prevTime_us = 0;
	mRobot.timeBaseTimer_1us->Instance->CNT = 0;
	mRobot.mSonar->isMeasureReady = 0;

	HAL_GPIO_WritePin(mRobot.mSonar->triggerPort, mRobot.mSonar->triggerPin, GPIO_PIN_SET);
	while(mRobot.timeBaseTimer_1us->Instance->CNT - mRobot.mSonar->prevTime_us < 10);
	HAL_GPIO_WritePin(mRobot.mSonar->triggerPort, mRobot.mSonar->triggerPin, GPIO_PIN_RESET);

	mRobot.timeBaseTimer_1us->Instance->CNT = 0;
}
