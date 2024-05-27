/*
 * robot.h
 *
 *  Created on: May 27, 2024
 *      Author: Marcel
 */

#ifndef INC_ROBOT_H_
#define INC_ROBOT_H_

#include "stdio.h"//for printf()
#include "stdlib.h"//for abs()
#include "stdbool.h"

#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

typedef enum {FORWARD, BACWARD, LEFT, RIGHT, STOP} moveDir;
typedef enum {LEFT_WHEEL, RIGHT_WHEEL} wheelName;

typedef struct {
	TIM_HandleTypeDef * wheelPWMTimer;
	uint32_t wheelPWMChannel;
	int8_t currentSpeed_forPWM, settedSpeed_forPWM;
	int8_t wheelMaxSpeed_forPWM, wheelMinSpeed_forPWM;
	GPIO_TypeDef* dirPort;
	uint16_t dirPin;
	wheelName name;
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

typedef struct{
	uint32_t currentResult_us, prevTime_us;
	uint8_t isMeasureReady;
	GPIO_TypeDef* triggerPort, *echoPort, *enablePort;
	uint16_t triggerPin, echoPin, enablePin;
	float distance_m;
}sonar;

typedef struct{
	ADC_HandleTypeDef * adc_handler;
	uint32_t result;
	float temperature, V_sense, V_25, AvgSlope, V_ref;
}temperatureSensor;

typedef struct {
	robotWheel *leftWheel, *rightWheel;
	RGB_led *mLED;
	sonar *mSonar;
	temperatureSensor *mtempSens;
	int8_t acceleration_forPWM;
	moveDir mDir;

	TIM_HandleTypeDef * IRQ_timer;
	TIM_HandleTypeDef * timeBaseTimer_1us;
	UART_HandleTypeDef * COM_UART_handler;
} robot;


void __robot_init(TIM_HandleTypeDef * xxx,
		TIM_HandleTypeDef * ledTim,
		TIM_HandleTypeDef * tim_1us,
		TIM_HandleTypeDef * stdIRQTim,
		TIM_HandleTypeDef * wheelTim,
		UART_HandleTypeDef * uartHandler,
		ADC_HandleTypeDef * innerTempSensADCHandler);

robot mRobot;

void __measureTemperature();
void __robot_update_wheel_speed(robotWheel *wheel);
void __robot_update_led_light();
void __robot_Move(moveDir md, int8_t R_power, int8_t L_power);
void __robot_set_led_light(uint8_t R, uint8_t G, uint8_t B);
void __startDistanceMeasure();

#endif /* INC_ROBOT_H_ */
