/*
 * Motor_Control.h
 *
 *  Created on: Jul 31, 2025
 *      Author: ngovi
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "stm32f4xx_hal.h"

typedef struct{
	GPIO_TypeDef* GPIO_IN1; uint16_t IN1;
	GPIO_TypeDef* GPIO_IN2; uint16_t IN2;
	TIM_HandleTypeDef* htim_pwm;
	uint32_t Channel_PWM;
	TIM_HandleTypeDef* htim_encoder;
}Motor_t;

void Motor_Init(Motor_t* new_Motor, GPIO_TypeDef* GPIO_IN1, uint16_t IN1, GPIO_TypeDef* GPIO_IN2, uint16_t IN2, TIM_HandleTypeDef* htim_pwm,
		uint32_t Channel_PWM, TIM_HandleTypeDef* htim_encoder);

void Turn_left(Motor_t *Motor);

void Turn_right(Motor_t *Motor);

void Stop(Motor_t *Motor);

void Control_Motor(Motor_t* Motor, float control_signal);

#endif /* INC_MOTOR_CONTROL_H_ */
