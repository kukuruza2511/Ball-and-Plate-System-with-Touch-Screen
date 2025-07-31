/*
 * Motor_Control.c
 *
 *  Created on: Jul 31, 2025
 *      Author: ngovi
 */

#include "Motor_Control.h"

void Motor_Init(Motor_t* new_Motor, GPIO_TypeDef* GPIO_IN1, uint16_t IN1, GPIO_TypeDef* GPIO_IN2, uint16_t IN2, TIM_HandleTypeDef* htim_pwm,
		uint32_t Channel_PWM, TIM_HandleTypeDef* htim_encoder){
	new_Motor->GPIO_IN1 = GPIO_IN1;
	new_Motor->GPIO_IN2 = GPIO_IN2;
	new_Motor->IN1 = IN1;
	new_Motor->IN2 = IN2;
	new_Motor->htim_pwm = htim_pwm;
	new_Motor->Channel_PWM = Channel_PWM;
	new_Motor->htim_encoder = htim_encoder;
}

void Stop(Motor_t *Motor){
	HAL_GPIO_WritePin(Motor->GPIO_IN1, Motor->IN1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor->GPIO_IN2, Motor->IN2, GPIO_PIN_RESET);
}

void Turn_right(Motor_t *Motor){
	HAL_GPIO_WritePin(Motor->GPIO_IN1, Motor->IN1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor->GPIO_IN2, Motor->IN2, GPIO_PIN_SET);
}

void Turn_left(Motor_t *Motor){
	HAL_GPIO_WritePin(Motor->GPIO_IN1, Motor->IN1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor->GPIO_IN2, Motor->IN2, GPIO_PIN_RESET);
}

void Control_Motor(Motor_t* Motor, float control_signal){
	if(control_signal >= 0){
		Turn_left(Motor);
	}else{
		Turn_right(Motor);
	}
	__HAL_TIM_SET_COMPARE(Motor->htim_pwm, Motor->Channel_PWM, control_signal);
}
