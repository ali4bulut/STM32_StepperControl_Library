/*
 * Stepper.h
 *
 *  Created on: Sep 21, 2021
 *      Author: alibl
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

#include "stm32f4xx_hal.h"

#define PreScaledClockSpeed 	1000000


#define STEPPER_DIR_CW 		0
#define STEPPER_DIR_CC 		1
#define STEPPER_DIR_STOP	2

#define STEPPER_CTRL_NONE	0
#define STEPPER_CTRL_SPD	1
#define STEPPER_CTRL_POSE	2

#define STEPPER_DONE		0
#define STEPPER_MOVING		1

#define STEPPER_OUT_OFF		0
#define STEPPER_OUT_ON		1

typedef struct {

	TIM_HandleTypeDef *tim;

	GPIO_TypeDef *dirPin_port;
	uint16_t dirPin;

	uint8_t direction;
	uint32_t Stepper_Speed;

	int32_t counter;
	int32_t moveFrom;
	uint32_t goal;
	uint8_t control;
	uint8_t status;

	uint8_t output;

	uint32_t remainPulses;
	uint32_t per20;
	uint32_t per80;
	uint32_t accelBias;
	uint32_t arrVal;

} Stepper_HandleTypeDef;

void Stepper_Init(Stepper_HandleTypeDef *stepper);

void Stepper_Output_Control(Stepper_HandleTypeDef* stepper);
void Stepper_Set_Speed(Stepper_HandleTypeDef *stepper, int16_t speed);
void Stepper_Pose(Stepper_HandleTypeDef *stepper);

void Stepper_Goto(Stepper_HandleTypeDef *stepper, uint32_t steps, uint8_t dir);
void Stepper_Control_Speed(Stepper_HandleTypeDef *stepper, int16_t speed, uint8_t dir);
void Stepper_Stop(Stepper_HandleTypeDef *stepper);

void Stepper_Accel(Stepper_HandleTypeDef *stepper);

void Stepper_IRQ_Handler(Stepper_HandleTypeDef *stepper);

#endif /* INC_STEPPER_H_ */
