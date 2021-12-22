/*
 * Robot.h
 *
 *  Created on: Oct 13, 2021
 *      Author: alibl
 */

#ifndef INC_ROBOT_H_
#define INC_ROBOT_H_

#include "stm32f4xx_hal.h"
#include "Stepper.h"
#include <math.h>

#define OneRevMove 		0.2136283004441059402f
#define OneRevStepNum 	800.f

#define HZ_TO_MMS(X)		(float)( X *((float)OneRevMove / (float)OneRevStepNum))
#define MMS_TO_HZ(X)		(float)( X *((float)OneRevStepNum / (float)OneRevMove))

#define DEG_TO_RAD		(float)(M_PI / 180.f)
#define RAD_TO_DEG		(float)(180.f / M_PI)

extern float v, vr, vl, vx, vy;

typedef struct {
	Stepper_HandleTypeDef *rightW;
	Stepper_HandleTypeDef *leftW;

	float angle;
	float x;
	float y;

	float goal_X;
	float goal_Y;

	float TyreDem;
	float SinglePulseDst;
	float Lenght;
	uint32_t revStepNum;

	uint32_t prevPoint;

	uint8_t Done;
	uint8_t Control;
	float l;

} Robot_HandleTypedef;

void Robot_Init(Robot_HandleTypedef *robot);
void Robot_Set_Speed(Robot_HandleTypedef *robot, uint16_t speed);
float mm2Pulse(Robot_HandleTypedef *robot, float dst);
float angle2mm(Robot_HandleTypedef *robot, float angle);

void go_Pos(Robot_HandleTypedef *robot, int32_t x, int32_t y, float angle);
void go_Pos_2(Robot_HandleTypedef *robot, int32_t x, int32_t y, float angle);

void Robot_IRQ_Handler(Robot_HandleTypedef *robot);

#endif /* INC_ROBOT_H_ */
