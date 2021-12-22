/*
 * Robot.c
 *
 *  Created on: Oct 13, 2021
 *      Author: alibl
 */

#include "Robot.h"

float v = 0, vr = 0, vl = 0, vx = 0, vy = 0;

float Q_rsqrt(float number);

void Robot_Init(Robot_HandleTypedef *robot) {

	if (!robot->revStepNum)
		robot->revStepNum = 800;
	if (!robot->TyreDem)
		robot->TyreDem = 34;
	if (!robot->Lenght)
		robot->Lenght = 215.0f;

	robot->x = 0;
	robot->y = 0;
	robot->angle = 0.0f;
	robot->prevPoint = 0;
	robot->Done = 0;
	robot->Control = 0;

	robot->SinglePulseDst = ((robot->TyreDem * 2 * M_PI) / (float) robot->revStepNum);
}

void Robot_Set_Speed(Robot_HandleTypedef *robot, uint16_t speed) {
	Stepper_Set_Speed(robot->leftW, speed);
	Stepper_Set_Speed(robot->rightW, speed);
}

float mm2Pulse(Robot_HandleTypedef *robot, float dst) {
	if (dst < 0)
		dst *= -1;
	return (uint32_t) ((dst / robot->SinglePulseDst) + 0.5f);

}

//return mm
float angle2mm(Robot_HandleTypedef *robot, float angle) {
	if (angle < 0)
		angle *= -1;
	return (((2.0f * M_PI * robot->Lenght * angle) / 360.0f) + 0.5f);
}
void go_Pos(Robot_HandleTypedef *robot, int32_t x, int32_t y, float angle) {

	uint8_t dir = 0;
	float turn = 0;

	float line = Q_rsqrt((x - robot->x) * (x - robot->x) + (y - robot->y) * (y - robot->y));
	float ang = atan2f((y - robot->y), (x - robot->x));
	ang *= RAD_TO_DEG;

	turn = ang - robot->angle;

	if (turn < 0)
		dir = 0;
	else
		dir = 1;

	turn = (uint32_t) (angle2mm(robot, turn) / 2.0f);
	turn = (uint32_t) mm2Pulse(robot, turn);

	if (dir) {
		Stepper_Goto(robot->rightW, turn, STEPPER_DIR_CW);
		Stepper_Goto(robot->leftW, turn, STEPPER_DIR_CC);
	}
	else {
		Stepper_Goto(robot->rightW, turn, STEPPER_DIR_CC);
		Stepper_Goto(robot->leftW, turn, STEPPER_DIR_CW);
	}

	while (!(robot->rightW->status == STEPPER_DONE));
	while (!(robot->leftW->status == STEPPER_DONE));
	HAL_Delay(500);

	line = (uint32_t) mm2Pulse(robot, line);

	Stepper_Goto(robot->rightW, line, STEPPER_DIR_CW);
	Stepper_Goto(robot->leftW, line, STEPPER_DIR_CW);

	while (!(robot->rightW->status == STEPPER_DONE));
	while (!(robot->leftW->status == STEPPER_DONE));

	robot->x = x;
	robot->y = y;
	HAL_Delay(500);

	turn = angle - robot->angle;

	if (turn < 0)
		dir = 0;
	else
		dir = 1;

	turn = (uint32_t) (angle2mm(robot, turn) / 2.0f);
	turn = (uint32_t) mm2Pulse(robot, turn);

	if (dir) {
		Stepper_Goto(robot->rightW, turn, STEPPER_DIR_CW);
		Stepper_Goto(robot->leftW, turn, STEPPER_DIR_CC);
	}
	else {
		Stepper_Goto(robot->rightW, turn, STEPPER_DIR_CC);
		Stepper_Goto(robot->leftW, turn, STEPPER_DIR_CW);
	}

	while (!(robot->rightW->status == STEPPER_DONE));
	while (!(robot->leftW->status == STEPPER_DONE));
	HAL_Delay(500);

}

void go_Pos_2(Robot_HandleTypedef *robot, int32_t x, int32_t y, float angle) {

	if (robot->rightW->status != STEPPER_MOVING || robot->leftW->status != STEPPER_MOVING) {
		robot->goal_X = x;
		robot->goal_Y = y;
		robot->Control = 1;
		robot->Done = 0;

//		robot->rightW->direction = STEPPER_DIR_CW;
//		robot->leftW->direction = STEPPER_DIR_CW;
//
//		robot->rightW->control = STEPPER_CTRL_SPD;
//		robot->leftW->control = STEPPER_CTRL_SPD;
//
//		robot->rightW->status = STEPPER_MOVING;
//		robot->leftW->status = STEPPER_MOVING;
//		robot->rightW->output = STEPPER_OUT_ON;
//		robot->leftW->output = STEPPER_OUT_ON;
	}

}

//TODO: go_Pos_2 not work properly

void Robot_IRQ_Handler(Robot_HandleTypedef *robot) {
	static uint32_t timer = 0;
	static float line = 0, ang = 0, dt = 0, turn = 0;

	if (robot->Control) {
		dt = (float) (HAL_GetTick() - timer) / 1000.f;

		line = Q_rsqrt((robot->goal_X - robot->x) * (robot->goal_X - robot->x) + (robot->goal_Y - robot->y) * (robot->goal_Y - robot->y));
		ang = atan2f((robot->goal_Y - robot->y), (robot->goal_X - robot->x));
		turn = (ang - robot->angle) * 1.75f;

		if (!robot->Done) {

			if (line <= 1.f) {
				Stepper_Stop(robot->rightW);
				Stepper_Stop(robot->leftW);
				v = 0;
				turn = 0;
				robot->Done = 1;
				robot->Control = 0;
			}
			else {
				v = HZ_TO_MMS(400);
				vr = (((2.f * v) + (turn * robot->Lenght * 0.001f)) / 2.f);
				vl = (((2.f * v) - (turn * robot->Lenght * 0.001f)) / 2.f);

				robot->angle += ((vr - vl) / robot->Lenght * dt * 1000);
				if (robot->angle > 2 * M_PI || robot->angle < -2 * M_PI)
					robot->angle = 0.f;

				vx = (((vr + vl) / 2.f) * cosf((robot->angle)));
				vy = (((vr + vl) / 2.f) * sinf((robot->angle)));

				robot->x += vx * dt * 1000;
				robot->y += vy * dt * 1000;

				Stepper_Control_Speed(robot->rightW, (int16_t) MMS_TO_HZ(vr), 0);
				Stepper_Control_Speed(robot->leftW, (int16_t) MMS_TO_HZ(vl), 0);
			}

		}
	}
	timer = HAL_GetTick();
}

float Q_rsqrt(float number) {
	long i;
	float x2, y;
	const float threehalfs = 1.5F;

	x2 = number * 0.5F;
	y = number;
	i = *(long*) &y;                       // evil floating point bit level hacking
	i = 0x5f3759df - (i >> 1);               // what the fuck?
	y = *(float*) &i;
	y = y * (threehalfs - (x2 * y * y));   // 1st iteration
//	y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

	return 1 / y;
}

