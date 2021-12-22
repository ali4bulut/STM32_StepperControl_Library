/*
 * Stepper.c
 *
 *  Created on: Sep 21, 2021
 *      Author: alibl
 */

#include "Stepper.h"

//void TIM3_Init(Stepper_HandleTypeDef *stepper);

void Stepper_Init(Stepper_HandleTypeDef *stepper) {

	HAL_TIM_PWM_Start_IT(stepper->tim, TIM_CHANNEL_1);
	stepper->tim->Instance->CCER &= ~(1 << 0);

	stepper->control = STEPPER_CTRL_NONE;
	stepper->counter = 0;
	stepper->Stepper_Speed = 0;
	stepper->moveFrom = 0;
	stepper->goal = 0;
	stepper->status = STEPPER_DONE;
	stepper->direction = STEPPER_DIR_STOP;
	stepper->output = STEPPER_OUT_OFF;

}

void Stepper_Goto(Stepper_HandleTypeDef *stepper, uint32_t steps, uint8_t dir) {
	if (steps != 0) {
		stepper->control = STEPPER_CTRL_POSE;

		if (stepper->status != STEPPER_MOVING) {

			if (dir == STEPPER_DIR_CW) {
				stepper->direction = STEPPER_DIR_CW;
				HAL_GPIO_WritePin(stepper->dirPin_port, stepper->dirPin, GPIO_PIN_SET);
			}
			else if (dir == STEPPER_DIR_CC) {
				stepper->direction = STEPPER_DIR_CC;
				HAL_GPIO_WritePin(stepper->dirPin_port, stepper->dirPin, GPIO_PIN_RESET);
			}

			stepper->moveFrom = stepper->counter;
			stepper->goal = steps;

			stepper->remainPulses = steps;
			stepper->per20 = ((stepper->goal * 2) / 10);
			stepper->per80 = ((stepper->goal * 8) / 10);

			uint32_t temp = stepper->Stepper_Speed;
			stepper->accelBias = (uint32_t) ((temp / stepper->per20) + 0.5f);
			if (stepper->accelBias == 0)
				stepper->accelBias = 1;
			stepper->arrVal = stepper->Stepper_Speed * 2;

			stepper->output = STEPPER_OUT_ON;
			stepper->status = STEPPER_MOVING;

		}
	}
}

void Stepper_Control_Speed(Stepper_HandleTypeDef *stepper, int16_t speed, uint8_t dir) {
	stepper->control = STEPPER_CTRL_SPD;

	if (speed > 0) {
		stepper->direction = STEPPER_DIR_CW;
		HAL_GPIO_WritePin(stepper->dirPin_port, stepper->dirPin, GPIO_PIN_SET);
	}
	else if (speed < 0) {
		stepper->direction = STEPPER_DIR_CC;
		HAL_GPIO_WritePin(stepper->dirPin_port, stepper->dirPin, GPIO_PIN_RESET);
	}
	Stepper_Set_Speed(stepper, speed);
	stepper->output = STEPPER_OUT_ON;
	stepper->status = STEPPER_MOVING;

}

void Stepper_Pose(Stepper_HandleTypeDef *stepper) {

	if (stepper->control == STEPPER_CTRL_POSE) {
		if (stepper->status == STEPPER_MOVING) {

			if (stepper->moveFrom - stepper->counter == stepper->goal || stepper->moveFrom - stepper->counter == -stepper->goal) {
				stepper->output = STEPPER_OUT_OFF;
				stepper->direction = STEPPER_DIR_STOP;
				stepper->status = STEPPER_DONE;
			}
		}
	}
}

void Stepper_Set_Speed(Stepper_HandleTypeDef *stepper, int16_t speed) {
	static uint32_t t_speed = 0, duty = 0;

	if(speed < 0)
		speed*= -1;
//	if (speed < 165)
//		speed = 165;

	//if timer mode is center aligned mode div 2 if up mode remove div 2
	t_speed = PreScaledClockSpeed / speed / 2;
	duty = (t_speed / 100);

	if (duty < 2)
		duty = 2;

	stepper->tim->Instance->CCR1 = duty - 1;
	stepper->tim->Instance->ARR = t_speed - 1;

	stepper->Stepper_Speed = t_speed - 1;
}

void Stepper_Stop(Stepper_HandleTypeDef *stepper) {
	stepper->output = STEPPER_OUT_OFF;
	stepper->direction = STEPPER_DIR_STOP;
	stepper->status = STEPPER_DONE;
	stepper->control = STEPPER_CTRL_NONE;

}

void Stepper_Output_Control(Stepper_HandleTypeDef *stepper) {

	if (stepper->output == STEPPER_OUT_ON) {
		stepper->tim->Instance->CCER |= (1 << 0);
	}
	else if ((stepper->output == STEPPER_OUT_OFF) && (stepper->tim->Instance->CCER & 0x01)) {
		stepper->tim->Instance->CCER &= ~(1 << 0);
		stepper->tim->Instance->ARR = 65535;
	}

}

void Stepper_Accel(Stepper_HandleTypeDef *stepper) {

	if (stepper->remainPulses <= stepper->per20 + 1) {
		//decel
		stepper->arrVal += stepper->accelBias;
		if (stepper->arrVal >= stepper->Stepper_Speed * 2)
			stepper->arrVal = stepper->Stepper_Speed * 2;
		stepper->tim->Instance->ARR = stepper->arrVal;
	}
	else if (stepper->remainPulses > stepper->per20 + 1 && stepper->remainPulses <= stepper->per80 + 1) {
		//top speed
		stepper->tim->Instance->ARR = stepper->Stepper_Speed;
	}
	else {
		//accel
		stepper->arrVal -= stepper->accelBias;
		if (stepper->arrVal <= stepper->Stepper_Speed)
			stepper->arrVal = stepper->Stepper_Speed;
		stepper->tim->Instance->ARR = stepper->arrVal;
	}
}

void Stepper_IRQ_Handler(Stepper_HandleTypeDef *stepper) {
	Stepper_Pose(stepper);
	Stepper_Output_Control(stepper);
	if (stepper->control == STEPPER_CTRL_POSE) {
		if (stepper->direction != STEPPER_DIR_STOP) {
			Stepper_Accel(stepper);
			stepper->remainPulses--;
		}
	}
	if (stepper->direction == STEPPER_DIR_CW) {
		stepper->counter++;
	}
	else if (stepper->direction == STEPPER_DIR_CC) {
		stepper->counter--;
	}
}

///* TIM3 init function */
//void TIM3_Init(Stepper_HandleTypeDef *stepper)
//{
//
//  /* USER CODE BEGIN TIM3_Init 0 */
//
//  /* USER CODE END TIM3_Init 0 */
//
//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};
//
//  /* USER CODE BEGIN TIM3_Init 1 */
//
//  /* USER CODE END TIM3_Init 1 */
//  stepper->tim->Instance = TIM3;
//  stepper->tim->Init.Prescaler = 90-1;
//  stepper->tim->Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
//  stepper->tim->Init.Period = 65535;
//  stepper->tim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  stepper->tim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
//  if (HAL_TIM_Base_Init(stepper->tim) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(stepper->tim, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_Init(stepper->tim) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(stepper->tim, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 10-1;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
//  if (HAL_TIM_PWM_ConfigChannel(stepper->tim, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM3_Init 2 */
//
//  /* USER CODE END TIM3_Init 2 */
//  HAL_TIM_MspPostInit(stepper->tim);
//
//}

