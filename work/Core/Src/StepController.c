/*
 * StepController.c
 *
 *  Created on: 2023. 3. 20.
 *      Author: dlals
 */

#include "StepController.h"
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim1;

void stepStart(int which) {

	switch (which) {
	//salt
	case 1:
	   	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

	      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

	      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);


	      break;

		//sugar
	case 2:
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
		//black

	}
}

void stepStop(int which){
	switch (which) {
	//salt
	case 1:
	   	  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);

	      HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);

	      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);

	      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);


	      break;

		//sugar
	case 2:
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);

		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);

		HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);

		HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
		//black
	}
}
