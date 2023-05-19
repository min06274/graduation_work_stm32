/*
 * StepController.c
 *
 *  Created on: 2023. 3. 20.
 *      Author: dlals
 */

#include "StepController.h"

extern TIM_HandleTypeDef htim9;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern int step_flag;
void micro_delay(uint16_t us) {

	htim9.Instance->CNT = 0;
	while (htim9.Instance->CNT <= us);
}

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
		break;

	case 3:
		HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

		micro_delay(3418);
		HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);

		micro_delay(6836);

		HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);

		break;

		//black
		/*
	case 3:
		HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);

		micro_delay(6836);
		HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
		micro_delay(6836);

		HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);


		micro_delay(6836);

		HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);

		break;
		*/
	}
	step_flag = 1;
}

void stepStop(int which) {
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
		break;
		//black

	case 3:

		HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);

		HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);

		micro_delay(9765);

		HAL_TIM_PWM_Stop(&htim13, TIM_CHANNEL_1);

		micro_delay(19531);

		HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);


		break;


	}


}
