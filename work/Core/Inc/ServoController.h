
#include "ServoController.h"
extern TIM_HandleTypeDef htim1;

void servoStart(int which)
{

	switch (which) {
		//salt
		case 1:

			  htim1.Instance->CCR1 = 140;

			break;
	}
}

void servoStop(int which)
{
	switch (which) {
		//salt
		case 1:

			  htim1.Instance->CCR1 = 0;

			break;
	}

}
