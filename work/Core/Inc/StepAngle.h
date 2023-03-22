/*
 * StepAngle.h
 *
 *  Created on: 2023. 3. 12.
 *      Author: dlals
 */

#ifndef INC_STEPANGLE_H_
#define INC_STEPANGLE_H_

#include "main.h"


void six_step(int n);
void stepper_set_rpm(int rpm);
void stepper_half_drive(int step);
void stepper_step_angle (float angle, int direction, int rpm);
/*
void micro_delay(uint16_t us);
*/
#endif /* INC_STEPANGLE_H_ */
