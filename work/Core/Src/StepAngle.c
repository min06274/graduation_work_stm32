#include "StepAngle.h"
#include "OledController.h"
/*
 * StepAngle.c
 *
 *  Created on: 2023. 3. 12.
 *      Author: dlals
 */

//first locate (default)
int locate = 1;

//using micro delay for step motor
extern TIM_HandleTypeDef htim4;


#define stepsperrev 4096

/*
void micro_delay(uint16_t us)
{
   __HAL_TIM_SET_COUNTER(&htim4,0);
   while(__HAL_TIM_GET_COUNTER(&htim4) < us);
}
*/
void stepper_set_rpm(int rpm)
{
   micro_delay(60000000/stepsperrev/rpm);
}

void stepper_half_drive(int step)
{
   switch(step){
   case 0:
      HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
      break;

   case 1:
      HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
      break;


   case 2:
      HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
      break;

   case 3:
      HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
      break;

   case 4:
      HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
      break;

   case 5:
      HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
      break;

   case 6:
      HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
      break;

   case 7:
      HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
      break;
   }
}

void stepper_step_angle (float angle, int direction, int rpm)
{
  float anglepersequence = 0.703125;  // 360 = 512 sequences
  int numberofsequences = (int) (angle/anglepersequence);

  for (int seq=0; seq<numberofsequences; seq++)
  {
    if (direction == 0)  // for anti- clockwise
    {
      for (int step=7; step>=0; step--)
      {
        stepper_half_drive(step);
        stepper_set_rpm(rpm);
      }

    }

    else if (direction == 1)  // for clockwise
    {
      for (int step=0; step<8; step++)
      {
        stepper_half_drive(step);
        stepper_set_rpm(rpm);
      }
    }
  }
}

void six_step(int n)
{
    printTemper(n);

   if(n != locate)
   {


   int temp =locate;
   int cnt1 = 0; //clock-wise

   int cnt2 = 0; //anti-clock-wise


   //clock
   for(int i = 1; i<=5; i++)
      {
         temp++;
         cnt1++;

         if(temp > 6)
         {
            temp = 1;
         }
         if(temp == n)
         {
            stepper_step_angle(60*cnt1,1,13);
            locate = n;

            return;
         }
         if(cnt1>=3)
         {

             break;
         }

      }

   //anti-clock
   temp = locate;
   for(int i = 1; i<=5; i++)
      {
         temp--;
         cnt2++;

         if(temp < 1)
         {
            temp = 6;
         }
         if(temp == n)
         {
            stepper_step_angle(60*cnt2,0,13);
            locate = n;
            return;
         }
         if(cnt2>=3)
         {
            stepper_step_angle(60*cnt2,0,13);
            locate = n;
            return;
         }

      }


   }


}
