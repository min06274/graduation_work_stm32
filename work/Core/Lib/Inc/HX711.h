#ifndef __HX711__H__
#define __HX711__H__
#include "main.h"
//extern void Init_Hx711();
extern uint32_t HX711_Read(void);
extern int32_t Get_Weight();
extern float Get_Weight_f();
extern void Get_Maopi();
void delay_us (uint16_t us);

#endif
