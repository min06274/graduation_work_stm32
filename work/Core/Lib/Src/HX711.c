#include "HX711.h"
#include "main.h"
int32_t HX711_Buffer = 0;
int32_t Weight_Maopi = 0,Weight_Shiwu = 0;
void delay_us (uint16_t us);
#define GapValue 473

//****************************************************
//HX711 초기화
//****************************************************
/*
void Init_Hx711()
{
	pinMode(HX711_SCK, OUTPUT);
	pinMode(HX711_DT, INPUT);
}
*/

//****************************************************
//펠트 무게 얻기
//****************************************************
void Get_Maopi()
{
	Weight_Maopi = HX711_Read();
}

//****************************************************
//무게
//****************************************************
int32_t Get_Weight()
{
	HX711_Buffer = HX711_Read();
	Weight_Shiwu = HX711_Buffer;
	Weight_Shiwu = Weight_Shiwu - Weight_Maopi;
	Weight_Shiwu = (int32_t)((float)Weight_Shiwu/GapValue);
	return Weight_Shiwu;
}

//****************************************************
// HX711 읽기
//****************************************************
uint32_t HX711_Read(void)	//128 게인
{
	uint32_t count;
	uint8_t i;
	//int Flag = 0;

	HAL_GPIO_WritePin(HX711_DT_GPIO_Port, HX711_DT_Pin, 1);
	delay_us(1);
	HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, 0);
	delay_us(1);
  	count=0;
  	while(HAL_GPIO_ReadPin(HX711_DT_GPIO_Port, HX711_DT_Pin));
  	for(i=0;i<24;i++)
	{
  		HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, 1);
  		delay_us(1);
	  	count=count<<1;
	  	HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, 0);
	  	delay_us(1);
	  	if(HAL_GPIO_ReadPin(HX711_DT_GPIO_Port, HX711_DT_Pin))
			count++;
	}
  	HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, 1);
	count ^= 0x800000;
	delay_us(1);
	HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, 0);
	delay_us(1);
	return(count);
}
