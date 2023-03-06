#include "OledController.h"
#include "eddy_logo.h"
#include <stdio.h>

void opening()
{
	/*
	SSD1306_GotoXY(0, 0);
	SSD1306_Puts("Start!", &Font_11x18, 1);
	SSD1306_GotoXY(10, 20);
	SSD1306_Puts("  Seasoning", &Font_11x18, 1);
	SSD1306_GotoXY(24, 40);
	SSD1306_Puts("dispensor!!", &Font_11x18, 1);
	SSD1306_UpdateScreen(); //display
	//SSD1306_InvertDisplay(1);

	HAL_Delay(5000);
	SSD1306_Clear();
	SSD1306_DrawBitmap(0, 0, eddylogo1, 128, 64, 1);
	SSD1306_UpdateScreen();

	HAL_Delay(300);

	SSD1306_Clear();
	SSD1306_DrawBitmap(0, 0, eddylogo2, 128, 64, 1);
	SSD1306_UpdateScreen();
	HAL_Delay(300);

	SSD1306_Clear();
	SSD1306_DrawBitmap(0, 0, eddylogo3, 128, 64, 1);
	SSD1306_UpdateScreen();
	HAL_Delay(300);

	SSD1306_Clear();
	SSD1306_DrawBitmap(0, 0, eddylogo4, 128, 64, 1);
	SSD1306_UpdateScreen();
	HAL_Delay(300);

	SSD1306_Clear();
	SSD1306_DrawBitmap(0, 0, eddylogo1, 128, 64, 1);
	SSD1306_UpdateScreen();
	HAL_Delay(5000);
*/

	printDefault();
}


void printDefault(){

	SSD1306_Clear();
	SSD1306_GotoXY(1, 0);
	SSD1306_Puts("Servo Angle", &Font_11x18, 1);
	SSD1306_GotoXY(0, 15);
	SSD1306_Puts("---------", &Font_11x18, 1);
	SSD1306_GotoXY(14, 38);
	SSD1306_Puts("0 degree", &Font_11x18, 1);



	SSD1306_UpdateScreen();


}

void printTemper(uint8_t temper)
{
	SSD1306_GotoXY(14, 38);
	char temper_str[100] = "";
	sprintf(temper_str,"%d",temper);

	SSD1306_Puts(temper, &Font_11x18, 1);


	SSD1306_UpdateScreen();


}
