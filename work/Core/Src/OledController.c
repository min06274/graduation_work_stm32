#include "OledController.h"
#include "eddy_logo.h"
#include <stdio.h>


void opening(int idx, int flag)
{





	switch(idx)
	{

	case 0:
		SSD1306_Clear();

		SSD1306_GotoXY(0, 0);

		if(flag == 2)
		{
		SSD1306_Puts("salt ing...", &Font_11x18, 1);
		}
		else if(flag == 1)
		{
			SSD1306_Puts("sugar ing...", &Font_11x18, 1);

		}

		SSD1306_DrawBitmap(0, 52, logo0, 128, 12, 1);
		SSD1306_UpdateScreen();
	case 1:
		SSD1306_DrawFilledRectangle(0,52,128,12,0);

		SSD1306_DrawBitmap(0, 52, logo1, 128, 12, 1);
		SSD1306_UpdateScreen();


		break;
	case 2:
		SSD1306_DrawFilledRectangle(0,52,128,12,0);

		SSD1306_DrawBitmap(0, 52, logo2, 128, 12, 1);
		SSD1306_UpdateScreen();
		break;
	case 3:
		SSD1306_DrawFilledRectangle(0,52,128,12,0);

		SSD1306_DrawBitmap(0, 52, logo3, 128, 12, 1);
		SSD1306_UpdateScreen();
		break;

	case 4:
		SSD1306_DrawBitmap(0, 52, logo4, 128, 12, 1);
		SSD1306_UpdateScreen();

		break;
	case 5:
		SSD1306_DrawFilledRectangle(0,52,128,12,0);

		SSD1306_DrawBitmap(0, 52, logo5, 128, 12, 1);
		SSD1306_UpdateScreen();
		break;
	case 6:
		SSD1306_DrawFilledRectangle(0,52,128,12,0);

		SSD1306_DrawBitmap(0, 52, logo6, 128, 12, 1);
		SSD1306_UpdateScreen();
		break;
	case 7:
		SSD1306_DrawBitmap(0, 52, logo7, 128, 12, 1);
		SSD1306_UpdateScreen();

		break;
	case 8:
		SSD1306_DrawFilledRectangle(0,52,128,12,0);

		SSD1306_DrawBitmap(0, 52, logo8, 128, 12, 1);
		SSD1306_UpdateScreen();
		break;
	case 9:
		SSD1306_DrawFilledRectangle(0,52,128,12,0);

		SSD1306_DrawBitmap(0, 52, logo9, 128, 12, 1);
		SSD1306_UpdateScreen();
		break;


	case 10:
		SSD1306_DrawBitmap(0, 52, logo10, 128, 12, 1);
		SSD1306_UpdateScreen();

		break;
	case 11:
		SSD1306_DrawFilledRectangle(0,52,128,12,0);

		SSD1306_DrawBitmap(0, 52, logo11, 128, 12, 1);
		SSD1306_UpdateScreen();
		break;
	case 12:
		SSD1306_DrawFilledRectangle(0,52,128,12,0);

		SSD1306_DrawBitmap(0, 52, logo12, 128, 12, 1);
		SSD1306_UpdateScreen();
		break;
	case 13:
		SSD1306_DrawBitmap(0, 52, logo13, 128, 12, 1);
		SSD1306_UpdateScreen();

		break;
	case 14:
		SSD1306_DrawFilledRectangle(0,52,128,12,0);

		SSD1306_DrawBitmap(0, 52, logo14, 128, 12, 1);
		SSD1306_UpdateScreen();
		break;
	case 15:

		SSD1306_GotoXY(0, 0);

		if(flag == 2)
		{
		SSD1306_Puts("salt end", &Font_11x18, 1);
		}
		else if(flag == 1)
		{
			SSD1306_Puts("sugar end", &Font_11x18, 1);

		}
		SSD1306_DrawFilledRectangle(0,52,128,12,0);

		SSD1306_DrawBitmap(0, 52, logo15, 128, 12, 1);
		SSD1306_UpdateScreen();
		break;
	}


}


void printDefault(){

	SSD1306_Clear();
	SSD1306_GotoXY(1, 0);
	SSD1306_Puts("select ", &Font_11x18, 1);
	SSD1306_GotoXY(0, 15);
	SSD1306_Puts("---------", &Font_11x18, 1);
	SSD1306_GotoXY(14, 38);
	SSD1306_Puts("seasoning!", &Font_11x18, 1);



	SSD1306_UpdateScreen();


}

void printTemper(int temper)
{
	SSD1306_GotoXY(14, 38);
	char temper_str[100] = "";
	sprintf(temper_str,"%d",temper);

	SSD1306_Puts(temper_str, &Font_11x18, 1);


	SSD1306_UpdateScreen();


}
