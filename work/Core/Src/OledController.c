#include "OledController.h"
#include "eddy_logo.h"
#include <stdio.h>
#include "ESP_DATA_HANDLER.h"

extern userDetails user[10];
extern int usernumber;
extern int print_usernumber;

void opening(int idx, int flag) {

	switch (idx) {

	case 0:
		SSD1306_Clear();

		SSD1306_GotoXY(0, 0);

		if (flag == 1) {
			char weight_str_salt[100] = "carboh ";
			char temp_salt[10]="";

			sprintf(temp_salt, "%d", user[print_usernumber].salt);

			strcat(weight_str_salt, temp_salt);
			SSD1306_Puts(weight_str_salt, &Font_11x18, 1);


		} else if (flag == 2) {
			char weight_str_sugar[100] = "protein ";
			char temp_sugar[10]="";

			sprintf(temp_sugar, "%d", user[print_usernumber].sugar);

			strcat(weight_str_sugar, temp_sugar);
			SSD1306_Puts(weight_str_sugar, &Font_11x18, 1);
		}
		else if (flag == 3) {
					char weight_str_black[100] = "fat ";
					char temp_black[10]="";

					sprintf(temp_black, "%d", user[print_usernumber].black);

					strcat(weight_str_black, temp_black);
					SSD1306_Puts(weight_str_black, &Font_11x18, 1);
				}

		SSD1306_DrawBitmap(0, 52, logo0, 128, 12, 1);
		SSD1306_UpdateScreen();
	case 1:
		SSD1306_DrawFilledRectangle(0, 52, 128, 12, 0);

		SSD1306_DrawBitmap(0, 52, logo1, 128, 12, 1);
		SSD1306_UpdateScreen();

		break;
	case 2:
		SSD1306_DrawFilledRectangle(0, 52, 128, 12, 0);

		SSD1306_DrawBitmap(0, 52, logo2, 128, 12, 1);
		SSD1306_UpdateScreen();
		break;
	case 3:
		SSD1306_DrawFilledRectangle(0, 52, 128, 12, 0);

		SSD1306_DrawBitmap(0, 52, logo3, 128, 12, 1);
		SSD1306_UpdateScreen();
		break;

	case 4:
		SSD1306_DrawBitmap(0, 52, logo4, 128, 12, 1);
		SSD1306_UpdateScreen();

		break;
	case 5:
		SSD1306_DrawFilledRectangle(0, 52, 128, 12, 0);

		SSD1306_DrawBitmap(0, 52, logo5, 128, 12, 1);
		SSD1306_UpdateScreen();
		break;
	case 6:
		SSD1306_DrawFilledRectangle(0, 52, 128, 12, 0);

		SSD1306_DrawBitmap(0, 52, logo6, 128, 12, 1);
		SSD1306_UpdateScreen();
		break;
	case 7:
		SSD1306_DrawBitmap(0, 52, logo7, 128, 12, 1);
		SSD1306_UpdateScreen();

		break;
	case 8:
		SSD1306_DrawFilledRectangle(0, 52, 128, 12, 0);

		SSD1306_DrawBitmap(0, 52, logo8, 128, 12, 1);
		SSD1306_UpdateScreen();
		break;
	case 9:
		SSD1306_DrawFilledRectangle(0, 52, 128, 12, 0);

		SSD1306_DrawBitmap(0, 52, logo9, 128, 12, 1);
		SSD1306_UpdateScreen();
		break;

	case 10:
		SSD1306_DrawBitmap(0, 52, logo10, 128, 12, 1);
		SSD1306_UpdateScreen();

		break;
	case 11:
		SSD1306_DrawFilledRectangle(0, 52, 128, 12, 0);

		SSD1306_DrawBitmap(0, 52, logo11, 128, 12, 1);
		SSD1306_UpdateScreen();
		break;
	case 12:
		SSD1306_DrawFilledRectangle(0, 52, 128, 12, 0);

		SSD1306_DrawBitmap(0, 52, logo12, 128, 12, 1);
		SSD1306_UpdateScreen();
		break;
	case 13:
		SSD1306_DrawBitmap(0, 52, logo13, 128, 12, 1);
		SSD1306_UpdateScreen();

		break;
	case 14:
		SSD1306_DrawFilledRectangle(0, 52, 128, 12, 0);

		SSD1306_DrawBitmap(0, 52, logo14, 128, 12, 1);
		SSD1306_UpdateScreen();
		break;
	case 15:

		SSD1306_GotoXY(0, 0);

		if (flag == 1) {
			SSD1306_Puts("carboh end", &Font_11x18, 1);
		} else if (flag == 2) {
			SSD1306_Puts("protein end", &Font_11x18, 1);

		}else if (flag == 3) {
			SSD1306_Puts("All end", &Font_11x18, 1);

		}
		SSD1306_DrawFilledRectangle(0, 52, 128, 12, 0);

		SSD1306_DrawBitmap(0, 52, logo15, 128, 12, 1);
		SSD1306_UpdateScreen();
		break;

	}

}

void printDefault() {

	SSD1306_Clear();
	SSD1306_GotoXY(1, 0);
	SSD1306_Puts("select ", &Font_11x18, 1);
	SSD1306_GotoXY(0, 15);
	SSD1306_Puts("---------", &Font_11x18, 1);
	SSD1306_GotoXY(14, 38);
	SSD1306_Puts("cereal!", &Font_11x18, 1);

	SSD1306_UpdateScreen();

}

void printTemper(int temper) {
	SSD1306_GotoXY(14, 38);
	char temper_str[100] = "";
	sprintf(temper_str, "%d", temper);

	SSD1306_Puts(temper_str, &Font_11x18, 1);

	SSD1306_UpdateScreen();

}
