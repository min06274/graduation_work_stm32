/**
  ******************************************************************************

  File:		ESP DATA HANDLER
  Author:   ControllersTech
  Updated:  3rd Aug 2020

  ******************************************************************************
  Copyright (C) 2017 ControllersTech.com

  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.

  ******************************************************************************
*/



#include "ESP_DATA_HANDLER.h"
#include "UartRingbuffer_multi.h"
#include "stdio.h"
#include "string.h"
#include "StepController.h"


extern print_flag;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

#define wifi_uart &huart1
#define pc_uart &huart2
#define maxnumberofusers  10  // Max number of users
#define SALT 1
#define SUGAR 2
#define BLACK 3
#define RED 4
char buffer[20];
userDetails user[maxnumberofusers];

int usernumber = 0;

int sizeofuser (userDetails *user)
{
	int size=0;
	while (user[size].Salt_weight[0] != '\0') size++;
	return size+1;
}

char *home = "<!DOCTYPE html>\n\
		<html>\n\
		<body>\n\
		<h1>ESP8266 USER DATA COLLECTION</h1>\n\
		<p>Enter the Details in the form below: </p>\n\
		<form action=\"/page1\">\n\
		<label for=\"salt\">Salt:</label><br>\n\
		<input type=\"number\" id=\"salt\" name=\"salt\" value=\"\"><br><br>\n\
		<label for=\"sugar\">Sugar:</label><br>\n\
		<input type=\"number\" id=\"sugar\" name=\"sugar\" value=\"\"><br><br>\n\
		<label for=\"blackpepper\">BlackPepper:</label><br>\n\
		<input type=\"number\" id=\"blackpepper\" name=\"blackpepper\" value=\"\"><br><br>\n\
		<label for=\"Redpepper\">RedPepper:</label><br>\n\
		<input type=\"number\" id=\"redpepper\" name=\"redpepper\" value=\"\"><br><br>\n\
		<input type=\"submit\" value=\"Submit\">\n\
		</form><br><br>\n\
		<form action=\"/page2\">\n\
		<input type=\"submit\" value=\"View Data\">\n\
		</form>\n\
		</body></html>";

char *page1 = "<!DOCTYPE html>\n\
		<html>\n\
		<body>\n\
		<h1>ESP8266 USER DATA COLLECTION</h1>\n\
		<h2> DATA STORED Successfully </h2>\n\
		<p> Click Below to Submit again </p>\n\
		<form action=\"/home\">\n\
		<input type=\"submit\" value=\"Submit Again\">\n\
		</form><br><br>\n\
		<form action=\"/page2\">\n\
		<input type=\"submit\" value=\"View Data\">\n\
		</form>\n\
		</body></html>";

char *page2_Top = "<!DOCTYPE html>\n\
		<html>\n\
		<body>\n\
		<h1>ESP8266 USER DATA COLLECTION</h1>\n\
		<h2> DATA CCOLLECTED is Shown BELOW </h2>\n";

char *page2_end = "<p> Click Below to Submit again </p>\n\
		<form action=\"/home\">\n\
		<input type=\"submit\" value=\"Submit again\">\n\
		</body></html>";

char *table = "<style>table {  font-family: arial, sans-serif;\
		border-collapse: collapse;  width: 50%;}\
		td, th {  border: 1px solid #dddddd;\
		text-align: left;  padding: 8px;}tr:nth-child(even)\
		{  background-color: #dddddd;}</style><table><tr><th>Salt</th><th>Sugar</th><th>BlackPepper</th><th>RedPepper</th></tr>";


/*****************************************************************************************************************************************/

void ESP_Init (char *SSID, char *PASSWD, char *STAIP)
{
	char data[80];

	Ringbuf_init();

	Uart_sendstring("AT+RST\r\n", wifi_uart);
	Uart_sendstring("RESETTING.", pc_uart);
	for (int i=0; i<5; i++)
	{
		Uart_sendstring(".", pc_uart);
		HAL_Delay(1000);
	}

	/********* AT **********/
	Uart_flush(wifi_uart);
	Uart_sendstring("AT\r\n", wifi_uart);
	while(!(Wait_for("OK\r\n", wifi_uart)));
	Uart_sendstring("AT---->OK\n\n", pc_uart);


	/********* AT+CWMODE=1 **********/
	Uart_flush(wifi_uart);
	Uart_sendstring("AT+CWMODE=1\r\n", wifi_uart);
	while (!(Wait_for("OK\r\n", wifi_uart)));
	Uart_sendstring("CW MODE---->1\n\n", pc_uart);


	/* Set Static IP Address */
	/********* AT+CWSTAIP=IPADDRESS **********/
	Uart_flush(wifi_uart);
	sprintf (data, "AT+CIPSTA=\"%s\"\r\n", STAIP);
	Uart_sendstring(data,wifi_uart);
	while (!(Wait_for("OK\r\n",wifi_uart)));

	/********* AT+CWJAP="SSID","PASSWD" **********/
	Uart_flush(wifi_uart);
	Uart_sendstring("connecting... to the provided AP\n", pc_uart);
	sprintf (data, "AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, PASSWD);
	Uart_sendstring(data, wifi_uart);
	while (!(Wait_for("OK\r\n", wifi_uart)));
	sprintf (data, "Connected to,\"%s\"\n\n", SSID);
	Uart_sendstring(data,pc_uart);




	/********* AT+CIPMUX **********/
	Uart_flush(wifi_uart);
	Uart_sendstring("AT+CIPMUX=1\r\n", wifi_uart);
	while (!(Wait_for("OK\r\n", wifi_uart)));
	Uart_sendstring("CIPMUX---->OK\n\n", pc_uart);

	/********* AT+CIPSERVER **********/
	Uart_flush(wifi_uart);
	Uart_sendstring("AT+CIPSERVER=1,80\r\n", wifi_uart);
	while (!(Wait_for("OK\r\n", wifi_uart)));
	Uart_sendstring("CIPSERVER---->OK\n\n", pc_uart);

	Uart_sendstring("Now Connect to the IP ADRESS\n\n", pc_uart);




}




int Server_Send (char *str, int Link_ID)
{
	int len = strlen (str);
	char data[80];
	sprintf (data, "AT+CIPSEND=%d,%d\r\n", Link_ID, len);
	Uart_sendstring(data, wifi_uart);
	while (!(Wait_for(">", wifi_uart)));
	Uart_sendstring (str, wifi_uart);
	while (!(Wait_for("SEND OK", wifi_uart)));
	sprintf (data, "AT+CIPCLOSE=5\r\n");
	Uart_sendstring(data, wifi_uart);
	while (!(Wait_for("OK\r\n", wifi_uart)));
	return 1;
}

void Server_Handle (char *str, int Link_ID)
{
	char datatosend[4096] = {0};
	if (!(strcmp (str, "/page1")))
	{
		sprintf(datatosend, page1);
		Server_Send(datatosend, Link_ID);
	}

	else if (!(strcmp (str, "/page2")))
	{
		char localbuf[2048];
		sprintf(datatosend, page2_Top);
		strcat (datatosend, table);
		int bufsize = (sizeofuser (user));
		for (int i=0; i<bufsize; i++)
		{
			sprintf (localbuf, "<tr><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>",user[i].Salt_weight,user[i].Sugar_weight,user[i].BlackPepper_weight, user[i].RedPepper_weight);
			strcat (datatosend, localbuf);
		}
		strcat (datatosend, "</table>");
		strcat(datatosend, page2_end);
		Server_Send(datatosend, Link_ID);
	}
	else
	{
		sprintf (datatosend, home);
		Server_Send(datatosend, Link_ID);
	}

}

void Server_Start (void)
{
	char buftostoreheader[128] = {0};
	char Link_ID;
	while (!(Get_after("+IPD,", 1, &Link_ID, wifi_uart)));

	Link_ID -= 48;
	while (!(Copy_upto(" HTTP/1.1", buftostoreheader, wifi_uart)));
	if (Look_for("/page1", buftostoreheader) == 1)
	{



		stepStart(BLACK);
		/*
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
		*/
		print_flag = 1;
		GetDataFromBuffer("salt=", "&", buftostoreheader, user[usernumber].Salt_weight,&user[usernumber].salt);

		GetDataFromBuffer("sugar=", "&", buftostoreheader, user[usernumber].Sugar_weight,&user[usernumber].sugar);
		GetDataFromBuffer("blackpepper=", "&", buftostoreheader, user[usernumber].BlackPepper_weight,&user[usernumber].black);
		GetDataFromBuffer("redpepper=", " HTTP", buftostoreheader, user[usernumber].RedPepper_weight,&user[usernumber].red);
		usernumber++;
		if (usernumber >9) usernumber = 0;
		Server_Handle("/page1",Link_ID);
	}

	else if (Look_for("/page2", buftostoreheader) == 1)
	{
		Server_Handle("/page2",Link_ID);
	}

	else if (Look_for("/home", buftostoreheader) == 1)
	{
		Server_Handle("/home",Link_ID);
	}

	else if (Look_for("/favicon.ico", buftostoreheader) == 1);

	else
	{
		Server_Handle("/ ", Link_ID);
	}
}



