#include "i2c-lcd.h"
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c3;  // change your handler here accordingly

#define SLAVE_ADDRESS_LCD 0x4E // change this according to your setup

void LcdSendCmd(char cmd)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c3, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void LcdSendData(char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c3, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void LcdClear(void)
{
	LcdSendCmd(0x80);
	for (int i=0; i<70; i++)
	{
		LcdSendData(' ');
	}
}

void LcdSetCursor(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    LcdSendCmd(col);
}

void LcdInit(void)
{
	// 4 bit initialisation
	HAL_Delay(50); // Wait for >40ms
	LcdSendCmd (0x30);
	HAL_Delay(5);  // Wait for >4.1ms
	LcdSendCmd (0x30);
	HAL_Delay(1);  // Wait for >100us
	LcdSendCmd (0x30);
	HAL_Delay(10);
	LcdSendCmd (0x20);  // 4-bit mode
	HAL_Delay(10);

  	/* Dislay initialisation */
	LcdSendCmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	LcdSendCmd (0x08); // Display on/off control --> D=0, C=0, B=0  ---> display off
	HAL_Delay(1);
	LcdSendCmd (0x01); // Clear display
	HAL_Delay(1);
	HAL_Delay(1);
	LcdSendCmd (0x06); // Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	LcdSendCmd (0x0C); // Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void LcdSendString (char *str)
{
	while (*str) LcdSendData (*str++);
}

void LcdSendStringAtPos(uint8_t row, uint8_t column, char *string)
{
	LcdSetCursor(row, column);
	LcdSendString(string);
}

/*
void lcd_send_float(float num){
	char txbuf[20];
	sprintf(txbuf,"%f",num);
	lcd_send_string(txbuf);
}

void lcd_send_integer(int num){
	uint8_t txbuf[20];
	sprintf((const char*)txbuf,"%i",num);
	lcd_send_string(txbuf);
}
*/
