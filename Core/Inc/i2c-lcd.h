#include "stm32l4xx_hal.h"

void LcdInit (void); /* Initialise LCD */

void LcdSendCmd (char cmd); /* Send command to the LCD */

void LcdSendData (char data);  /* Send data to the LCD */

void LcdSendString (char *str);  /* Send string to the LCD */

void LcdSendStringAtPos(uint8_t row, uint8_t column, char *string);

void LcdSetCursor(int row, int col);  /* Set cursor at the entered position row (0 or 1), col (0-15) */

void LcdClear (void); /* Clear LCD */