/*
 * dht.c
 *
 *  Created on: Aug 8, 2023
 *      Author: Minh Nguyen
 */

#include "dht.h"

volatile uint32_t time1, time2, time3;

void SetPinOutput (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void SetPinInput (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/*********************************** DHT22 FUNCTIONS ********************************************/
void DhtStart (void)
{
	SetPinOutput(DHT_GPIO_Port, DHT_Pin);         /* Set the pin as OUTPUT */
	HAL_GPIO_WritePin(DHT_GPIO_Port, DHT_Pin, 0); /* Set the pin to LOW    */
	MicroDelay(1300);                              /* Wait for at least 1ms */
	HAL_GPIO_WritePin(DHT_GPIO_Port, DHT_Pin, 1); /* Set the pin to HIGH   */
	MicroDelay(30);
	SetPinInput(DHT_GPIO_Port, DHT_Pin);           /* Set the pin as OUTPUT */
}

uint8_t DhtCheckResponse (void)
{
  int response = 0;
  MicroDelay(40);
  if (!(HAL_GPIO_ReadPin (DHT_GPIO_Port, DHT_Pin)))
    {
	  MicroDelay(80);
      if ((HAL_GPIO_ReadPin (DHT_GPIO_Port, DHT_Pin))) response = 1;
      else response = -1;
    }
  if (response != 1) return 0;
  time1 = HAL_GetTick();
  while ((HAL_GPIO_ReadPin (DHT_GPIO_Port, DHT_Pin))){ /* Wait for the pin go LOW - break after 10ms */
	  if (HAL_GetTick() - time1 > 10) break;
  }
  return response;
}

uint8_t DhtRead (void)
{
  uint8_t i,j;
  for (j=0;j<8;j++)
    {
	  time2 = HAL_GetTick();
	  while (!(HAL_GPIO_ReadPin (DHT_GPIO_Port, DHT_Pin))){  /* Wait for the pin go HIGH - break after 10ms */
		  if (HAL_GetTick() - time2 > 10) break;
	  }
	  MicroDelay(40); /* Wait for 40ms */
      if (!(HAL_GPIO_ReadPin (DHT_GPIO_Port, DHT_Pin))) i&= ~(1<<(7-j)); /* If the pin is LOW , write 0 */
      else i|= (1<<(7-j));                                               /* If the pin is HIGH, write 1 */

      time3 = HAL_GetTick();
      while ((HAL_GPIO_ReadPin (DHT_GPIO_Port, DHT_Pin))){  /* Wait for the pin go LOW - break after 10ms */
    	  if (HAL_GetTick() - time3 > 10) break;
      }
    }
  return i;
}

void Dht22(float *temperature, float *humidity){
	DhtStart();
	if (DhtCheckResponse()){
		uint8_t Rh_byte1 = DhtRead();
		uint8_t Rh_byte2 = DhtRead();
		uint8_t Temp_byte1 = DhtRead();
		uint8_t Temp_byte2 = DhtRead();
		uint16_t SUM = DhtRead();
		if (SUM != Rh_byte1+Rh_byte2+Temp_byte1+Temp_byte2);

		uint16_t TEMP = ((Temp_byte1<<8)|Temp_byte2);
		uint16_t RH = ((Rh_byte1<<8)|Rh_byte2);
		*temperature = (float) (TEMP/10.0);
		*humidity = (float) (RH/10.0);
	}
}
