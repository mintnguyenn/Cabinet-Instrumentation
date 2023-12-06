/*
 * dht.c
 *
 *  Created on: Aug 8, 2023
 *      Author: Minh Nguyen
 */

#include <stdbool.h>
#include "dht.h"

volatile uint32_t time1, time2, time3;
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2, SUM;

void SetPinOutput (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void SetPinInput (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/*********************************** DHT22 FUNCTIONS ********************************************/
void DhtStart (GPIO_TypeDef *dht_port, uint16_t dht_pin, bool dht_type){
	SetPinOutput(dht_port, dht_pin);               /* Set the pin as OUTPUT */
	HAL_GPIO_WritePin(dht_port, dht_pin, 0);       /* Set the pin to LOW    */
	if (dht_type == DHT22) MicroDelay(1300);       /* Wait for at least 1ms */
	else if (dht_type == DHT11) MicroDelay(19000); /* Wait for at least 1ms */
	HAL_GPIO_WritePin(dht_port, dht_pin, 1);       /* Set the pin to HIGH   */
	MicroDelay(30);
	SetPinInput(dht_port, dht_pin);                /* Set the pin as OUTPUT */
}

uint8_t DhtCheckResponse (GPIO_TypeDef *dht_port, uint16_t dht_pin){
	uint8_t response = 0;
	MicroDelay(40);
	if (!(HAL_GPIO_ReadPin (dht_port, dht_pin))) {
		MicroDelay(80);
		if ((HAL_GPIO_ReadPin (dht_port, dht_pin))) response = 1;
		else response = -1;
	}

	if (response != 1) return 0;

	time1 = HAL_GetTick();
	while ((HAL_GPIO_ReadPin (dht_port, dht_pin))){ /* Wait for the pin go LOW - break after 1ms */
		if (HAL_GetTick() - time1 > 5) break;
	}
  
  return response;
}

uint8_t DhtRead (GPIO_TypeDef *dht_port, uint16_t dht_pin){
	uint8_t data;
	for (uint8_t j=0;j<8;j++){

		time2 = HAL_GetTick();
		while (!(HAL_GPIO_ReadPin (dht_port, dht_pin))){  /* Wait for the pin go HIGH - break after 1ms */
			if (HAL_GetTick() - time2 > 5) break;
		}
		MicroDelay(40); /* Wait for 40us */
		if (!(HAL_GPIO_ReadPin (dht_port, dht_pin))) data&= ~(1<<(7-j)); /* If the pin is LOW , write 0 */
		else data|= (1<<(7-j));                                          /* If the pin is HIGH, write 1 */

		time3 = HAL_GetTick();
		while ((HAL_GPIO_ReadPin (dht_port, dht_pin))){  /* Wait for the pin go LOW - break after 1ms */
			if (HAL_GetTick() - time3 > 5) break;
		}
	}
	return data;
}

void Dht22(GPIO_TypeDef *dht_port, uint16_t dht_pin, float *temperature, float *humidity){
	DhtStart(dht_port, dht_pin, DHT22);
	if (DhtCheckResponse(dht_port, dht_pin)){
		Rh_byte1 = DhtRead(dht_port, dht_pin);
		Rh_byte2 = DhtRead(dht_port, dht_pin);
		Temp_byte1 = DhtRead(dht_port, dht_pin);
		Temp_byte2 = DhtRead(dht_port, dht_pin);
		SUM = DhtRead(dht_port, dht_pin);

		if (SUM != ((Rh_byte1+Rh_byte2+Temp_byte1+Temp_byte2)&0xFF)){
			*temperature = 0;
			*humidity = 0;
			return;
		}

		uint16_t TEMP = ((Temp_byte1<<8)|Temp_byte2);
		uint16_t RH = ((Rh_byte1<<8)|Rh_byte2);
		*temperature = (float) (TEMP/10.0);
		*humidity = (float) (RH/10.0);
	}
	else {
		*temperature = 0;
		*humidity = 0;
	}
}

void Dht11(GPIO_TypeDef *dht_port, uint16_t dht_pin, float *temperature, float *humidity){
	DhtStart(dht_port, dht_pin, DHT11);
	if (DhtCheckResponse(dht_port, dht_pin))
	{
		Rh_byte1   = DhtRead(dht_port, dht_pin);
		Rh_byte2   = DhtRead(dht_port, dht_pin);
		Temp_byte1 = DhtRead(dht_port, dht_pin);
		Temp_byte2 = DhtRead(dht_port, dht_pin);
		SUM        = DhtRead(dht_port, dht_pin);
		if (SUM != Rh_byte1+Rh_byte2+Temp_byte1+Temp_byte2){
			*temperature = -1;
			*humidity = -1;
			return;
		}

		*temperature = (float)Temp_byte1 + ((float)Temp_byte2 / 10.0);
		*humidity = (float)Rh_byte1 + ((float)Rh_byte2 / 10.0);
	}
	else {
		*temperature = -1;
		*humidity = -1;
	}
}
