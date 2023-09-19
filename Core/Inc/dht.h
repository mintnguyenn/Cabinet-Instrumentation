/*
 * dht.h
 *
 *  Created on: Aug 8, 2023
 *      Author: Minh Nguyen
 */

#ifndef INC_DHT_H_
#define INC_DHT_H_

#define DHT_Pin GPIO_PIN_3
#define DHT_GPIO_Port GPIOI

#include <stdio.h>
#include "stm32l4xx_hal.h"

extern TIM_HandleTypeDef htim6;

/* Delay function */
void MicroDelay (uint16_t time);

/* GPIO setup */
void SetPinOutput (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void SetPinInput (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

/* DHT22 Functions */
void Dht22(float *temperature, float *humidity);
void DhtStart (void);
uint8_t DhtCheckResponse (void);
uint8_t DhtRead (void);

#endif /* INC_DHT_H_ */
