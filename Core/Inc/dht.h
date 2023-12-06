/*
 * dht.h
 *
 *  Created on: Aug 8, 2023
 *      Author: Minh Nguyen
 */

#ifndef INC_DHT_H_
#define INC_DHT_H_

#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"

/* DHT type */
#define DHT11 0
#define DHT22 1

extern TIM_HandleTypeDef htim6;

/* Delay function */
void MicroDelay (uint16_t time);

/* GPIO setup */
void SetPinOutput (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void SetPinInput (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

/* DHT22 Functions */
void Dht22(GPIO_TypeDef *dht_port, uint16_t dht_pin, float *temperature, float *humidity);
void Dht11(GPIO_TypeDef *dht_port, uint16_t dht_pin, float *temperature, float *humidity);
void DhtStart (GPIO_TypeDef *dht_port, uint16_t dht_pin, bool dht_type);
uint8_t DhtCheckResponse (GPIO_TypeDef *dht_port, uint16_t dht_pin);
uint8_t DhtRead (GPIO_TypeDef *dht_port, uint16_t dht_pin);

#endif /* INC_DHT_H_ */
