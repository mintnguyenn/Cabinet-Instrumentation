/*
 * lis3dh.h
 *
 *  Created on: Aug 15, 2023
 *      Author: Minh Nguyen
 */

#ifndef INC_LIS3DH_H_
#define INC_LIS3DH_H_

#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "lis3dh_reg.h"

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

uint8_t Lis3dhInit(stmdev_ctx_t *dev_ctx, I2C_HandleTypeDef *i2c_handle);

#endif /* INC_LIS3DH_H_ */


