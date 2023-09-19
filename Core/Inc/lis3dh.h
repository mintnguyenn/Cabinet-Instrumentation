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

stmdev_ctx_t accelerometerInit();

//void lis3dhReadFIFO(stmdev_ctx_t dev_ctx, FIL fil);

#endif /* INC_LIS3DH_H_ */


