/*
 * lis3dh.c
 *
 *  Created on: Aug 15, 2023
 *      Author: nhatm
 */

#include "lis3dh.h"

static uint8_t whoamI;
extern I2C_HandleTypeDef hi2c3;

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len){
	reg |= 0x80;
	HAL_I2C_Mem_Write(handle, LIS3DH_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
	return 0;
}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len){
	reg |= 0x80;
	HAL_I2C_Mem_Read(handle, LIS3DH_I2C_ADD_H, reg,	I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	return 0;
}

uint8_t Lis3dhInit(stmdev_ctx_t *dev_ctx,  I2C_HandleTypeDef *i2c_handle){
	/* Initialize mems driver interface */
	dev_ctx->write_reg = platform_write;
	dev_ctx->read_reg = platform_read;
	dev_ctx->handle = i2c_handle;

	/* Check device ID */
	lis3dh_device_id_get(dev_ctx, &whoamI);

	if (whoamI != LIS3DH_ID) {
		printf("LIS3DH not found, %i\r\n", whoamI);
		return 0;
	}
	else{
		printf("LIS3DH detected\r\n");

		/*  Enable Block Data Update */
		lis3dh_block_data_update_set(dev_ctx, PROPERTY_ENABLE);
		/* Set Output Data Rate to 1 khz */
		lis3dh_data_rate_set(dev_ctx, LIS3DH_ODR_5kHz376_LP_1kHz344_NM_HP);
		/* Set full scale to 2 g */
		lis3dh_full_scale_set(dev_ctx, LIS3DH_4g);
		/* Set operating mode to high resolution */
		lis3dh_operating_mode_set(dev_ctx, LIS3DH_HR_12bit);
		/* Set FIFO watermark to 25 samples */
		lis3dh_fifo_watermark_set(dev_ctx, 25);
		/* Set FIFO mode to Stream mode: Accumulate samples and override old data */
		lis3dh_fifo_mode_set(dev_ctx, LIS3DH_BYPASS_MODE);
		/* Enable FIFO */
		lis3dh_fifo_set(dev_ctx, PROPERTY_DISABLE);

		printf("LIS3DH initialised\r\n");
		return 1;
	}
}
