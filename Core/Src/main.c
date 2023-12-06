/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "i2c-lcd.h"
#include "lis3dh.h"
#include "dht.h"
#include "tmp117.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  GPIO_TypeDef* dht_port;
  uint16_t dht_pin;
  float temperature;
  float humidity;
} dht_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHT
#define LIS3DH
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

SD_HandleTypeDef hsd1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
bool lcd=1;

/**** Timer related ****/
volatile uint32_t msec=0;
uint32_t dht_millis=0, lis3dh_millis=0, timer_millis=0;
char timer_buffer[9];

/**** DHT22 related ****/
#define DHT22_FREQ 20000 /* Get data every 1 minute */
dht_t *dht = NULL;
uint8_t dht_num;
float tmp117=0;

/**** SD Card related ****/
FIL accelerometer, thermohygrometer, dht_parameters; /* File */
FRESULT fresult; /* FatFs function common result code */
volatile uint32_t byteswritten;/* File write counts */
char read_buffer[20], acc_buffer[20], temp_buffer_sd[150], temp_buffer_lcd[20], timer_lcd[7];

/**** Accelerometer related ****/
stmdev_ctx_t dev_ctx;
int16_t data_raw_acceleration;
float acceleration_mg;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MicroDelay (uint16_t time){
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim6))<time);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if (htim == &htim2){
    msec++;
  }
}

GPIO_TypeDef* GetGpioFromString(const char* gpio_string){
	if (strcmp(gpio_string,"GPIOA")==0) return GPIOA;
	else if (strcmp(gpio_string,"GPIOB")==0) return GPIOB;
	else if (strcmp(gpio_string,"GPIOC")==0) return GPIOC;
	else if (strcmp(gpio_string,"GPIOI")==0) return GPIOI;
	else {printf("Detect nothing\r\n"); return NULL;}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_I2C3_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim6);

  printf("\r\n");

  /* Initialise I2C hardwares */
  if(lcd) LcdInit();
//  Lis3dhInit(&dev_ctx, &hi2c3);
//  Tmp117_Init(hi2c3);

  /* Initialise logging file */
  if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 1) != FR_OK) /* Mount uSD card */
    printf("ERROR in mounting uSD CARD...\r\n");
  else {
  	printf("uSD CARD mounted SUCCESSFULLY\r\n");

  	/* Read parameters from DHT_PARA.TXT file, initialise DHTs */
  	if(f_open(&dht_parameters, "DHT_PARA.TXT", FA_READ) != FR_OK)	printf("Can't open DHT_PARA.TXT\r\n");
  	else {
  		printf("DHT_PARA.TXT can be opened\r\n");
  		/* Read number of DHTs (first line of TXT file) and re-allocated the array */
  		if (f_gets(read_buffer, sizeof(read_buffer), &dht_parameters) != NULL){
  			dht_num = atoi(strchr(read_buffer, ':')+1);
  			dht = malloc(dht_num * sizeof(dht_t));
  		}
  		/* Read DHTs GPIO Port and GPIO Pin (the remaining lines) */
  		uint8_t dht_count=0;
  		for (uint8_t i=0; i<dht_num; i++){
  			if(f_gets(read_buffer, sizeof(read_buffer), &dht_parameters) != NULL){
  				dht_count++;
  				char gpio_port[6];
  				strncpy(gpio_port, strchr(read_buffer, ',')+1, 5); /* Get string "GPIOX" from read buffer */
  				gpio_port[5]= '\0'; /* NULL Terminator */

  				dht[i].dht_port = GetGpioFromString(gpio_port);
  				dht[i].dht_pin = pow(2,atoi(strchr(strchr(read_buffer, ',')+1, ',')+1)); /* Refer to #define GPIO_PIN_x (DEC)2^x */
  				printf("%i %s%i | ",i+1,gpio_port,(uint8_t)log2(dht[i].dht_pin));
  			}
  		}
			printf("\r\nNumber of DHTs initialised from TXT file: %i\r\n",dht_num);
  		printf("Number of DHTs can be read from TXT file: %i\r\n",dht_count);
  		if (dht_count < dht_num) dht_num = dht_count;
  		f_close(&dht_parameters); /* Close TXT file */
  	}
  	/* Initialise TEMP_LOG.TXT file */
  	if(f_open(&thermohygrometer, "TEMP_LOG.TXT", FA_OPEN_APPEND | FA_WRITE) != FR_OK)	printf("Can't create/open TEMP.TXT\r\n");
  	else {
  		printf("TEMP_LOG.TXT can be created\r\n");
  		if (f_write(&thermohygrometer, "Starting logging data to TEMP_LOG.TXT\r\n", 42, (void *)&byteswritten) != FR_OK) printf("Write error\r\n");
  		else{
  			printf("TEMP_LOG.TXT created and data is written\r\n");
  			f_close(&thermohygrometer); /* Close TXT file */
  		}
  	}
  	/* Initialise ACC_LOG.TXT file */
  	if(f_open(&accelerometer, "ACC_LOG.TXT", FA_OPEN_APPEND | FA_WRITE) != FR_OK) printf("Can't create/open ACC_LOG.TXT\r\n");
  	else {
  		printf("ACC_LOG.TXT can be created\r\n");
  		if (f_write(&accelerometer, "Starting logging data to ACC_LOG.TXT...\r\n", 41, (void *)&byteswritten) != FR_OK) printf("Write error\r\n");
  		else{
  			printf("ACC_LOG.TXT created and data is written\r\n");
	  		f_close(&accelerometer); /* Close TXT file */
  		}
  	}
  }
  /* Start Timer2 */
  HAL_TIM_Base_Start_IT(&htim2);

  printf("\r\n");
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); /* Turn LD2 on to confirm all components are initialised succesfully */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if defined(LIS3DHH)
  	if(HAL_GetTick() - lis3dh_millis >= 10000){
  		lis3dh_millis = HAL_GetTick();

//  		f_open(&accelerometer, "ACC_LOG.TXT", FA_OPEN_APPEND | FA_WRITE);
  		while(HAL_GetTick() - lis3dh_millis <= 5000){
  			lis3dh_acceleration_raw_get_z(&dev_ctx, &data_raw_acceleration);
  			acceleration_mg = (lis3dh_from_fs4_hr_to_mg(data_raw_acceleration));//-(-48))*1.4359 + (-441.6);
  			sprintf(acc_buffer, "%li, %.2f\t\r\n", HAL_GetTick(), acceleration_mg);
//  			f_write(&accelerometer, acc_buffer, strlen((char *)acc_buffer), (void *)&byteswritten); /* Write to TXT file */
  			printf(acc_buffer);
  		}
//  		if (f_close(&accelerometer) == FR_OK) printf("Write and close successfully\r\n"); /* Close TXT file */
  	}
#endif

#if defined(DHT)
  	/* Get temperature/humidity data and log to TXT file */
  	if (HAL_GetTick() - dht_millis >= 5000){
  		dht_millis = HAL_GetTick();

  		/* Read temperature and humidity from DHTs */
  		for (uint8_t i=0; i<dht_num; i++){
  			Dht22(dht[i].dht_port, dht[i].dht_pin, &dht[i].temperature, &dht[i].humidity);
  		}
//  		tmp117 = TMP117_get_Temperature(hi2c3);

  		/* Create string to write to file */
  		uint8_t buffer_offset=0, buffer_offset_lcd=0;
  		buffer_offset += sprintf(temp_buffer_sd + buffer_offset, "%li", HAL_GetTick());
  		for (uint8_t i=0; i<dht_num; i++){
  			buffer_offset += sprintf(temp_buffer_sd + buffer_offset, ", %.2f,%.2f", dht[i].temperature, dht[i].humidity);
  			buffer_offset_lcd += sprintf(temp_buffer_lcd+buffer_offset_lcd, "%.1f  ",dht[i].temperature);
      }
//  		buffer_offset += sprintf(temp_buffer_sd + buffer_offset, ", %.2f", tmp117);
  		sprintf(temp_buffer_sd + buffer_offset, "\r\n");
  		sprintf(timer_lcd, "%li",(dht_millis/1000));

  		/* Logging to TXT file and debugging */
  		fresult = f_open(&thermohygrometer, "TEMP_LOG.TXT", FA_OPEN_APPEND | FA_WRITE); /* Open TXT file */
  		if (fresult != FR_OK){
  			printf("ERROR! File could not be opened! Fresuslt is %i\r\n",fresult);
//  			if(lcd) LcdSendStringAtPos(0, 15, "E");
  		}
  		else {
  			// printf("File is opened successfully!\r\n");
//  			if(lcd) LcdSendStringAtPos(0,15,"O");
  			fresult = f_write(&thermohygrometer, temp_buffer_sd, strlen(temp_buffer_sd), (void *)&byteswritten); /* Write data to TXT file */
  			if (fresult != FR_OK) printf("ERROR! File could not be written! Fresult is %i\r\n",fresult);
  			else {
  				printf("%s", temp_buffer_sd);
  				if(lcd) {
  					// LcdSendStringAtPos(0,15,"W")
  					LcdSendStringAtPos(1, 0, temp_buffer_lcd);
  					LcdSendStringAtPos(0, 10, timer_lcd);
  				}
  				fresult = f_close(&thermohygrometer); /* Close TXT file */
  				if (fresult != FR_OK) printf("ERROR! File could not be closed! Fresult is %i\r\n",fresult);
  				else {
  					// printf("File is closed successfully!\r\n");
//  					if(lcd) LcdSendStringAtPos(0,15,"C");
  				}
  			}
  		}
  	}

  	/* Display timer on LCD */
  	if (HAL_GetTick() - timer_millis >= 1000){
  		timer_millis = HAL_GetTick();
  		sprintf(timer_buffer, "%02lu:%02lu:%02lu", timer_millis/3600000, (timer_millis/60000)%60, (timer_millis/1000)%60); /* Format "HH:MM:SS" */
  		LcdSendStringAtPos(0, 0, timer_buffer);
  	}
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  f_close(&accelerometer); /* Close TXT file */
  printf("Closed\r\n");
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 15;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00B03FDB;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  hsd1.Init.Transceiver = SDMMC_TRANSCEIVER_DISABLE;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 120-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 120-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xFFFF -1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT_GPIO_Port, DHT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DHT_Pin */
  GPIO_InitStruct.Pin = DHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DHT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_Detect_Pin */
  GPIO_InitStruct.Pin = uSD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
