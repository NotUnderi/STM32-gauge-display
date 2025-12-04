/*
 * AHT20.c
 *
 *  Created on: Nov 11, 2025
 *      Author: henrik
 */
#include "stm32f4xx_hal.h"
#include <AHT20.h>

#define AHT20_ADDRESS 0x38<<1
float Temperature = 0.0f;
float Humidity    = 0.0f;
void AHT20_Init (void)
{
	HAL_Delay(40);
	uint8_t status;
	HAL_I2C_Mem_Read(&hi2c1, AHT20_ADDRESS, 0x71, 1, &status, 1, 1000);
	if ((status>>3 & 0x01) == 0)  // if CAL = 0
	{
		uint8_t init_commands[3] = {0xBE, 0x08, 0x00};
		HAL_I2C_Master_Transmit(&hi2c1, AHT20_ADDRESS, init_commands, 3, 1000);
		HAL_Delay (10);
	}
}
void AHT20_Measure (void)
{
	uint8_t measure_command[3] = {0xAC, 0x33, 0x00};
	HAL_I2C_Master_Transmit(&hi2c1, AHT20_ADDRESS, measure_command, 3, 1000);
	HAL_Delay(80);

	uint8_t status;
	do {
		HAL_I2C_Mem_Read(&hi2c1, AHT20_ADDRESS, 0x71, 1, &status, 1, 1000);
		HAL_Delay(100);
	}
	while ((status>>7 & 0x01) == 1);

	uint8_t RxData[7];
	HAL_I2C_Master_Receive(&hi2c1, AHT20_ADDRESS, RxData, 7, 1000);
	uint32_t HUM_DATA = (RxData[1]<<16)|(RxData[2]<<8)|RxData[3];  // accumulated 24bit data
	HUM_DATA = HUM_DATA>>4;  // 20bit data
	Humidity = (float) ((HUM_DATA/pow(2,20)) * 100);
	uint32_t TEMP_DATA = (RxData[3]<<16)|(RxData[4]<<8)|RxData[5];  // accumulated 24bit data
	TEMP_DATA = TEMP_DATA&0xFFFFF;  // first 20bit data
	Temperature = (float) (((TEMP_DATA/pow(2,20)) * 200) - 50);
}
AHT20_Data AHT20_Get(void){
	AHT20_Data val;
	AHT20_Measure();
	printf("Humidity: %.2f %%RH | Temperature: %.2f °C\n", Humidity, Temperature);
	printf("-------------------------------------------\n");
	val.temp = Temperature;
	val.humid = Humidity;
	return val;
}

