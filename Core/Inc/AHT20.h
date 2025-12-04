/*
 * AHT20.h
 *
 *  Created on: Nov 11, 2025
 *      Author: henrik
 */

#ifndef AHT20_H
#define AHT20_H

#include "stm32f4xx_hal.h"
#include "math.h"
extern float Temperature;
extern float Humidity;

typedef struct aht20 {
	float temp,humid;
} AHT20_Data;

void AHT20_Init(void);
void AHT20_Measure(void);
AHT20_Data AHT20_Get(void);
extern I2C_HandleTypeDef hi2c1;


#endif
