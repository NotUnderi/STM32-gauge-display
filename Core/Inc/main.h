/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <stdint.h>
#include "math.h"
#include <stdbool.h>
#include "lvgl.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
typedef struct {
    float tempC;
    float resOhm;
} TempPoint;

typedef struct {
    lv_obj_t *arc;
    lv_obj_t *title;
    lv_obj_t *value;
} QuadGauge;


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ILI_RESET_Pin GPIO_PIN_0
#define ILI_RESET_GPIO_Port GPIOA
#define ILI_D_C_Pin GPIO_PIN_1
#define ILI_D_C_GPIO_Port GPIOA
#define ILI_CS_Pin GPIO_PIN_2
#define ILI_CS_GPIO_Port GPIOA
#define OIL_TEMP_Pin GPIO_PIN_3
#define OIL_TEMP_GPIO_Port GPIOA
#define PRESSURE_Pin GPIO_PIN_4
#define PRESSURE_GPIO_Port GPIOA
#define EGT_CS_Pin GPIO_PIN_0
#define EGT_CS_GPIO_Port GPIOB
#define EGT_FAULT_Pin GPIO_PIN_1
#define EGT_FAULT_GPIO_Port GPIOB
#define EGT_DRDY_Pin GPIO_PIN_2
#define EGT_DRDY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define PRESSURE_MAX_PSI   150.0f
#define SENSOR_V_MIN       0.5f
#define SENSOR_V_MAX       4.5f
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
