/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ILI9341.h"
#include "AHT20.h"
#include "ILI9341.h"
#include "lvgl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



int _write(int file, char *ptr, int len)
{
    (void)file;
    for (int i = 0; i < len; i++) {
        ITM_SendChar((uint32_t)*ptr++);
    }
    return len;
}

static const TempPoint tempTable[] = {
    {20.0f, 2031.0f},
    {30.0f, 1286.0f},
    {40.0f, 843.9f},
    {50.0f, 569.9f},
    {60.0f, 388.0f},
    {70.0f, 277.8f},
    {80.0f, 200.0f},
    {90.0f, 146.7f},
    {100.0f, 108.0f},
    {110.0f, 82.7f},
    {120.0f, 63.5f},
    {130.0f, 49.3f},
    {140.0f, 38.9f},
    {150.0f, 30.4f},
    {160.0f, 24.4f},
    {170.0f, 19.8f},
};
static QuadGauge g_egt, g_oilT, g_oilP, g_amb;

/**
 *
 * @param min Value which arc is empty
 * @param max Value at which arc is full
 * @param title_dx Title x coordinate
 * @param title_dy Title y coordinate
 * @param value_dx Value x coordinate
 * @param value_dy value y coordinate
 */
static void Create_Gauge(lv_obj_t *parent, QuadGauge *g,
                          const char *title, lv_color_t color,
                          int32_t min, int32_t max,
                          uint16_t start_deg, uint16_t end_deg,
                          lv_coord_t title_dx, lv_coord_t title_dy,
                          lv_coord_t value_dx, lv_coord_t value_dy);
static void Gauge_Update(QuadGauge *g, float val, int32_t arc_val, const char *fmt);

uint16_t ADC_ReadRaw(ADC_HandleTypeDef* hadc)
{
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
    uint16_t value = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return value;  // 0..4095 for 12-bit
}

uint16_t ADC_ReadAvg(uint8_t samples,ADC_HandleTypeDef* hadc)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; i++) {
        sum += ADC_ReadRaw(hadc);
    }
    return (uint16_t)(sum / samples);
}


float ADC_To_Temperature(uint16_t adc)
{
	float Rs;
	if (adc==0){Rs = 1e9f;}
	Rs = 330.0f * ((4095.0f/(float)adc)-1.0f);
    int size = sizeof(tempTable)/sizeof(tempTable[0]);

    // Too cold (Rs higher than highest table value)
    if (Rs >= tempTable[0].resOhm)
        return tempTable[0].tempC;

    // Too hot (Rs lower than lowest table value)
    if (Rs <= tempTable[size - 1].resOhm)
        return tempTable[size - 1].tempC;

    // Find interval
    for (int i = 0; i < size - 1; i++) {
        float R1 = tempTable[i].resOhm;
        float R2 = tempTable[i+1].resOhm;
        if (Rs <= R1 && Rs >= R2) {
            float T1 = tempTable[i].tempC;
            float T2 = tempTable[i+1].tempC;

            float f = (Rs - R1) / (R2 - R1);   // linear interpolation
            return T1 + f * (T2 - T1);
        }
    }

    return -273.15f; // shouldn't happen
}

float ADC_ToBar(uint16_t adcv)
{
	float vsens = (3.3f * (float)adcv / 4095.0f)*1.5f;
    if (vsens < SENSOR_V_MIN) vsens = SENSOR_V_MIN;
    if (vsens > SENSOR_V_MAX) vsens = SENSOR_V_MAX;

    float fraction = (vsens - SENSOR_V_MIN) / (SENSOR_V_MAX - SENSOR_V_MIN); // 0..1
    return fraction * PRESSURE_MAX_PSI * 0.0689476f;
}


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	  printf("Booting...\n");
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  AHT20_Init();
  ILI9341_Init();
  ILI9341_SetRotation(1);

  lv_init();
  static uint16_t buf1[240 * 40];   // line buffer for 240px wide round display
  lv_display_t *disp = lv_display_create(240, 240);
  lv_display_set_flush_cb(disp, ILI_flush);
  lv_display_set_buffers(disp, buf1, NULL, sizeof(buf1),
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565_SWAPPED);

  lv_obj_t *scr = lv_display_get_screen_active(disp);
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);



  Create_Gauge(scr, &g_egt,  "EGT",   lv_color_hex(0xF65C00),   0, 1200, 225, 315,    0, -60,   0, -40);
  Create_Gauge(scr, &g_oilT, "OilT",  lv_color_hex(0xFFB000),   0,  100, 315,  45,   40, -10,  40,  10);   // reverse sweep (NW→NE)
  Create_Gauge(scr, &g_amb,  "Amb",   lv_color_hex(0x00D1B2), -400,  800,  45, 135,    0,  60,   0,  40);
  Create_Gauge(scr, &g_oilP, "OilP",  lv_color_hex(0x58A6FF),   0,  100, 135, 225,  -40, -10, -40,  10);
  lv_arc_set_mode(g_oilT.arc, LV_ARC_MODE_REVERSE);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		AHT20_Data a = AHT20_Get();
		uint16_t temp_adc = ADC_ReadAvg(16,&hadc1);
		uint16_t press_adc = ADC_ReadAvg(16,&hadc2);
		float oilTemp = ADC_To_Temperature(temp_adc);
		float oilPress = ADC_ToBar(press_adc);
		float egtTemp = 1100.0f; // replace with real

		Gauge_Update(&g_egt,  egtTemp,   (int32_t)egtTemp,         "%4.0f");
		Gauge_Update(&g_oilT, oilTemp,   (int32_t)oilTemp,         "%4.1f");
		Gauge_Update(&g_oilP, oilPress,  (int32_t)(oilPress * 10), "%4.1f");
		Gauge_Update(&g_amb,  a.temp,   (int32_t)(a.temp * 10),  "%4.1f");
		lv_timer_handler();
		HAL_Delay(5);
  }
  /* USER CODE END 3 */
}




static void Create_Gauge(lv_obj_t *parent, QuadGauge *g,
                          const char *title, lv_color_t color,
                          int32_t min, int32_t max,
                          uint16_t start_deg, uint16_t end_deg,
                          lv_coord_t title_dx, lv_coord_t title_dy,
                          lv_coord_t value_dx, lv_coord_t value_dy)
{

    g->arc = lv_arc_create(parent);
    lv_obj_remove_style_all(g->arc);
    lv_obj_set_size(g->arc, 200, 200);
    lv_obj_center(g->arc);
    lv_arc_set_bg_angles(g->arc, start_deg, end_deg);
    lv_arc_set_range(g->arc, min, max);
    lv_arc_set_value(g->arc, min);
    lv_obj_set_style_arc_width(g->arc, 12, LV_PART_MAIN);
    lv_obj_set_style_arc_width(g->arc, 12, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(g->arc, lv_color_hex(0x303030), LV_PART_MAIN);
    lv_obj_set_style_arc_color(g->arc, color, LV_PART_INDICATOR);

    g->title = lv_label_create(parent);
    lv_label_set_text(g->title, title);
    lv_obj_set_style_text_color(g->title, color, 0);
    lv_obj_align(g->title, LV_ALIGN_CENTER, title_dx, title_dy);

    g->value = lv_label_create(parent);
    lv_label_set_text(g->value, "--");
    lv_obj_set_style_text_color(g->value, lv_color_white(), 0);
    lv_obj_align(g->value, LV_ALIGN_CENTER, value_dx, value_dy);
}
static void Gauge_Update(QuadGauge *g, float val, int32_t arc_val, const char *fmt)
{
    char buf[16];
    snprintf(buf, sizeof(buf), fmt, val);
    lv_label_set_text(g->value, buf);
    lv_arc_set_value(g->arc, arc_val);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ILI_RESET_Pin|ILI_D_C_Pin|ILI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EGT_CS_Pin|EGT_DRDY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ILI_RESET_Pin ILI_D_C_Pin ILI_CS_Pin */
  GPIO_InitStruct.Pin = ILI_RESET_Pin|ILI_D_C_Pin|ILI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EGT_CS_Pin EGT_DRDY_Pin */
  GPIO_InitStruct.Pin = EGT_CS_Pin|EGT_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : EGT_FAULT_Pin */
  GPIO_InitStruct.Pin = EGT_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EGT_FAULT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
