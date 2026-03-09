#ifndef GC9A01_H
#define GC9A01_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "lvgl.h"

/* Display size */
#define GC9A01_WIDTH   240
#define GC9A01_HEIGHT  240

#define GC9A01_SPI hspi1
extern SPI_HandleTypeDef hspi1;

#define GC9A01_CS_PORT     GPIOA
#define GC9A01_CS_PIN      GPIO_PIN_2

#define GC9A01_DC_PORT     GPIOA
#define GC9A01_DC_PIN      GPIO_PIN_1

#define GC9A01_RST_PORT    GPIOA
#define GC9A01_RST_PIN     GPIO_PIN_0

#define GC9A01_MADCTL 0x36
#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_BGR 0x08

/* Public API */
void ILI_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * color_p);
void GC9A01_Init(void);
void GC9A01_Init_Bodmer(void);

#endif
