#ifndef ILI9341_H
#define ILI9341_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "lvgl.h"

/* Display size */
#define ILI9341_WIDTH   240
#define ILI9341_HEIGHT  320


#define ILI9341_MADCTL 0x36
#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_BGR 0x08

/* Public API */
void ILI_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * color_p);

void ILI9341_Init(void);
void ILI9341_FillScreen(uint16_t color);
void ILI9341_DrawPixel(int32_t x, int32_t y, uint16_t color);
void ILI9341_FillRect(uint16_t x, uint16_t y,
        uint16_t w, uint16_t h,
        uint16_t color);
void ILI9341_SetRotation(uint8_t r);
void ILI9341_DrawBitmap(uint16_t x, uint16_t y,
                        uint16_t w, uint16_t h,
                        const uint16_t *data);

extern SPI_HandleTypeDef hspi1;


#endif
