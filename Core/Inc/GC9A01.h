#ifndef GC9A01_H
#define GC9A01_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "lvgl.h"

/* Display size */
#define GC9A01_WIDTH   240
#define GC9A01_HEIGHT  240


#define GC9A01_MADCTL 0x36
#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_BGR 0x08

/* Public API */
void ILI_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * color_p);

void GC9A01_Init(void);
void GC9A01_FillScreen(uint16_t color);
void GC9A01_DrawPixel(int32_t x, int32_t y, uint16_t color);
void GC9A01_FillRect(uint16_t x, uint16_t y,
        uint16_t w, uint16_t h,
        uint16_t color);
void GC9A01_SetRotation(uint8_t r);
void GC9A01_DrawBitmap(uint16_t x, uint16_t y,
                        uint16_t w, uint16_t h,
                        const uint16_t *data);

extern SPI_HandleTypeDef hspi1;


#endif
