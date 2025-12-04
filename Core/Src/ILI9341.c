/*
 * ILI9341.c
 *
 *  Created on: Nov 11, 2025
 *      Author: henrik
 */
#include <ILI9341.h>
#include "main.h"

#define ILI9341_SPI hspi1
extern SPI_HandleTypeDef hspi1;

#define ILI9341_CS_PORT     GPIOA
#define ILI9341_CS_PIN      GPIO_PIN_2

#define ILI9341_DC_PORT     GPIOA
#define ILI9341_DC_PIN      GPIO_PIN_1

#define ILI9341_RST_PORT    GPIOA
#define ILI9341_RST_PIN     GPIO_PIN_0

static uint16_t _width  = ILI9341_WIDTH;
static uint16_t _height = ILI9341_HEIGHT;


static void ILI9341_Select(void){ HAL_GPIO_WritePin(ILI9341_CS_PORT, ILI9341_CS_PIN, GPIO_PIN_RESET); }
static void ILI9341_Unselect(void){ HAL_GPIO_WritePin(ILI9341_CS_PORT, ILI9341_CS_PIN, GPIO_PIN_SET);}
static void ILI9341_DC_Command(void){HAL_GPIO_WritePin(ILI9341_DC_PORT, ILI9341_DC_PIN, GPIO_PIN_RESET);}
static void ILI9341_DC_Data(void){    HAL_GPIO_WritePin(ILI9341_DC_PORT, ILI9341_DC_PIN, GPIO_PIN_SET);}
static void ILI9341_Reset(void){
HAL_GPIO_WritePin(ILI9341_RST_PORT, ILI9341_RST_PIN, GPIO_PIN_RESET);
HAL_Delay(5);
HAL_GPIO_WritePin(ILI9341_RST_PORT, ILI9341_RST_PIN, GPIO_PIN_SET);
HAL_Delay(120);
}






static void ILI9341_WriteCommand(uint8_t cmd)
{
    ILI9341_Select();
    ILI9341_DC_Command();
    HAL_SPI_Transmit(&ILI9341_SPI, &cmd, 1, HAL_MAX_DELAY);
    ILI9341_Unselect();
}

static void ILI9341_WriteData(uint8_t data)
{
    ILI9341_Select();
    ILI9341_DC_Data();
    HAL_SPI_Transmit(&ILI9341_SPI, &data, 1, HAL_MAX_DELAY);
    ILI9341_Unselect();
}

static void ILI9341_WriteDataBuffer(uint8_t *buff, size_t len)
{
    ILI9341_Select();
    ILI9341_DC_Data();
    HAL_SPI_Transmit(&ILI9341_SPI, buff, len, HAL_MAX_DELAY);
    ILI9341_Unselect();
}
void ILI9341_Init(void)
{
    ILI9341_Unselect();
    ILI9341_Reset();

    // Software reset
    ILI9341_WriteCommand(0x01);
    HAL_Delay(5);

    // Sleep out
    ILI9341_WriteCommand(0x11);
    HAL_Delay(120);

    // Pixel format: 16 bits per pixel
    ILI9341_WriteCommand(0x3A);
    ILI9341_WriteData(0x55); // 16-bit

    // Memory Access Control (orientation)
    // 0x48: MX=0, MY=1, MV=0, BGR=1 → typical portrait 240x320 BGR
    ILI9341_WriteCommand(0x36);
    ILI9341_WriteData(0x48);

    // Display ON
    ILI9341_WriteCommand(0x29);
    HAL_Delay(20);
}
static void ILI9341_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4];

    // Column addr set (0x2A)
    ILI9341_WriteCommand(0x2A);
    data[0] = (x0 >> 8);
    data[1] = (x0 & 0xFF);
    data[2] = (x1 >> 8);
    data[3] = (x1 & 0xFF);
    ILI9341_WriteDataBuffer(data, 4);

    // Row addr set (0x2B)
    ILI9341_WriteCommand(0x2B);
    data[0] = (y0 >> 8);
    data[1] = (y0 & 0xFF);
    data[2] = (y1 >> 8);
    data[3] = (y1 & 0xFF);
    ILI9341_WriteDataBuffer(data, 4);

    // Write to RAM
    ILI9341_WriteCommand(0x2C);
}
void ILI9341_DrawPixel(int32_t x, int32_t y, uint16_t color)
{
    if (x >= _width || y >= _height) return;
    uint8_t data[2] = { color >> 8, color & 0xFF };
    ILI9341_SetAddressWindow(x, y, x, y);
    ILI9341_Select();
    ILI9341_DC_Data();
    HAL_SPI_Transmit(&ILI9341_SPI, data, 2, HAL_MAX_DELAY);
    ILI9341_Unselect();
}





void ILI9341_SetRotation(uint8_t r)
{
    uint8_t madctl = 0;

    switch (r & 3) {
    case 0: // Portrait 240x320
        madctl = MADCTL_MX | MADCTL_BGR;                               // 0x48
        _width  = 240;
        _height = 320;
        break;

    case 1: // Landscape 320x240
        madctl = MADCTL_MV | MADCTL_BGR;                               // 0x28
        _width  = 320;
        _height = 240;
        break;

    case 2: // Portrait 180°
        madctl = MADCTL_MY | MADCTL_BGR;                               // 0x88
        _width  = 240;
        _height = 320;
        break;

    case 3: // Landscape 180°
        madctl = MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR;       // 0xE8
        _width  = 320;
        _height = 240;
        break;
    }

    ILI9341_WriteCommand(ILI9341_MADCTL);
    ILI9341_WriteData(madctl);
}


void ILI_flush(lv_display_t * disp, const lv_area_t * area, uint8_t *px_map){
    uint32_t w = (uint32_t)(area->x2 - area->x1 + 1);
    uint32_t h = (uint32_t)(area->y2 - area->y1 + 1);
    uint32_t len = w * h * 2;              // bytes, RGB565

    ILI9341_SetAddressWindow(area->x1, area->y1, area->x2, area->y2);

    ILI9341_Select();
    ILI9341_DC_Data();
    HAL_SPI_Transmit(&ILI9341_SPI, px_map, len, HAL_MAX_DELAY);
    ILI9341_Unselect();

    lv_display_flush_ready(disp);
}

