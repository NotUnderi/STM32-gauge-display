/*
 * GC9A01.c
 *
 *  Created on: Nov 11, 2025
 *      Author: henrik
 */
#include <GC9A01.h>
#include "main.h"

#define GC9A01_SPI hspi1
extern SPI_HandleTypeDef hspi1;

#define GC9A01_CS_PORT     GPIOA
#define GC9A01_CS_PIN      GPIO_PIN_2

#define GC9A01_DC_PORT     GPIOA
#define GC9A01_DC_PIN      GPIO_PIN_1

#define GC9A01_RST_PORT    GPIOA
#define GC9A01_RST_PIN     GPIO_PIN_0

static uint16_t _width  = GC9A01_WIDTH;
static uint16_t _height = GC9A01_HEIGHT;


static void GC9A01_Select(void){ HAL_GPIO_WritePin(GC9A01_CS_PORT, GC9A01_CS_PIN, GPIO_PIN_RESET); }
static void GC9A01_Unselect(void){ HAL_GPIO_WritePin(GC9A01_CS_PORT, GC9A01_CS_PIN, GPIO_PIN_SET);}
static void GC9A01_DC_Command(void){HAL_GPIO_WritePin(GC9A01_DC_PORT, GC9A01_DC_PIN, GPIO_PIN_RESET);}
static void GC9A01_DC_Data(void){    HAL_GPIO_WritePin(GC9A01_DC_PORT, GC9A01_DC_PIN, GPIO_PIN_SET);}
static void GC9A01_Reset(void){
HAL_GPIO_WritePin(GC9A01_RST_PORT, GC9A01_RST_PIN, GPIO_PIN_RESET);
HAL_Delay(5);
HAL_GPIO_WritePin(GC9A01_RST_PORT, GC9A01_RST_PIN, GPIO_PIN_SET);
HAL_Delay(120);
}






static void GC9A01_WriteCommand(uint8_t cmd)
{
    GC9A01_Select();
    GC9A01_DC_Command();
    HAL_SPI_Transmit(&GC9A01_SPI, &cmd, 1, HAL_MAX_DELAY);
    GC9A01_Unselect();
}

static void GC9A01_WriteData(uint8_t data)
{
    GC9A01_Select();
    GC9A01_DC_Data();
    HAL_SPI_Transmit(&GC9A01_SPI, &data, 1, HAL_MAX_DELAY);
    GC9A01_Unselect();
}

static void GC9A01_WriteDataBuffer(uint8_t *buff, size_t len)
{
    GC9A01_Select();
    GC9A01_DC_Data();
    HAL_SPI_Transmit(&GC9A01_SPI, buff, len, HAL_MAX_DELAY);
    GC9A01_Unselect();
}
void GC9A01_Init(void)
{
    GC9A01_Unselect();
    GC9A01_Reset();

    uint8_t cmd;

    GC9A01_Select();
    GC9A01_DC_Command();

    cmd = 0xFE; HAL_SPI_Transmit(&GC9A01_SPI, &cmd, 1, HAL_MAX_DELAY);
    cmd = 0xEF; HAL_SPI_Transmit(&GC9A01_SPI, &cmd, 1, HAL_MAX_DELAY);
    GC9A01_Unselect();

    //Memory access control
    GC9A01_WriteCommand(0x36);
    GC9A01_WriteData(0x00);


    // Pixel format: 16 bits per pixel
    GC9A01_WriteCommand(0x3A);
    GC9A01_WriteData(0x05); // 16-bit

    GC9A01_WriteCommand(0x21); //inversion on

    GC9A01_WriteCommand(0x11); //Sleep out
    HAL_Delay(120);

    // Display ON
    GC9A01_WriteCommand(0x29);
    HAL_Delay(20);
}
static void GC9A01_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4];

    // Column addr set (0x2A)
    GC9A01_WriteCommand(0x2A);
    data[0] = (x0 >> 8);
    data[1] = (x0 & 0xFF);
    data[2] = (x1 >> 8);
    data[3] = (x1 & 0xFF);
    GC9A01_WriteDataBuffer(data, 4);

    // Row addr set (0x2B)
    GC9A01_WriteCommand(0x2B);
    data[0] = (y0 >> 8);
    data[1] = (y0 & 0xFF);
    data[2] = (y1 >> 8);
    data[3] = (y1 & 0xFF);
    GC9A01_WriteDataBuffer(data, 4);

    // Write to RAM
    GC9A01_WriteCommand(0x2C);
}





void GC9A01_SetRotation(uint8_t r)
{
	uint8_t madctl = 0;

	switch (r & 3) {
	case 0: madctl = 0x00; break;
	case 1: madctl = 0x60; break;
	case 2: madctl = 0xC0; break;
	case 3: madctl = 0xA0; break;
	}

	GC9A01_WriteCommand(0x36);
	GC9A01_WriteData(madctl);

	_width  = 240;
	_height = 240;

}


void ILI_flush(lv_display_t * disp, const lv_area_t * area, uint8_t *px_map){
    uint32_t w = (uint32_t)(area->x2 - area->x1 + 1);
    uint32_t h = (uint32_t)(area->y2 - area->y1 + 1);
    uint32_t len = w * h * 2;              // bytes, RGB565

    GC9A01_SetAddressWindow(area->x1, area->y1, area->x2, area->y2);

    GC9A01_Select();
    GC9A01_DC_Data();
    HAL_SPI_Transmit(&GC9A01_SPI, px_map, len, HAL_MAX_DELAY);
    GC9A01_Unselect();

    lv_display_flush_ready(disp);
}

