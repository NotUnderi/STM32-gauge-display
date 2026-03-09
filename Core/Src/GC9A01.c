/*
 * GC9A01.c
 *
 *  Created on: Nov 11, 2025
 *      Author: henrik
 */
#include <GC9A01.h>
#include "main.h"

static void GC9A01_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);


static void GC9A01_Select(void){ HAL_GPIO_WritePin(GC9A01_CS_PORT, GC9A01_CS_PIN, GPIO_PIN_RESET); }
static void GC9A01_Unselect(void){ HAL_GPIO_WritePin(GC9A01_CS_PORT, GC9A01_CS_PIN, GPIO_PIN_SET);}
static void GC9A01_DC_Command(void){HAL_GPIO_WritePin(GC9A01_DC_PORT, GC9A01_DC_PIN, GPIO_PIN_RESET);}
static void GC9A01_DC_Data(void){    HAL_GPIO_WritePin(GC9A01_DC_PORT, GC9A01_DC_PIN, GPIO_PIN_SET);}
static void GC9A01_Reset(void){
HAL_GPIO_WritePin(GC9A01_RST_PORT, GC9A01_RST_PIN, GPIO_PIN_RESET);
HAL_Delay(10);
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

void GC9A01_Init_Bodmer(void){
	  GC9A01_WriteCommand(0xEF);
	  GC9A01_WriteCommand(0xEB);
	  GC9A01_WriteData(0x14);

	  GC9A01_WriteCommand(0xFE);
	  GC9A01_WriteCommand(0xEF);

	  GC9A01_WriteCommand(0xEB);
	  GC9A01_WriteData(0x14);

	  GC9A01_WriteCommand(0x84);
	  GC9A01_WriteData(0x40);

	  GC9A01_WriteCommand(0x85);
	  GC9A01_WriteData(0xFF);

	  GC9A01_WriteCommand(0x86);
	  GC9A01_WriteData(0xFF);

	  GC9A01_WriteCommand(0x87);
	  GC9A01_WriteData(0xFF);

	  GC9A01_WriteCommand(0x88);
	  GC9A01_WriteData(0x0A);

	  GC9A01_WriteCommand(0x89);
	  GC9A01_WriteData(0x21);

	  GC9A01_WriteCommand(0x8A);
	  GC9A01_WriteData(0x00);

	  GC9A01_WriteCommand(0x8B);
	  GC9A01_WriteData(0x80);

	  GC9A01_WriteCommand(0x8C);
	  GC9A01_WriteData(0x01);

	  GC9A01_WriteCommand(0x8D);
	  GC9A01_WriteData(0x01);

	  GC9A01_WriteCommand(0x8E);
	  GC9A01_WriteData(0xFF);

	  GC9A01_WriteCommand(0x8F);
	  GC9A01_WriteData(0xFF);

	  GC9A01_WriteCommand(0xB6);
	  GC9A01_WriteData(0x00);
	  GC9A01_WriteData(0x20);

	  GC9A01_WriteCommand(0x3A);
	  GC9A01_WriteData(0x05);

	  GC9A01_WriteCommand(0x90);
	  GC9A01_WriteData(0x08);
	  GC9A01_WriteData(0x08);
	  GC9A01_WriteData(0x08);
	  GC9A01_WriteData(0x08);

	  GC9A01_WriteCommand(0xBD);
	  GC9A01_WriteData(0x06);

	  GC9A01_WriteCommand(0xBC);
	  GC9A01_WriteData(0x00);

	  GC9A01_WriteCommand(0xFF);
	  GC9A01_WriteData(0x60);
	  GC9A01_WriteData(0x01);
	  GC9A01_WriteData(0x04);

	  GC9A01_WriteCommand(0xC3);
	  GC9A01_WriteData(0x13);
	  GC9A01_WriteCommand(0xC4);
	  GC9A01_WriteData(0x13);

	  GC9A01_WriteCommand(0xC9);
	  GC9A01_WriteData(0x22);

	  GC9A01_WriteCommand(0xBE);
	  GC9A01_WriteData(0x11);

	  GC9A01_WriteCommand(0xE1);
	  GC9A01_WriteData(0x10);
	  GC9A01_WriteData(0x0E);

	  GC9A01_WriteCommand(0xDF);
	  GC9A01_WriteData(0x21);
	  GC9A01_WriteData(0x0c);
	  GC9A01_WriteData(0x02);

	  GC9A01_WriteCommand(0xF0);
	  GC9A01_WriteData(0x45);
	  GC9A01_WriteData(0x09);
	  GC9A01_WriteData(0x08);
	  GC9A01_WriteData(0x08);
	  GC9A01_WriteData(0x26);
	  GC9A01_WriteData(0x2A);

	  GC9A01_WriteCommand(0xF1);
	  GC9A01_WriteData(0x43);
	  GC9A01_WriteData(0x70);
	  GC9A01_WriteData(0x72);
	  GC9A01_WriteData(0x36);
	  GC9A01_WriteData(0x37);
	  GC9A01_WriteData(0x6F);

	  GC9A01_WriteCommand(0xF2);
	  GC9A01_WriteData(0x45);
	  GC9A01_WriteData(0x09);
	  GC9A01_WriteData(0x08);
	  GC9A01_WriteData(0x08);
	  GC9A01_WriteData(0x26);
	  GC9A01_WriteData(0x2A);

	  GC9A01_WriteCommand(0xF3);
	  GC9A01_WriteData(0x43);
	  GC9A01_WriteData(0x70);
	  GC9A01_WriteData(0x72);
	  GC9A01_WriteData(0x36);
	  GC9A01_WriteData(0x37);
	  GC9A01_WriteData(0x6F);

	  GC9A01_WriteCommand(0xED);
	  GC9A01_WriteData(0x1B);
	  GC9A01_WriteData(0x0B);

	  GC9A01_WriteCommand(0xAE);
	  GC9A01_WriteData(0x77);

	  GC9A01_WriteCommand(0xCD);
	  GC9A01_WriteData(0x63);

	  GC9A01_WriteCommand(0x70);
	  GC9A01_WriteData(0x07);
	  GC9A01_WriteData(0x07);
	  GC9A01_WriteData(0x04);
	  GC9A01_WriteData(0x0E);
	  GC9A01_WriteData(0x0F);
	  GC9A01_WriteData(0x09);
	  GC9A01_WriteData(0x07);
	  GC9A01_WriteData(0x08);
	  GC9A01_WriteData(0x03);

	  GC9A01_WriteCommand(0xE8);
	  GC9A01_WriteData(0x34);

	  GC9A01_WriteCommand(0x62);
	  GC9A01_WriteData(0x18);
	  GC9A01_WriteData(0x0D);
	  GC9A01_WriteData(0x71);
	  GC9A01_WriteData(0xED);
	  GC9A01_WriteData(0x70);
	  GC9A01_WriteData(0x70);
	  GC9A01_WriteData(0x18);
	  GC9A01_WriteData(0x0F);
	  GC9A01_WriteData(0x71);
	  GC9A01_WriteData(0xEF);
	  GC9A01_WriteData(0x70);
	  GC9A01_WriteData(0x70);

	  GC9A01_WriteCommand(0x63);
	  GC9A01_WriteData(0x18);
	  GC9A01_WriteData(0x11);
	  GC9A01_WriteData(0x71);
	  GC9A01_WriteData(0xF1);
	  GC9A01_WriteData(0x70);
	  GC9A01_WriteData(0x70);
	  GC9A01_WriteData(0x18);
	  GC9A01_WriteData(0x13);
	  GC9A01_WriteData(0x71);
	  GC9A01_WriteData(0xF3);
	  GC9A01_WriteData(0x70);
	  GC9A01_WriteData(0x70);

	  GC9A01_WriteCommand(0x64);
	  GC9A01_WriteData(0x28);
	  GC9A01_WriteData(0x29);
	  GC9A01_WriteData(0xF1);
	  GC9A01_WriteData(0x01);
	  GC9A01_WriteData(0xF1);
	  GC9A01_WriteData(0x00);
	  GC9A01_WriteData(0x07);

	  GC9A01_WriteCommand(0x66);
	  GC9A01_WriteData(0x3C);
	  GC9A01_WriteData(0x00);
	  GC9A01_WriteData(0xCD);
	  GC9A01_WriteData(0x67);
	  GC9A01_WriteData(0x45);
	  GC9A01_WriteData(0x45);
	  GC9A01_WriteData(0x10);
	  GC9A01_WriteData(0x00);
	  GC9A01_WriteData(0x00);
	  GC9A01_WriteData(0x00);

	  GC9A01_WriteCommand(0x67);
	  GC9A01_WriteData(0x00);
	  GC9A01_WriteData(0x3C);
	  GC9A01_WriteData(0x00);
	  GC9A01_WriteData(0x00);
	  GC9A01_WriteData(0x00);
	  GC9A01_WriteData(0x01);
	  GC9A01_WriteData(0x54);
	  GC9A01_WriteData(0x10);
	  GC9A01_WriteData(0x32);
	  GC9A01_WriteData(0x98);

	  GC9A01_WriteCommand(0x74);
	  GC9A01_WriteData(0x10);
	  GC9A01_WriteData(0x85);
	  GC9A01_WriteData(0x80);
	  GC9A01_WriteData(0x00);
	  GC9A01_WriteData(0x00);
	  GC9A01_WriteData(0x4E);
	  GC9A01_WriteData(0x00);

	  GC9A01_WriteCommand(0x98);
	  GC9A01_WriteData(0x3e);
	  GC9A01_WriteData(0x07);

	  GC9A01_WriteCommand(0x35);
	  GC9A01_WriteCommand(0x21);

	  GC9A01_WriteCommand(0x11);
	  HAL_Delay(120);
	  GC9A01_WriteCommand(0x29);
	  HAL_Delay(20);

	    // Fill entire screen red
	    GC9A01_SetAddressWindow(0, 0, 239, 239);
	    GC9A01_Select();
	    GC9A01_DC_Data();
	    for (int i = 0; i < 240*240; i++) {
	        uint8_t hi = 0xF8, lo = 0x00; // Red in RGB565
	        HAL_SPI_Transmit(&GC9A01_SPI, &hi, 1, HAL_MAX_DELAY);
	        HAL_SPI_Transmit(&GC9A01_SPI, &lo, 1, HAL_MAX_DELAY);
	    }
	    GC9A01_Unselect();

	}
void GC9A01_Init(void)
{
    GC9A01_Unselect();
    GC9A01_Reset();

	GC9A01_WriteCommand(0xEF);

	GC9A01_WriteCommand(0xEB);
	GC9A01_WriteData(0x14);

	GC9A01_WriteCommand(0xFE);
	GC9A01_WriteCommand(0xEF);

	GC9A01_WriteCommand(0xEB);
	GC9A01_WriteData(0x14);

	GC9A01_WriteCommand(0x84);
	GC9A01_WriteData(0x40);

	GC9A01_WriteCommand(0x85);
	GC9A01_WriteData(0xFF);

	GC9A01_WriteCommand(0x86);
	GC9A01_WriteData(0xFF);

	GC9A01_WriteCommand(0x87);
	GC9A01_WriteData(0xFF);

	GC9A01_WriteCommand(0x88);
	GC9A01_WriteData(0x0A);

	GC9A01_WriteCommand(0x89);
	GC9A01_WriteData(0x21);

	GC9A01_WriteCommand(0x8A);
	GC9A01_WriteData(0x00);

	GC9A01_WriteCommand(0x8B);
	GC9A01_WriteData(0x80);

	GC9A01_WriteCommand(0x8C);
	GC9A01_WriteData(0x01);

	GC9A01_WriteCommand(0x8D);
	GC9A01_WriteData(0x01);

	GC9A01_WriteCommand(0x8E);
	GC9A01_WriteData(0xFF);

	GC9A01_WriteCommand(0x8F);
	GC9A01_WriteData(0xFF);

	// Display Function Control
	GC9A01_WriteCommand(0xB6);
	GC9A01_WriteData(0x00);
	GC9A01_WriteData(0x20);  // change to 0x00 if image is upside down

	// Pixel format: RGB565
	GC9A01_WriteCommand(0x3A);
	GC9A01_WriteData(0x05);

	GC9A01_WriteCommand(0x90);
	GC9A01_WriteData(0x08);
	GC9A01_WriteData(0x08);
	GC9A01_WriteData(0x08);
	GC9A01_WriteData(0x08);

	GC9A01_WriteCommand(0xBD);
	GC9A01_WriteData(0x06);

	GC9A01_WriteCommand(0xBC);
	GC9A01_WriteData(0x00);

	GC9A01_WriteCommand(0xFF);
	GC9A01_WriteData(0x60);
	GC9A01_WriteData(0x01);
	GC9A01_WriteData(0x04);

	// VRH1 / VRH2 — supply voltage
	GC9A01_WriteCommand(0xC3);
	GC9A01_WriteData(0x13);

	GC9A01_WriteCommand(0xC4);
	GC9A01_WriteData(0x13);

	GC9A01_WriteCommand(0xC9);
	GC9A01_WriteData(0x22);

	GC9A01_WriteCommand(0xBE);
	GC9A01_WriteData(0x11);

	GC9A01_WriteCommand(0xE1);
	GC9A01_WriteData(0x10);
	GC9A01_WriteData(0x0E);

	GC9A01_WriteCommand(0xDF);
	GC9A01_WriteData(0x21);
	GC9A01_WriteData(0x0C);
	GC9A01_WriteData(0x02);

	// Gamma correction (positive)
	GC9A01_WriteCommand(0xF0);
	GC9A01_WriteData(0x45);
	GC9A01_WriteData(0x09);
	GC9A01_WriteData(0x08);
	GC9A01_WriteData(0x08);
	GC9A01_WriteData(0x26);
	GC9A01_WriteData(0x2A);

	// Gamma correction (negative)
	GC9A01_WriteCommand(0xF1);
	GC9A01_WriteData(0x43);
	GC9A01_WriteData(0x70);
	GC9A01_WriteData(0x72);
	GC9A01_WriteData(0x36);
	GC9A01_WriteData(0x37);
	GC9A01_WriteData(0x6F);

	GC9A01_WriteCommand(0xF2);
	GC9A01_WriteData(0x45);
	GC9A01_WriteData(0x09);
	GC9A01_WriteData(0x08);
	GC9A01_WriteData(0x08);
	GC9A01_WriteData(0x26);
	GC9A01_WriteData(0x2A);

	GC9A01_WriteCommand(0xF3);
	GC9A01_WriteData(0x43);
	GC9A01_WriteData(0x70);
	GC9A01_WriteData(0x72);
	GC9A01_WriteData(0x36);
	GC9A01_WriteData(0x37);
	GC9A01_WriteData(0x6F);

	GC9A01_WriteCommand(0xED);
	GC9A01_WriteData(0x1B);
	GC9A01_WriteData(0x0B);

	GC9A01_WriteCommand(0xAE);
	GC9A01_WriteData(0x77);

	GC9A01_WriteCommand(0xCD);
	GC9A01_WriteData(0x63);

	GC9A01_WriteCommand(0x70);
	GC9A01_WriteData(0x07);
	GC9A01_WriteData(0x07);
	GC9A01_WriteData(0x04);
	GC9A01_WriteData(0x0E);
	GC9A01_WriteData(0x0F);
	GC9A01_WriteData(0x09);
	GC9A01_WriteData(0x07);
	GC9A01_WriteData(0x08);
	GC9A01_WriteData(0x03);

	GC9A01_WriteCommand(0xE8);
	GC9A01_WriteData(0x34);

	GC9A01_WriteCommand(0x62);
	GC9A01_WriteData(0x18); GC9A01_WriteData(0x0D);
	GC9A01_WriteData(0x71); GC9A01_WriteData(0xED);
	GC9A01_WriteData(0x70); GC9A01_WriteData(0x70);
	GC9A01_WriteData(0x18); GC9A01_WriteData(0x0F);
	GC9A01_WriteData(0x71); GC9A01_WriteData(0xEF);
	GC9A01_WriteData(0x70); GC9A01_WriteData(0x70);

	GC9A01_WriteCommand(0x63);
	GC9A01_WriteData(0x18); GC9A01_WriteData(0x11);
	GC9A01_WriteData(0x71); GC9A01_WriteData(0xF1);
	GC9A01_WriteData(0x70); GC9A01_WriteData(0x70);
	GC9A01_WriteData(0x18); GC9A01_WriteData(0x13);
	GC9A01_WriteData(0x71); GC9A01_WriteData(0xF3);
	GC9A01_WriteData(0x70); GC9A01_WriteData(0x70);

	GC9A01_WriteCommand(0x64);
	GC9A01_WriteData(0x28); GC9A01_WriteData(0x29);
	GC9A01_WriteData(0xF1); GC9A01_WriteData(0x01);
	GC9A01_WriteData(0xF1); GC9A01_WriteData(0x00);
	GC9A01_WriteData(0x07);

	GC9A01_WriteCommand(0x66);
	GC9A01_WriteData(0x3C); GC9A01_WriteData(0x00);
	GC9A01_WriteData(0xCD); GC9A01_WriteData(0x67);
	GC9A01_WriteData(0x45); GC9A01_WriteData(0x45);
	GC9A01_WriteData(0x10); GC9A01_WriteData(0x00);
	GC9A01_WriteData(0x00); GC9A01_WriteData(0x00);

	GC9A01_WriteCommand(0x67);
	GC9A01_WriteData(0x00); GC9A01_WriteData(0x3C);
	GC9A01_WriteData(0x00); GC9A01_WriteData(0x00);
	GC9A01_WriteData(0x00); GC9A01_WriteData(0x01);
	GC9A01_WriteData(0x54); GC9A01_WriteData(0x10);
	GC9A01_WriteData(0x32); GC9A01_WriteData(0x98);

	GC9A01_WriteCommand(0x74);
	GC9A01_WriteData(0x10); GC9A01_WriteData(0x85);
	GC9A01_WriteData(0x80); GC9A01_WriteData(0x00);
	GC9A01_WriteData(0x00); GC9A01_WriteData(0x4E);
	GC9A01_WriteData(0x00);

	GC9A01_WriteCommand(0x98);
	GC9A01_WriteData(0x3E);
	GC9A01_WriteData(0x07);

	// Tearing effect line ON (optional but good practice)
	GC9A01_WriteCommand(0x35);



    //Memory access control
    //GC9A01_WriteCommand(0x36);
    //GC9A01_WriteData(0x00); 			//rotation

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

