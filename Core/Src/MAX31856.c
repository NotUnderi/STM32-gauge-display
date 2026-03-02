/*
 * MAX31856.c
 *
 *  Created on: Mar 1, 2026
 *      Author: henrik
 */
#include "MAX31856.h"

#define MAX31856_CS_PORT     GPIOA
#define MAX31856_CS_PIN      GPIO_PIN_2

extern SPI_HandleTypeDef hspi1;   // Change if needed
static SPI_HandleTypeDef *max_spi;

/* Private helpers */

static void MAX31856_CS_Low(void)
{
    HAL_GPIO_WritePin(MAX31856_CS_PORT, MAX31856_CS_PIN, GPIO_PIN_RESET);
}

static void MAX31856_CS_High(void)
{
    HAL_GPIO_WritePin(MAX31856_CS_PORT, MAX31856_CS_PIN, GPIO_PIN_SET);
}

/* Init */

void MAX31856_Init(SPI_HandleTypeDef *hspi)
{
    max_spi = hspi;

    MAX31856_CS_High();

    // CR0: Auto conversion, 50Hz filter
    MAX31856_WriteReg(MAX31856_CR0_REG, 0x80);

    // CR1: Type K thermocouple (default 0x03)
    MAX31856_WriteReg(MAX31856_CR1_REG, 0x03);
}

/* Write single register */
HAL_StatusTypeDef MAX31856_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];

    tx[0] = reg | 0x80;   // MSB = 1 for write
    tx[1] = value;

    MAX31856_CS_Low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(max_spi, tx, 2, HAL_MAX_DELAY);
    MAX31856_CS_High();

    return status;
}

/* Read single register */
HAL_StatusTypeDef MAX31856_ReadReg(uint8_t reg, uint8_t *value)
{
    uint8_t addr = reg & 0x7F;

    MAX31856_CS_Low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(max_spi, &addr, 1, HAL_MAX_DELAY);
    if (status == HAL_OK)
        status = HAL_SPI_Receive(max_spi, value, 1, HAL_MAX_DELAY);
    MAX31856_CS_High();

    return status;
}

/* Read multiple registers */
HAL_StatusTypeDef MAX31856_ReadRegs(uint8_t reg, uint8_t *buffer, uint8_t len)
{
    uint8_t addr = reg & 0x7F;

    MAX31856_CS_Low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(max_spi, &addr, 1, HAL_MAX_DELAY);
    if (status == HAL_OK)
        status = HAL_SPI_Receive(max_spi, buffer, len, HAL_MAX_DELAY);
    MAX31856_CS_High();

    return status;
}

/* Read thermocouple temperature (°C) */
float MAX31856_ReadThermocoupleTemp(void)
{
    uint8_t data[3];
    int32_t temp_raw = 0;

    if (MAX31856_ReadRegs(MAX31856_LTCBH_REG, data, 3) != HAL_OK)
        return -1000.0f;

    temp_raw = ((int32_t)data[0] << 16) |
               ((int32_t)data[1] << 8)  |
               data[2];

    temp_raw >>= 5;  // 19-bit signed

    if (temp_raw & 0x00080000)
        temp_raw |= 0xFFF00000;

    return temp_raw * 0.0078125f;  // 1/128
}

/* Read cold junction temperature (°C) */
float MAX31856_ReadColdJunctionTemp(void)
{
    uint8_t data[2];
    int16_t temp_raw;

    if (MAX31856_ReadRegs(MAX31856_CJTH_REG, data, 2) != HAL_OK)
        return -1000.0f;

    temp_raw = ((int16_t)data[0] << 8) | data[1];
    temp_raw >>= 2;

    return temp_raw * 0.015625f; // 1/64
}

/* Read fault status */
uint8_t MAX31856_ReadFault(void)
{
    uint8_t fault;
    if (MAX31856_ReadReg(MAX31856_SR_REG, &fault) != HAL_OK)
        return 0xFF;

    return fault;
}
