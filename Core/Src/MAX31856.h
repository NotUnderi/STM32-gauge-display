/*
 * MAX31856.h
 *
 *  Created on: Mar 1, 2026
 *      Author: henrik
 */

#ifndef MAX31856_H
#define MAX31856_H

#include "stm32f4xx_hal.h"

/* Register map */
#define MAX31856_CR0_REG        0x00	//Configuration 0 Register
#define MAX31856_CR1_REG        0x01	//Configuration 1 Register
#define MAX31856_MASK_REG       0x02	//Fault Mask Register
#define MAX31856_CJHF_REG       0x03	//Cold-Junction High Fault Threshold
#define MAX31856_CJLF_REG       0x04	//Cold-Junction Low Fault Threshold
#define MAX31856_LTHFTH_REG     0x05	//Linearized Temperature High Fault Threshold MSB
#define MAX31856_LTHFTL_REG     0x06	//Linearized Temperature High Fault Threshold LSB
#define MAX31856_LTLFTH_REG     0x07	//Linearized Temperature Low Fault Threshold MSB
#define MAX31856_LTLFTL_REG     0x08	//Linearized Temperature Low Fault Threshold LSB
#define MAX31856_CJTO_REG       0x09	//Cold-Junction Temperature Offset Register
#define MAX31856_CJTH_REG       0x0A	//Cold-Junction Temperature Register, MSB
#define MAX31856_CJTL_REG       0x0B	//Cold-Junction Temperature Register, LSB
#define MAX31856_LTCBH_REG      0x0C	//Linearized TC Temperature, Byte 2
#define MAX31856_LTCBM_REG      0x0D	//Linearized TC Temperature, Byte 1
#define MAX31856_LTCBL_REG      0x0E	//Linearized TC Temperature, Byte 0
#define MAX31856_SR_REG         0x0F	//Fault Status Register

/* Public API */
void MAX31856_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef MAX31856_WriteReg(uint8_t reg, uint8_t value);
HAL_StatusTypeDef MAX31856_ReadReg(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef MAX31856_ReadRegs(uint8_t reg, uint8_t *buffer, uint8_t len);

float MAX31856_ReadThermocoupleTemp(void);
float MAX31856_ReadColdJunctionTemp(void);
uint8_t MAX31856_ReadFault(void);

#endif
