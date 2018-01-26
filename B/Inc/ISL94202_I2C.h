#ifndef __ISL94202_I2C_H__
#define  __ISL94202_I2C_H__
#include "stm32f1xx_hal.h"





void I2C_readMany(uint8_t slave, uint8_t add, uint8_t count, uint8_t* buffer);

uint8_t I2C_readReg8(uint8_t slave, uint8_t add);

uint16_t I2C_readReg16(uint8_t slave, uint8_t add);

void I2C_writeReg8(uint8_t slave, uint8_t add, uint8_t data);

void I2C_writeReg16(uint8_t slave, uint8_t add, uint16_t data);

#endif
