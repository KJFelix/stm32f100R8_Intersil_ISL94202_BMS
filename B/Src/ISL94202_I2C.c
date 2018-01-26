#include "ISL94202_I2C.h"                  // Device header
extern I2C_HandleTypeDef hi2c2;

uint8_t I2C_readReg8(uint8_t slave, uint8_t add)
{
	uint8_t ret=0;
	HAL_I2C_Master_Transmit(&hi2c2, slave, &add, 1, 100 );
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c2,slave,&ret, 1, 100);
	return ret;
}

uint16_t I2C_readReg16(uint8_t slave, uint8_t add)
{
	uint16_t ret=0;
	HAL_I2C_Master_Transmit(&hi2c2, slave, &add, 1, 100 );
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c2,slave, (uint8_t *)&ret, 2, 100);
	return ret;
}

void I2C_readMany(uint8_t slave, uint8_t add, uint8_t count, uint8_t* buffer)
{
		
	

		HAL_I2C_Master_Transmit(&hi2c2,slave,&add,1,100);
		
		HAL_Delay(20);
		
		HAL_I2C_Master_Receive(&hi2c2,slave,buffer,count,100);
}

void I2C_writeReg8(uint8_t slave, uint8_t add, uint8_t data)
{
	static uint8_t buffer[2];
  buffer[0]=add;
  buffer[1]=data;
	  HAL_I2C_Master_Transmit(&hi2c2,slave,buffer,2,100);
    HAL_Delay(20);


}
void I2C_writeReg16(uint8_t slave, uint8_t add, uint16_t data)
{
	static uint16_t buffer[2];
  buffer[0]=add;
  buffer[1]=data;
   HAL_I2C_Master_Transmit(&hi2c2,slave,(uint8_t *)buffer,2,100);
//		HAL_Delay(20);
//	  HAL_I2C_Master_Transmit(&hi2c2,slave,(uint8_t *)&data,2,100);
//    HAL_Delay(20);

}
