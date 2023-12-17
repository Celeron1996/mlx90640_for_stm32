/**
  ******************************************************************************
  * @file    MLX90640_I2C_Driver.c
  * @author  zhusl
  * @brief   MLX90640_I2C_Driver
  ******************************************************************************
  * @attention
  *
  * I2C 写数据都是以字为单位，也就是两个字节
  * I2C 写数据或地址，先写高八位，再写第八位
  *
  ******************************************************************************
  */
#include "MLX90640_I2C_Driver.h"
#include "stm32f4xx_hal.h"




#define mlx90640_i2c_handl hi2c1
extern I2C_HandleTypeDef hi2c1;


void MLX90640_I2CInit(void)
{
	while (HAL_I2C_GetState(&mlx90640_i2c_handl) != HAL_I2C_STATE_READY);
}


int MLX90640_I2CGeneralReset(void)
{
	uint8_t reset_value = 0x06;
	
	if (HAL_I2C_Master_Transmit(&mlx90640_i2c_handl, 0x00, &reset_value, 1, 200) != HAL_OK)
	{
		return -1;
	}

	return MLX90640_NO_ERROR;
}


int MLX90640_I2CRead(uint8_t slaveAddr,uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{

	uint8_t* p = (uint8_t*) data;

	int ack = 0;                               
	int cnt = 0;
	
	ack = HAL_I2C_Mem_Read(&mlx90640_i2c_handl, (slaveAddr << 1), startAddress, I2C_MEMADD_SIZE_16BIT, p, nMemAddressRead*2, 500);

	if (ack != HAL_OK)
	{
			return -1;
	}
	

	for(cnt=0; cnt < nMemAddressRead*2; cnt+=2) {
		uint8_t tempBuffer = p[cnt+1];
		p[cnt+1] = p[cnt];
		p[cnt] = tempBuffer;
	}

	return 0;   
} 



int MLX90640_I2CWrite(uint8_t slaveAddr,uint16_t writeAddress, uint16_t data)
{
	int ack = 0;
	uint8_t cmd[2];
	static uint16_t dataCheck;

	cmd[0] = data >> 8;
	cmd[1] = data & 0x00FF;


	ack = HAL_I2C_Mem_Write(&mlx90640_i2c_handl, (slaveAddr << 1), writeAddress, I2C_MEMADD_SIZE_16BIT, cmd, sizeof(cmd), 500);

	if (ack != HAL_OK)
	{
			return -1;
	}         
	
	MLX90640_I2CRead(slaveAddr,writeAddress,1, &dataCheck);
	
	if ( dataCheck != data)
	{
			return -2;
	}    
	
	return 0;
}





void MLX90640_I2CFreqSet(int freq)
{
	freq = freq;
}

