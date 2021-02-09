/*
 * MPU9250.c
 *
 *  Created on: 15 janv. 2021
 *      Author: abdou
 */


#include "MPU9250.h"

// Write the MPU's data registers low speed spi 1MHz is always selected
MPU9250_Result_t MPU9250_Write_Register(MPU9250_CONFIG_t *MPU9250_CONFIG, MPU9250_CONFIGURATION_REGISTERS address, uint8_t data)
{
    uint8_t buf[2] = {address, data}; // buf[0] for register selection read/write buf[1] for data
    HAL_StatusTypeDef res;
    HAL_GPIO_WritePin(MPU9250_CONFIG->GPIOx,MPU9250_CONFIG->GPIO_PIN , 0);
    res = HAL_SPI_Transmit(MPU9250_CONFIG->hspi,&buf[0],1,10);
    res = HAL_SPI_Transmit(MPU9250_CONFIG->hspi,&buf[1],1,10);
    HAL_GPIO_WritePin(MPU9250_CONFIG->GPIOx,MPU9250_CONFIG->GPIO_PIN , 1);
    //Exception
    if (res != HAL_OK) {
    	return MPU9250_RESULT_FAIL;
    }

    return MPU9250_RESULT_OK;
}

// Read the MPU's data registers
MPU9250_Result_t MPU9250_Read_Data_Registers(MPU9250_CONFIG_t *MPU9250_CONFIG, MPU9250_DATA_REGISTERS address, uint8_t count, uint8_t* dest)
{

    uint8_t reg = address | MPU9250_READ_MASK; // reg for register selection read/write
	HAL_StatusTypeDef res;

    HAL_GPIO_WritePin(MPU9250_CONFIG->GPIOx,MPU9250_CONFIG->GPIO_PIN , 0);
    res = HAL_SPI_Transmit(MPU9250_CONFIG->hspi,&reg,1,10);
    for (int i = 0; i < count; i++)
    {
    	res = HAL_SPI_Receive(MPU9250_CONFIG->hspi,(uint8_t*)(dest + i),1,10);
    }
    HAL_GPIO_WritePin(MPU9250_CONFIG->GPIOx,MPU9250_CONFIG->GPIO_PIN , 1);

    //Exception
    if (res != HAL_OK) {
    	return MPU9250_RESULT_FAIL;
    }
    return MPU9250_RESULT_OK;
}

// Read the MPU's configuration registers
MPU9250_Result_t MPU9250_Read_Config_Registers(MPU9250_CONFIG_t *MPU9250_CONFIG, MPU9250_CONFIGURATION_REGISTERS address, uint8_t count, uint8_t* dest)
{
    uint8_t reg = address | MPU9250_READ_MASK; // reg for register selection read/write
	HAL_StatusTypeDef res;

    HAL_GPIO_WritePin(MPU9250_CONFIG->GPIOx,MPU9250_CONFIG->GPIO_PIN , 0);
    res = HAL_SPI_Transmit(MPU9250_CONFIG->hspi,&reg,1,10);
    for (int i = 0; i < count; i++)
    {
    	res = HAL_SPI_Receive(MPU9250_CONFIG->hspi,(uint8_t*)(dest + i),1,10);
    }
    HAL_GPIO_WritePin(MPU9250_CONFIG->GPIOx,MPU9250_CONFIG->GPIO_PIN , 1);

    //Exception
    if (res != HAL_OK) {
    	return MPU9250_RESULT_FAIL;
    }
    return MPU9250_RESULT_OK;
}


MPU9250_Result_t MPU9250_Initialize(MPU9250_CONFIG_t *MPU9250_CONFIG){
    uint8_t buf;
    MPU9250_Result_t res;

    /* MPU9250_WHO_AM_I ---------------------------------------------------------*/
    res = MPU9250_Read_Config_Registers(MPU9250_CONFIG, MPU9250_WHO_AM_I, 1, &buf);
    //WHO_AM_I Check
    if (buf != 0x73) {
    	return MPU9250_RESULT_FAIL;
    }

    /* MPU9250_Reset ----------------------------------------------------------*/
    res = MPU9250_Write_Register(MPU9250_CONFIG, MPU9250_PWR_MGMT_1, 0x80);
    //Exception
    if (res != MPU9250_RESULT_OK) {
    	return MPU9250_RESULT_FAIL;
    }
    HAL_Delay(100);
    return MPU9250_RESULT_OK;
}

MPU9250_Result_t MPU9250_Config_accel_scale(MPU9250_CONFIG_t *MPU9250_CONFIG){
    uint8_t scale;
    MPU9250_Result_t res;

	/* MPU9250_Set_Accel_Scale ---------------------------------------------------------*/
	scale = MPU9250_CONFIG->ACCEL_SCALE;
	if(scale == ACCEL_SCALE_2G){		//2G
		MPU9250_CONFIG->ARES = 2.0;
	}
	else if(scale == ACCEL_SCALE_4G){	//4G
		MPU9250_CONFIG->ARES = 4.0;
	}
	else if(scale == ACCEL_SCALE_8G){	//8G
		MPU9250_CONFIG->ARES = 8.0;
	}
	else if(scale == ACCEL_SCALE_16G){	//16G
		MPU9250_CONFIG->ARES = 16.0;
	}

	res = MPU9250_Write_Register(MPU9250_CONFIG, MPU9250_ACCEL_CONFIG, scale);

	if (res != MPU9250_RESULT_OK) {
		return MPU9250_RESULT_FAIL;
	}

    return MPU9250_RESULT_OK;
}

MPU9250_Result_t MPU9250_Config_fifo_int(MPU9250_CONFIG_t *MPU9250_CONFIG){
	MPU9250_Result_t res;
	#define INT_ANYRD_2CLEAR (1 << 4)
	/* MPU9250_Configure_interrupt_pin_behavior---------------------------------------------*/
	// Interrupt pin cleared if any read operation is performed
	MPU9250_Write_Register(MPU9250_CONFIG, MPU9250_PWR_MGMT_1, 0x80);
	HAL_Delay(100);
	MPU9250_Write_Register(MPU9250_CONFIG, MPU9250_PWR_MGMT_2, 0x37);
	MPU9250_Write_Register(MPU9250_CONFIG, MPU9250_CONFIG_REG, 0x40);
	res = MPU9250_Write_Register(MPU9250_CONFIG, MPU9250_INT_PIN_CFG, 0x00);

	if (res != MPU9250_RESULT_OK) {
		return MPU9250_RESULT_FAIL;
	}
	// Enable FIFO overflow interrupt
	#define FIFO_OVERFLOW_EN (1 << 4)

	res = MPU9250_Write_Register(MPU9250_CONFIG, MPU9250_INT_ENABLE,FIFO_OVERFLOW_EN );

	if (res != MPU9250_RESULT_OK) {
		return MPU9250_RESULT_FAIL;
	}
	// Enable + reset the FIFO



	#define EN_FIFO			(1 << 6)
	#define RST_FIFO 		(1 << 2)
	res = MPU9250_Write_Register(MPU9250_CONFIG, MPU9250_USER_CTRL, RST_FIFO);
	if (res != MPU9250_RESULT_OK) {
		return MPU9250_RESULT_FAIL;
	}
	res = MPU9250_Write_Register(MPU9250_CONFIG, MPU9250_USER_CTRL, 0x40);
	if (res != MPU9250_RESULT_OK) {
		return MPU9250_RESULT_FAIL;
	}

	// Configure the data pushed to the FIFO
	#define EN_FIFO_TEMP_OUT	(1 << 7)
	#define EN_FIFO_ACCEL 		(1 << 3)
	res = MPU9250_Write_Register(MPU9250_CONFIG, MPU9250_FIFO_EN, EN_FIFO_ACCEL);
	if (res != MPU9250_RESULT_OK) {
		return MPU9250_RESULT_FAIL;
	}


    return MPU9250_RESULT_OK;
}
