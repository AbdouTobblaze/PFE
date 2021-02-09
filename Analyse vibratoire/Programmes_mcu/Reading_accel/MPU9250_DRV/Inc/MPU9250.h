/*
 * MPU9250.h
 *
 *  Created on: 15 janv. 2021
 *      Author: abdou
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#include "stm32f1xx_hal.h"


// From section 7.5 SPI Interface
// SPI read and write operations are completed in 16 or more clock cycles (two or more bytes). The
// first byte contains the SPI A ddress, and the following byte(s) contain(s) the SPI data. The first
// bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation.
// The following 7 bits contain the Register Address. In cases of multiple-byte Read/Writes, data is
// two or more bytes...
#define MPU9250_READ_MASK		0x80


// constants
#define MPU9250_G				9.807f
#define	MPU9250_d2r				3.14159265359f/180.0f; // degree to radian conversion coefficient
#define MPU9250_TEMP_SCALE		333.87f
#define MPU_9250_TEMP_OFFSET	21.0f

// MPU9250 Configuration registers
typedef enum
{
	// Self Test, Gyro
	MPU9250_SELF_TEST_X_GYRO = 0x00,
	MPU9250_SELF_TEST_Y_GYRO,
	MPU9250_SELF_TEST_Z_GYRO,
	// Self Test, Accelerometer
	MPU9250_SELF_TEST_X_ACCEL = 0x0D,
	MPU9250_SELF_TEST_Y_ACCEL,
	MPU9250_SELF_TEST_Z_ACCEL,
	// Gyro Offset
	MPU9250_XG_OFFSET_H = 0x13,
	MPU9250_XG_OFFSET_L,
	MPU9250_YG_OFFSET_H,
	MPU9250_YG_OFFSET_L,
	MPU9250_ZG_OFFSET_H,
	MPU9250_ZG_OFFSET_L,
	// Sample Rate Divider
	MPU9250_SMPLRT_DIV,
	// Config
	MPU9250_CONFIG_REG,
	MPU9250_GYRO_CONFIG,
	MPU9250_ACCEL_CONFIG,
	MPU9250_ACCEL_CONFIG_2,
	// Low Power Accelerometer ODR Control
	MPU9250_LP_ACCEL_ODR,
	// Wake-on Motion Threshold
	MPU9250_WOM_THR,
	// FIFO Enable
	MPU9250_FIFO_EN = 0x23,
	// I2C (MPU9250 can act as an I2C master to control other sensors)
	MPU9250_I2C_MST_CTRL,
	MPU9250_I2C_SLV0_ADDR,
	MPU9250_I2C_SLV0_REG,
	MPU9250_I2C_SLV0_CTRL,
	MPU9250_I2C_SLV1_ADDR,
	MPU9250_I2C_SLV1_REG,
	MPU9250_I2C_SLV1_CTRL,
	MPU9250_I2C_SLV2_ADDR,
	MPU9250_I2C_SLV2_REG,
	MPU9250_I2C_SLV2_CTRL,
	MPU9250_I2C_SLV3_ADDR,
	MPU9250_I2C_SLV3_REG,
	MPU9250_I2C_SLV3_CTRL,
	MPU9250_I2C_SLV4_ADDR,
	MPU9250_I2C_SLV4_REG,
	MPU9250_I2C_SLV4_DO,
	MPU9250_I2C_SLV4_CTRL,
	MPU9250_I2C_SLV4_DI,
	MPU9250_I2C_MST_STATUS,
	// Interrupt control registers
	MPU9250_INT_PIN_CFG,
	MPU9250_INT_ENABLE,
	MPU9250_DMP_INT_STATUS, // Check DMP Interrupt, see 0x6d
	MPU9250_INT_STATUS,
	// The rest of the I2C master registers
	MPU9250_I2C_SLV0_DO = 0x63,
	MPU9250_I2C_SLV1_D0,
	MPU9250_I2C_SLV2_DO,
	MPU9250_I2C_SLV3_DO,
	MPU9250_I2C_MST_DELAY_CTRL,
	// Signal path
	MPU9250_SIGNAL_PATH_RESET,
	// Motion detect
	MPU9250_MOT_DETECT_CTRL,
	// User
	MPU9250_USER_CTRL, // Bit 7 enable DMP, bit 3 reset DMP. See 0x6d
	// Power management
	MPU9250_PWR_MGMT_1,
	MPU9250_PWR_MGMT_2,
	// ...Looked for notes on DMP features, but Invensense docs were lacking.
	// Found kriswiner's Arduino sketch for Basic AHRS, and found values/notes for
	// Digital Motion Processing registers.
	//
	// See https://github.com/kriswiner/MPU-9250/blob/master/MPU9250BasicAHRS.ino
	MPU9250_DMP_BANK,
	MPU9250_DMP_RW_PNT,
	MPU9250_DMP_REG,
	MPU9250_DMP_REG_1,
	MPU9250_DMP_REG_2,
	// FIFO Count
	MPU9250_FIFO_COUNTH,
	MPU9250_FIFO_COUNTL,
	MPU9250_FIFO_R_W,
	MPU9250_WHO_AM_I,
	// Accel offset
	MPU9250_XA_OFFSET_H,
	MPU9250_XA_OFFSET_L,
	MPU9250_YA_OFFSET_H,
	MPU9250_YA_OFFSET_L,
	MPU9250_ZA_OFFSET_H,
	MPU9250_ZA_OFFSET_L
} MPU9250_CONFIGURATION_REGISTERS;


// MPU9250 Data registers
typedef enum
{
	// Accel XOUT
	MPU9250_ACCEL_XOUT_H = 0x3B,
	MPU9250_ACCEL_XOUT_L,
	MPU9250_ACCEL_YOUT_H,
	MPU9250_ACCEL_YOUT_L,
	MPU9250_ACCEL_ZOUT_H,
	MPU9250_ACCEL_ZOUT_L,
	// TEMP out
	MPU9250_TEMP_OUT_H,
	MPU9250_TEMP_OUT_L,
	// Gyro
	MPU9250_GYRO_XOUT_H,
	MPU9250_GYRO_XOUT_L,
	MPU9250_GYRO_YOUT_H,
	MPU9250_GYRO_YOUT_L,
	MPU9250_GYRO_ZOUT_H,
	MPU9250_GYRO_ZOUT_L,
	// Ext. Sensor data : These registers store data read from external sensors
	// by the Slave 0, 1, 2, and 3 on the auxiliary I2C interface.
	// Data read by Slave 4 is stored in I2C_SLV4_DI (Register 53)
	// only the first register is declared (0x49) from the 24 existing regs, the rest of the registers can be accessed by the mean of an addition
	MPU9250_EXT_SENS_DATA,
} MPU9250_DATA_REGISTERS;

// Error monitoring
typedef enum {
	MPU9250_RESULT_OK = 0x00,
	MPU9250_RESULT_FAIL
} MPU9250_Result_t;

// Accelerometer scaling factors
typedef enum {
    ACCEL_SCALE_2G  = 0x00,
    ACCEL_SCALE_4G  = 0x08,
    ACCEL_SCALE_8G  = 0x10,
    ACCEL_SCALE_16G = 0x18
} MPU9250_ACCEL_SCALE_t;

typedef enum
{
	DLPF_BANDWIDTH_184HZ	= 0x01,
	DLPF_BANDWIDTH_92HZ		= 0x02,
	DLPF_BANDWIDTH_41HZ		= 0x03,
	DLPF_BANDWIDTH_20HZ		= 0x04,
	DLPF_BANDWIDTH_10HZ		= 0x05,
	DLPF_BANDWIDTH_5HZ		= 0x06
}DLPF_BANDWIDTH;

// MPU9250 instance configuration
typedef struct {
    SPI_HandleTypeDef *hspi;//SPI handle
    GPIO_TypeDef *GPIOx;//CS pin GPIOx
    uint16_t GPIO_PIN ;//CS pin number
    MPU9250_ACCEL_SCALE_t ACCEL_SCALE;
    float ARES;//Accel resolution
    float ACCEL_OFFSET[3];
} MPU9250_CONFIG_t;

typedef struct {
    float accel[3];
    uint16_t accel_raw[3];
    float gyro[3];
    uint16_t gyro_raw[3];
    float mag[3];
    uint16_t mag_raw[3];
    float temperature;
    uint16_t temperature_raw;
} MPU9250_DATA_t;

typedef enum{
	MPU9250_ZG = (1 << 0),
	MPU9250_YG = (1 << 1),
	MPU9250_XG = (1 << 2),
	MPU9250_ZA = (1 << 3),
	MPU9250_YA = (1 << 4),
	MPU9250_XA = (1 << 5),
	MPU9250_AXES_ALL = 0x3F
}MPU9250_EN_DIS_AXES;

// Helper functions
MPU9250_Result_t MPU9250_Write_Register(MPU9250_CONFIG_t *MPU9250_CONFIG, MPU9250_CONFIGURATION_REGISTERS address, uint8_t data);
MPU9250_Result_t MPU9250_Read_Data_Registers(MPU9250_CONFIG_t *MPU9250_CONFIG, MPU9250_DATA_REGISTERS address, uint8_t count, uint8_t* dest);
MPU9250_Result_t MPU9250_Read_Config_Registers(MPU9250_CONFIG_t *MPU9250_CONFIG, MPU9250_CONFIGURATION_REGISTERS address, uint8_t count, uint8_t* dest);


// MPU functions
MPU9250_Result_t MPU9250_Initialize(MPU9250_CONFIG_t *MPU9250_CONFIG);
MPU9250_Result_t MPU9250_Config_accel_scale(MPU9250_CONFIG_t *MPU9250_CONFIG);
MPU9250_Result_t MPU9250_Config_fifo_int(MPU9250_CONFIG_t *MPU9250_CONFIG);



#endif /* MPU9250_H_ */
