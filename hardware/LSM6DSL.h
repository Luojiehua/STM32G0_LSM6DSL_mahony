#ifndef __LSM6DSL_H
#define __LSM6DSL_H

#include "i2c.h"

#define u8 uint8_t 
#define u16 uint16_t 

/* I2C Addresses */
#define ACC_GYRO_ADDRESS     0xD6

/* sensor output data */
#define LSM6DSL_OUTX_L_G  			0X22
#define LSM6DSL_OUTX_H_G  			0X23
#define LSM6DSL_OUTY_L_G  			0X24
#define LSM6DSL_OUTY_H_G  			0X25
#define LSM6DSL_OUTZ_L_G  			0X26
#define LSM6DSL_OUTZ_H_G  			0X27

#define LSM6DSL_OUTX_L_XL  			0X28
#define LSM6DSL_OUTX_H_XL  			0X29
#define LSM6DSL_OUTY_L_XL  			0X2A
#define LSM6DSL_OUTY_H_XL  			0X2B
#define LSM6DSL_OUTZ_L_XL  			0X2C
#define LSM6DSL_OUTZ_H_XL  			0X2D
/* sensor control reg */
#define LSM6DSL_CTRL1_XL  		  0X10

#define LSM6DSL_CTRL2_G  			  0X11
#define LSM6DSL_CTRL3_C  			  0X12
#define LSM6DSL_CTRL4_C  			  0X13
#define LSM6DSL_CTRL5_C  			  0X14
#define LSM6DSL_CTRL6_G  			  0X15
#define LSM6DSL_CTRL7_G  			  0X16
#define LSM6DSL_CTRL8_XL  			0X17
#define LSM6DSL_CTRL9_XL  			0X18
#define LSM6DSL_CTRL10_C  			0X19
#define LSM6DSL_INT1_CTRL  			0X0D
#define LSM6DSL_INT2_CTRL  			0X0E
#define LSM6DSL_WHO_AM_I        0x0F    //get id

#define LSM6DSL_FIFO_CTRL1      0X06
#define LSM6DSL_FIFO_CTRL2      0X07
#define LSM6DSL_FIFO_CTRL3      0X08
#define LSM6DSL_FIFO_CTRL4      0X09
#define LSM6DSL_FIFO_CTRL5      0X0A

#define LSM6DSL_SENSORHUB1_REG  		0X2E
#define LSM6DSL_SENSORHUB2_REG  		0X2F
#define LSM6DSL_SENSORHUB3_REG  		0X30
#define LSM6DSL_SENSORHUB4_REG  		0X31
#define LSM6DSL_SENSORHUB5_REG  		0X32
#define LSM6DSL_SENSORHUB6_REG  		0X33
#define LSM6DSL_SENSORHUB7_REG  		0X34
#define LSM6DSL_SENSORHUB8_REG  		0X35
#define LSM6DSL_SENSORHUB9_REG  		0X36
#define LSM6DSL_SENSORHUB10_REG  		0X37
#define LSM6DSL_SENSORHUB11_REG  		0X38
#define LSM6DSL_SENSORHUB12_REG  		0X39
#define LSM6DSL_FIFO_STATUS1  				0X3A
#define LSM6DSL_FIFO_STATUS2  				0X3B
#define LSM6DSL_FIFO_STATUS3  				0X3C
#define LSM6DSL_FIFO_STATUS4  				0X3D
#define LSM6DSL_FIFO_DATA_OUT_L  		0X3E
#define LSM6DSL_FIFO_DATA_OUT_H  		0X3F
#define LSM6DSL_TIMESTAMP0_REG  			0X40
#define LSM6DSL_TIMESTAMP1_REG  			0X41
#define LSM6DSL_TIMESTAMP2_REG  			0X42
#define LSM6DSL_STEP_TIMESTAMP_L  		0X49
#define LSM6DSL_STEP_TIMESTAMP_H  		0X4A
#define LSM6DSL_STEP_COUNTER_L  			0X4B
#define LSM6DSL_STEP_COUNTER_H  			0X4C
#define LSM6DSL_SENSORHUB13_REG  		0X4D
#define LSM6DSL_SENSORHUB14_REG  		0X4E
#define LSM6DSL_SENSORHUB15_REG  		0X4F
#define LSM6DSL_SENSORHUB16_REG  		0X50
#define LSM6DSL_SENSORHUB17_REG  		0X51
#define LSM6DSL_SENSORHUB18_REG  		0X52
#define LSM6DSL_FUNC_SRC1  					0X53
#define LSM6DSL_FUNC_SRC2  					0X54
#define LSM6DSL_TAP_CFG  						0X58
#define LSM6DSL_TAP_THS_6D  			  0X59
#define LSM6DSL_INT_DUR2  					0X5A
#define LSM6DSL_WAKE_UP_THS  				0X5B
#define LSM6DSL_WAKE_UP_DUR  				0X5C
#define LSM6DSL_FREE_FALL  					0X5D
#define LSM6DSL_MD1_CFG  						0X5E
#define LSM6DSL_MD2_CFG  						0X5F
#define LSM6DSL_TAP_SRC             0X1C
/************** Access Device RAM  *******************/
#define LSM6DSL_MASTER_CMD_CODE  						0X60
#define LSM6DSL_SENS_SYNC_SPI_ERROR_CODE  		0X61

/************** Embedded functions register mapping  *******************/
#define LSM6DSL_OUT_MAG_RAW_X_L              0x66
#define LSM6DSL_OUT_MAG_RAW_X_H              0x67
#define LSM6DSL_OUT_MAG_RAW_Y_L              0x68
#define LSM6DSL_OUT_MAG_RAW_Y_H              0x69
#define LSM6DSL_OUT_MAG_RAW_Z_L              0x6A
#define LSM6DSL_OUT_MAG_RAW_Z_H              0x6B
#define LSM6DSL_X_OFS_USR                  	 0x73
#define LSM6DSL_Y_OFS_USR                    0x74
#define LSM6DSL_Z_OFS_USR                    0x75

#define LSM6DSL_STATUS_REG                   0x1E 
#define  OUT_TEMP_H                          0x21
/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0X10
* Bit Group Name: FS_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DSL_FS_XL_2g 		 = 0x00,
	LSM6DSL_FS_XL_16g 	 = 0x04,
	LSM6DSL_FS_XL_4g 		 = 0x08,
	LSM6DSL_FS_XL_8g 		 = 0x0C,
} LSM6DSL_FS_XL_t;
/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0X10
* Bit Group Name: ODR_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DSL_ODR_XL_POWER_DOWN 		 = 0x00,
	LSM6DSL_ODR_XL_13Hz 		       = 0x10,
	LSM6DSL_ODR_XL_26Hz 		       = 0x20,
	LSM6DSL_ODR_XL_52Hz 		       = 0x30,
	LSM6DSL_ODR_XL_104Hz 		 	= 0x40,
	LSM6DSL_ODR_XL_208Hz 		 	= 0x50,
	LSM6DSL_ODR_XL_416Hz 		 	= 0x60,
	LSM6DSL_ODR_XL_833Hz 		 	= 0x70,
	LSM6DSL_ODR_XL_1660Hz 			= 0x80,
	LSM6DSL_ODR_XL_3330Hz 		 	= 0x90,
	LSM6DSL_ODR_XL_6660Hz 		 	= 0xA0,
	LSM6DSL_ODR_XL_13330Hz 		 = 0xB0,
} LSM6DSL_ODR_XL_t;

/*select io device*/
typedef enum 
{
  USE_SPI = 0,
  USE_I2C = 1
}io_device;

typedef struct{
	short X;
	short Y;
	short Z;
}S_short_XYZ;

/******USE IIC MODEL****/
void Lsm_Init_iic(void);
void Lsm_Get_Rawacc(void);
void Lsm_Get_Rawgryo(void);
/******USE SPI MODEL****/
void Lsm_Init_spi(void) ;
void Lsm_Get_RawAcc(void);
void Lsm_Get_RawGryo(void);

#endif
void lsm6dsl_Init(void);
void LSM6DSL_Read(void);
void LSM6DSL_Read_Temp(void);
void LSM6DSL_Read_Step(void);


u8 LSM6DSL_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);
u8 LSM6DSL_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf);
u8 LSM6DSL_Write_Byte(u8 reg,u8 data);
u8 LSM6DSL_Read_Byte(u8 reg);


//u8 LSM6DSL_Get_Accelerometer(short *ax,short *ay,short *az);
//u8 LSM6DSL_Get_Gyroscope(short *gx,short *gy,short *gz);
short LSM6DSL_Get_Temperature(void);
void LSM6DSL_Read_timestamp(void);
void CountTurns(float *newdata,float *olddata,short *turns);
void LSM6DSL_Read_Mag(void);
void LSM6DSL_Read_Accel(short *accData);
void LSM6DSL_Read_Gyro(short *gyroData);
void LSM6DSL_ReadValu(void);
 void LSM6DSL_Accel(void);
 void LSM6DSL_Gyro(void);
