#include "lsm6dsl.h"
float Accel_x;	     		//X轴加速度值暂存
float Accel_y;	    		//Y轴加速度值暂存
float Accel_z;	     		//Z轴加速度值暂存
float Gyro_x;				//X轴陀螺仪数据暂存
float Gyro_y;        		//Y轴陀螺仪数据暂存
float Gyro_z;		 		//Z轴陀螺仪数据暂存
float  Ax,Ay,Az,Gx,Gy,Gz;
int16_t Mag_x,Mag_y,Mag_z;
int16_t Temp_RAW = 0;
float Temp,TIMESTAMP0,TIMESTAMP1,TIMESTAMP2;
S_short_XYZ	ACC_Valu_First;
S_short_XYZ	GYRO_Valu_First;
S_short_XYZ	ACC_Offset,GYRO_Offset;
//初始化LSM6DSL 双击  
void lsm6dsl_Init(void)
{
	uint8_t check;
	
	// 检查设备ID   WHO_AM_I
	
	HAL_I2C_Mem_Read (&hi2c1 ,ACC_GYRO_ADDRESS,LSM6DSL_WHO_AM_I,1,&check ,1,1000);
	
	if(check == 0X6A)		//如果设备存在
	{		
//		Data = 0X9C;//陀螺仪速率104hz 2000dps 
		LSM6DSL_Write_Byte(LSM6DSL_CTRL2_G ,0X4C);				
//		Data = 0x2F;//禁止手腕倾斜算法，启用时间戳数，禁止计步器算法，使能使倾斜的计算
		LSM6DSL_Write_Byte (LSM6DSL_CTRL10_C,0x2F);			
//		Data = 0x70;// 加速度计ODR寄存器设置为104hz，±2g
		LSM6DSL_Write_Byte (LSM6DSL_CTRL1_XL,0x40);				
//		Data = 0x9E;// 允许中断和不活动功能，配置过滤和点击识别功能
		LSM6DSL_Write_Byte (LSM6DSL_TAP_CFG,0x9E);			
//		Data = 0x00;//自由落体，唤醒，时间戳和睡眠模式功能持续时间设置寄存器
		LSM6DSL_Write_Byte (LSM6DSL_WAKE_UP_DUR,0x10);				
//		Data = 0x06;//纵向/横向位置和点击功能阈值寄存器，点击识别阈值00010
		LSM6DSL_Write_Byte (LSM6DSL_TAP_THS_6D,0x06);		
//		Data = 0X7F; // 设置 Quiet 和 Shock 时间窗口
		LSM6DSL_Write_Byte (LSM6DSL_INT_DUR2,0X7F);	
//		Data = 0x84;//单双击，阈值唤醒000100
		LSM6DSL_Write_Byte (LSM6DSL_WAKE_UP_THS,0x84);			
//		Data = 0x01;//线性加速度传感器控制寄存器8，不选择加速度低通滤波LPF2
		LSM6DSL_Write_Byte (LSM6DSL_CTRL8_XL,0x01);		
//		Data = 0x08;//函数在INT1寄存器上路由
		LSM6DSL_Write_Byte (LSM6DSL_MD1_CFG,0x08);				
	}	
}
void LSM6DSL_Gyro(void)
{
	uint8_t Rec_Data[6];
	// 从LSM6DSL_XOUT_H寄存器读取6字节的数据
	LSM6DSL_Read_Len (ACC_GYRO_ADDRESS,LSM6DSL_OUTX_H_G ,6,Rec_Data);	
	Gyro_x = (int16_t )(Rec_Data [0] << 8 | Rec_Data [1]);
	Gyro_y = (int16_t )(Rec_Data [2] << 8 | Rec_Data [3]);
	Gyro_z = (int16_t )(Rec_Data [4] << 8 | Rec_Data [5]);
	
	Gx = Gyro_x/14.29;
	Gy = Gyro_y/14.29;
	Gz = Gyro_z/14.29;
}
//得到加速度原始值
 void LSM6DSL_Accel(void)
{
	uint8_t Rec_Data[6];	
	//从ACCEL_XOUT_H寄存器读取6字节的数据
	LSM6DSL_Read_Len (ACC_GYRO_ADDRESS ,LSM6DSL_OUTX_H_XL,6,Rec_Data);	
	Accel_x = (int16_t )(Rec_Data [0] <<8 | Rec_Data [1]);
	Accel_y = (int16_t )(Rec_Data [2] <<8 | Rec_Data [3]);
	Accel_z = (int16_t )(Rec_Data [4] <<8 | Rec_Data [5]);
	
	Ax = Accel_x/16393.44;
	Ay = Accel_y/16393.44;
	Az = Accel_z/16393.44;
}
//得到温度原始值
void LSM6DSL_Read_Temp(void)
{
	u8 Rec_Data[2];
	int16_t Temp_RAW = 0;
	LSM6DSL_Read_Len (ACC_GYRO_ADDRESS,OUT_TEMP_H,2 ,Rec_Data);	
	Temp_RAW = (int16_t )(Rec_Data [0]<<8)|Rec_Data [1];
	Temp = (((float)Temp_RAW/256.0) + 25.0f);
}
//得到陀螺仪原始值
void LSM6DSL_Read_Gyro(short *gyroData)
{
	uint8_t Rec_Data[6];
	// 从LSM6DSL_XOUT_H寄存器读取6字节的数据
	LSM6DSL_Read_Len (ACC_GYRO_ADDRESS,LSM6DSL_OUTX_H_G ,6,Rec_Data);	
	gyroData[0] = (int16_t )(Rec_Data [0] << 8 | Rec_Data [1]);
	gyroData[1] = (int16_t )(Rec_Data [2] << 8 | Rec_Data [3]);
	gyroData[2] = (int16_t )(Rec_Data [4] << 8 | Rec_Data [5]);
}
//得到加速度原始值
 void LSM6DSL_Read_Accel(short *accData)
{
	uint8_t Rec_Data[6];	
	//从ACCEL_XOUT_H寄存器读取6字节的数据
	LSM6DSL_Read_Len (ACC_GYRO_ADDRESS ,LSM6DSL_OUTX_H_XL,6,Rec_Data);	
	accData[0] = (int16_t )(Rec_Data [0] <<8 | Rec_Data [1]);
	accData[1] = (int16_t )(Rec_Data [2] <<8 | Rec_Data [3]);
	accData[2] = (int16_t )(Rec_Data [4] <<8 | Rec_Data [5]);
}
//磁力计读取
 void LSM6DSL_Read_Mag(void)
 {
	 	uint8_t Rec_Data[6];	
	//从OUT_MAG_RAW_X_H寄存器读取6字节的数据
	LSM6DSL_Read_Len (ACC_GYRO_ADDRESS ,LSM6DSL_OUT_MAG_RAW_X_H,6,Rec_Data);	
	Mag_x = (int16_t )(Rec_Data [0] <<8 | Rec_Data [1]);
	Mag_y = (int16_t )(Rec_Data [2] <<8 | Rec_Data [3]);
	Mag_z = (int16_t )(Rec_Data [4] <<8 | Rec_Data [5]);
 }
//读取时间戳
 void LSM6DSL_Read_timestamp(void)
 {
	 uint8_t TS0_Data[2],TS1_Data[2],TS2_Data[2];
	 LSM6DSL_Read_Len (ACC_GYRO_ADDRESS,LSM6DSL_TIMESTAMP0_REG,2,TS0_Data);
   TIMESTAMP0	= (int16_t )(TS0_Data [0]<<8)|TS0_Data [1];
	 
	 LSM6DSL_Read_Len (ACC_GYRO_ADDRESS,LSM6DSL_TIMESTAMP1_REG,2,TS1_Data);
   TIMESTAMP1	= (int16_t )(TS1_Data [0]<<8)|TS1_Data [1];
	 
	 LSM6DSL_Read_Len (ACC_GYRO_ADDRESS,LSM6DSL_TIMESTAMP2_REG,2,TS2_Data);
   TIMESTAMP2	= (int16_t )(TS2_Data [0]<<8)|TS2_Data [1];
 }
 
//计算水平方向转的圈数
 void CountTurns(float *newdata,float *olddata,short *turns)
{
	if (*newdata<-170.0f && *olddata>170.0f)
		(*turns)++;
	if (*newdata>170.0f && *olddata<-170.0f)
		(*turns)--;
}
//short型角度和加速度最新一次的值定义，用于存放解算后的值
S_short_XYZ	ACC_Valu_First;
S_short_XYZ	GYRO_Valu_First;
S_short_XYZ	ACC_Offset,GYRO_Offset;


//读取初始偏差 读100次求平均
void MPU6050_ReadVOffset()
{
	int32_t offsetAcc_x = 0,offsetAcc_y = 0,offsetAcc_z = 0,offsetGyro_x = 0,offsetGyro_y = 0,offsetGyro_z = 0;
	int i;
	short accbuffer[3],gyrobuffer[3];
	
	for(i = 0; i < 100; i ++)
	{
		LSM6DSL_Read_Accel(accbuffer);
		LSM6DSL_Read_Gyro(gyrobuffer);
		
		offsetAcc_x = (offsetAcc_x + accbuffer[0]);
		offsetAcc_y = (offsetAcc_y + accbuffer[1]);
		offsetAcc_z = (offsetAcc_z + (accbuffer[2] - 16384));
		
		offsetGyro_x = (offsetGyro_x + gyrobuffer[0]);
		offsetGyro_y = (offsetGyro_y + gyrobuffer[1]);
		offsetGyro_z = (offsetGyro_z + gyrobuffer[2]);
		HAL_Delay(1);
	}
	
	ACC_Offset.X = offsetAcc_x / 100;
	ACC_Offset.Y = offsetAcc_y / 100;
	ACC_Offset.Z = offsetAcc_z / 100;
	
	GYRO_Offset.X = offsetGyro_x / 100;
	GYRO_Offset.Y = offsetGyro_y / 100;
	GYRO_Offset.Z = offsetGyro_z / 100;
} 
void LSM6DSL_ReadValu(void)
{
	short accbuffer[3],gyrobuffer[3];
	LSM6DSL_Read_Accel(accbuffer);
	LSM6DSL_Read_Gyro(gyrobuffer);
	
	ACC_Valu_First.X = accbuffer[0] - ACC_Offset.X;
	ACC_Valu_First.Y = accbuffer[1] - ACC_Offset.Y;
	ACC_Valu_First.Z = accbuffer[2] - ACC_Offset.Z;
	
	GYRO_Valu_First.X = gyrobuffer[0]- GYRO_Offset.X;
	GYRO_Valu_First.Y = gyrobuffer[1]- GYRO_Offset.Y;
	GYRO_Valu_First.Z = gyrobuffer[2]- GYRO_Offset.Z;
}
//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 LSM6DSL_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 sta;
	sta=HAL_I2C_Mem_Write(&hi2c1,addr,reg,I2C_MEMADD_SIZE_8BIT,buf,len,100);
	if(sta==HAL_OK)
		return 0;	
	else
		return 1;	
} 
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 LSM6DSL_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
	u8 sta;
	sta=HAL_I2C_Mem_Read(&hi2c1,addr,reg,I2C_MEMADD_SIZE_8BIT,buf,len,100);
	if(sta==HAL_OK)
		return 0;	
	else
		return 1;
}
//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 LSM6DSL_Write_Byte(u8 reg,u8 data) 				 
{
	u8 sta;
	sta=HAL_I2C_Mem_Write(&hi2c1,ACC_GYRO_ADDRESS,reg,I2C_MEMADD_SIZE_8BIT,&data,1,100);
	if(sta!=HAL_OK)
		return 1;
	else
		return 0;
}

//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 LSM6DSL_Read_Byte(u8 reg)
{
	u8 dat,sta;

	sta=HAL_I2C_Mem_Read(&hi2c1,ACC_GYRO_ADDRESS,reg,I2C_MEMADD_SIZE_8BIT,&dat,1,100);
	if(sta!=HAL_OK)
		return 1;	
	else
		return dat;
	
}







