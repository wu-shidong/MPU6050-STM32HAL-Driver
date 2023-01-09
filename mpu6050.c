/**
 * @file mpu6050.c
 * @author Epsilon Wu(epsilon5400@gmail.com)
 * @brief Universal mpu6050 drivers for STM32 HAL library
 * @version 0.1
 * @date 2022-12-20
 * @copyright Copyright (c) 2022
 * @attention The IIC channel must be "I2C1"
 * @example
 * int main()
 * {
 * MPU6050_Init();
 * float pitch,roll,yaw; 
 * short aacx,aacy,aacz;		
 * short gyrox,gyroy,gyroz;	
 * short temp;	
 * while(1)
 * {
 * 		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{
			temp=MPU_Get_Temperature();								
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	
		}
 * }
 * }
 */
#include "mpu6050.h"
#include "delay.h"
#include "i2c.h"
#include "inv_mpu.h"
/**
 * @brief Init of MPU6050,It only needs to be executed once.
 * @name MPU6050_Init
 * @retval none
 */
void MPU6050_Init (void)
{
	uint8_t count;
	while(MPU_Init());				
	while(mpu_dmp_init())
	{
		HAL_Delay(200);
		count++;
		if(count>20) break;
	}

}


uint8_t MPU_Init(void)
{ 
	uint8_t res;
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	
	delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	
	MPU_Set_Gyro_Fsr(3);				
	MPU_Set_Accel_Fsr(0);				
	MPU_Set_Rate(50);					
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);
		MPU_Set_Rate(50);						
 	}else 
		return 1;
	return 0;
}

uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);
}

uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);
}

uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);
}

uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	
 	return MPU_Set_LPF(rate/2);	
}


short MPU_Get_Temperature(void)
{
  uint8_t buf[2]; 
  short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
	raw=((uint16_t)buf[0]<<8)|buf[1];  
	temp=36.53+((double)raw)/340;  
	return temp*100;
}

uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
  uint8_t buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((uint16_t)buf[0]<<8)|buf[1];  
		*gy=((uint16_t)buf[2]<<8)|buf[3];  
		*gz=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;;
}

uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
  uint8_t buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];  
		*ay=((uint16_t)buf[2]<<8)|buf[3];  
		*az=((uint16_t)buf[4]<<8)|buf[5];
	} 	
  return res;;
}

uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
	uint8_t sta;
	sta=HAL_I2C_Mem_Write(&hi2c1,addr<<1,reg,I2C_MEMADD_SIZE_8BIT,buf,len,100);
	if(sta==HAL_OK)
		return 0;	
	else
		return 1;	
} 

uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
	uint8_t sta;
	sta=HAL_I2C_Mem_Read(&hi2c1,addr<<1,reg,I2C_MEMADD_SIZE_8BIT,buf,len,100);
	if(sta==HAL_OK)
		return 0;	
	else
		return 1;
}

uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data) 				 
{
	uint8_t sta;
	sta=HAL_I2C_Mem_Write(&hi2c1,MPU_ADDR<<1,reg,I2C_MEMADD_SIZE_8BIT,&data,1,100);
	if(sta!=HAL_OK)
		return 1;
	else
		return 0;
}


uint8_t MPU_Read_Byte(uint8_t reg)
{
	uint8_t dat,sta;

	sta=HAL_I2C_Mem_Read(&hi2c1,MPU_ADDR<<1,reg,I2C_MEMADD_SIZE_8BIT,&dat,1,100);
	if(sta!=HAL_OK)
		return 1;	
	else
		return dat;
	
}


