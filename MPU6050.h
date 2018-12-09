/*
  MPU6050.h - Library for controllin MPU6050.
  Created by info@xkematic.com (Juan Guerrero and Manolo Madueño), September , 2018.
  Released into the public domain under Creative Common License.
*/
#ifndef MPU6050_h
#define MPU6050_h
#include "Arduino.h"

class MPU6050
{
  public:
	MPU6050(int MPU_addr); // Constructor
	void MPU6050_ReadData(int16_t *AcX,int16_t *AcY,int16_t *AcZ,int16_t *Tmp,int16_t *GyX,int16_t *GyY,int16_t *GyZ,int MPU_addr);
	void MPU6050_ShowDataSerial(int16_t AcX,int16_t AcY,int16_t AcZ,int16_t Tmp,int16_t GyX,int16_t GyY,int16_t GyZ,int idelay);
  private:
    
};
  //MPU6050_1.ReadingDataMPU6050(&AcX, &AcY, &AcZ, &Tmp, &GyX, &GyY, &GyZ);
  //MPU6050_1.ShowDataUSBlog(&AcX, &AcY, &AcZ, &Tmp, &GyX, &GyY, &GyZ);

#endif