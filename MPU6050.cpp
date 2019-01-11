/*
  MPU6050.h - Library for controllin MPU6050.
  Created by info@xkematic.com (Juan Guerrero and Manolo Madueño), September , 2018.
  Released into the public domain under Creative Common License.
*/

 // #include "Arduino.h"
  #include "MPU6050.h"
  #include<Wire.h> //allows communication between MPU6050 and STm32 by ISP  

// const int MPU_addr=0x68;  // I2C address of the MPU-6050
const int LIMITTOSTOP = -6500; // limit to trigger STOP signal, under this value STOP will blink.
const int NUMBEROFTIMETOLEDONAc = 15; //number of time to blink
const int OutPinSignal =  0;      // the number of the LED pin to send signal to speaker/LED_Board

/************************************************************************************/
// MPU6050_Constructor
/************************************************************************************/
MPU6050::MPU6050(int addr){
	MPU_addr = addr;
}

/************************************************************************************/
// MPU6050_begin
// Input parameter:
//    MPU_addr -> address to make the pipe to communicate between MPU6050 and STM32
// Output parameter:
//    return 0 if no error
bool MPU6050::begin(void){
  Wire.begin(); // initiate i2c system
  Wire.beginTransmission(MPU_addr); // be sure we talk to our MPU vs some other device
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  return Wire.endTransmission(true); // done talking over to MPU device, for the moment  
}


/************************************************************************************/
// MPU6050_ReadData
// Input parameter: 
//    MPU_addr -> address to make the pipe to communicate between MPU6050 and STM32
// Output parameter:
//    AcX -> Read and Store acceleration of X
//    AcY -> Read and Store acceleration of Y
//    AcZ -> Read and Store acceleration of Z
//    Tmp -> Read and Store Temperature
//    GyX -> Read and Store Gyroscope values of X
//    GyY -> Read and Store Gyroscope values of Y
//    GyZ -> Read and Store Gyroscope values of Z
// Return:
//    0 if no error, otherwise higher than 0
/************************************************************************************/
void MPU6050::ReadData(int16_t *AcX,int16_t *AcY,int16_t *AcZ,int16_t *Tmp,int16_t *GyX,int16_t *GyY,int16_t *GyZ){
  Wire.beginTransmission(MPU_addr); // get ready to talk to MPU again
  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.write(0x3B);
  Wire.endTransmission(true); // done talking to MPU for the time being
  Wire.requestFrom(MPU_addr,14);  // request a total of 14 registers
  // all the fancy <<8| stuff is to bit shift the first 8 bits to
  // the left & combine it with the next 8 bits to form 16 bits
  *AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  *AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  *AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  *Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  *GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  *GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  *GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  // the above lines have gathered Accellerometer values for X, Y, Z
  //  as well as Gyroscope values for X, Y, Z    
}

/************************************************************************************/
// TriggerSTOP
// Input parameter: 
//    iTrigger -> if TRUE then STOP shall blink
// Output parameter:
//		NONE
// Return:
//    0 if no error, otherwise higher than 0
/************************************************************************************/
void MPU6050::TriggerSTOP(bool iTrigger){
	
	if (iTrigger){
		pinMode(OutPinSignal, OUTPUT);
		for (int i=0; i<NUMBEROFTIMETOLEDONAc; i++)
		{
			digitalWrite(OutPinSignal, HIGH);   // turn the LED on (HIGH is the voltage level)
			delay(150);
			digitalWrite(OutPinSignal, LOW);   // turn the LED on (HIGH is the voltage level)
			delay(50);
		}
	}
}

/************************************************************************************/
// ProcessingSignalforSTOP
// Input parameter: 
//    AcX_Array -> Array where micro is storing aceleretation of MPU
//	  iMAXNUMBERDATAPROCESSING -> maximum number of data to be processed
// Output parameter:
//    NONE
// Return:
//    false no trigger STOP no value under limit, otherwise true and trigger signal
/************************************************************************************/
bool MPU6050::ProcessingSignalforSTOP(int16_t AcX_ARRAY[], int iMAXNUMBERDATAPROCESSING){
	long int iResult = 0;
	for (int i=0;i<iMAXNUMBERDATAPROCESSING;i++){
		iResult = iResult + AcX_ARRAY[i];
		//Serial.print("iResult = "); Serial.println(iResult); // share accellerometer values over debug channel 
		//Serial.print("AcX_ARRAY[i] = "); Serial.println(AcX_ARRAY[i]); // share accellerometer values over debug channel 
	}
	//delay(500);
	if ((iResult/iMAXNUMBERDATAPROCESSING) < LIMITTOSTOP)
		return true;
	else
		return false;
}

/************************************************************************************/
// MPU6050_ShowDataSerial
// Input parameter:
//    idelay -> in milliseconds. It is the time to stop after shown data by serial data.
//    AcX -> to show acceleration of X
//    AcY -> to show acceleration of Y
//    AcZ -> to show acceleration of Z
//    Tmp -> to show Temperature
//    GyX -> to show Gyroscope values of X
//    GyY -> to show Gyroscope values of Y
//    GyZ -> to show Gyroscope values of Z
// Output parameter:
//    NONE
/************************************************************************************/
void MPU6050::ShowDataSerial(int16_t AcX,int16_t AcY,int16_t AcZ,int16_t Tmp,int16_t GyX,int16_t GyY,int16_t GyZ,int idelay, int iLED){
  Serial.print("AcX = "); Serial.print(AcX); // share accellerometer values over debug channel 
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX); // share gyroscope values over debug channel
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.print(GyZ);
  Serial.print(" | LED = "); Serial.println(iLED);
  //delay(idelay); // delay a bit to not overwhelm you the user/programmer as you view the results
}