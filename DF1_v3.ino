//Libraries
#include <MPU6050.h> // MPU Library

//Constant
const int MPU_addr=0x68;  // I2C address of the MPU-6050
const int chipSelect = 25; // STM32 pin into olimexino 

//Variables

//Creating a class MPU
MPU6050 MPU6050(MPU_addr); // This will apply once MPU6050 is defined.

void setup() {

}

void loop() {
int16_t AcX=0,AcY=0,AcZ=0,Tmp=0,GyX=0,GyY=0,GyZ=0; // declare accellerometer and gyro variables TWO Bytes each
int idelay = 333; //in milliseconds. It is the time to stop after shown data by serial data.

  MPU6050.MPU6050_ReadData(&AcX, &AcY, &AcZ, &Tmp, &GyX, &GyY, &GyZ, MPU_addr);
  MPU6050.MPU6050_ShowDataSerial(AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, idelay);

}
