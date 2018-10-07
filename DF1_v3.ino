//Libraries
#include <MPU6050.h> // MPU Library
#include "Arduino.h"
#include<Wire.h>

//MPU6050 MPU6050_1 = MPU6050(); // This will apply once MPU6050 is defined.

//Constant
#define MAXNUMBERITERACALIBRA 1000 // Iteration to store parameter to aboid spurois peaks
#define MAXUMBRAL 4000 //limit to trigger LED ON
#define NUMBEROFTIMETOLEDON 10
//#define DEBUG_TO_USB    // Controls if we are gonna show data by USB port
#define DEBUG_TO_SD    // Controls if we are gonna store data in SD
#include <SPI.h>
#include <SD.h>
char snombrefichero[25];     // Creating new file for storing in SD
int iCuentaFichero = 0;// Creating new file for storing in SD
int iResult;           
int previousMillis = 0;     
File dataFile; // CREO QUE PODEMOS BORRARLA HABLAR CON MANUEL. will store the last time the LED was updated, this time is to control                                                                                                                                                          
                            // constants won't change. They're used here to set pin numbers:
const int buttonPin = 38;   // the number of the pushbutton pin
const int ledPin =  3;      // the number of the LED pin
int buttonState = 0;        // variable for reading the pushbutton status


const int MPU_addr=0x68;  // I2C address of the MPU-6050
const int chipSelect = 25; // STM32 pin into olimexino 

//Global Var
int16_t AcX_Init=0,AcY_Init=0,AcZ_Init=0,Tmp_Init=0,GyX_Init=0,GyY_Init=0,GyZ_Init=0; // declare accellerometer and gyro variables TWO Bytes each
int16_t AcX=0,AcY=0,AcZ=0,Tmp=0,GyX=0,GyY=0,GyZ=0; // declare accellerometer and gyro variables TWO Bytes each

void setup() {
  
  MPU6050_Init(); 
  //MPU6050_InitCalibration(&AcX_Init, &AcY_Init, &AcZ_Init, &Tmp_Init, &GyX_Init, &GyY_Init, &GyZ_Init);
  MPU6050_AcquiringData(&AcX_Init, &AcY_Init, &AcZ_Init, &Tmp_Init, &GyX_Init, &GyY_Init, &GyZ_Init);
  #ifdef DEBUG_TO_USB  
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }
    Serial.begin(9600); // initialize serial port to 9600 bps so you can see your debug messages in Arduino IDE via debug channel 
    Serial.println("Initial Calibration value");
    MPU6050_ShowtoUSB(AcX_Init, AcY_Init, AcZ_Init, Tmp_Init, GyX_Init, GyY_Init, GyZ_Init);
    delay(5000);
  #endif
  #ifdef DEBUG_TO_SD
    // initialize the LED pin as an output:
    pinMode(ledPin, OUTPUT);
    // initialize the pushbutton pin as an input:
    pinMode(buttonPin, INPUT);
    //to control press button and indicate with LED
    // Initialize the built-in LED pin as an output:
    SPI.setModule(2); 
//    Serial.print("Initializing SD card...");
    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
//      Serial.println("Card failed, or not present");
      // don't do anything more:
      return;
    }
//    Serial.println("card initialized.");
    sprintf(snombrefichero, "DF%d.txt", iCuentaFichero);
//    Serial.println(snombrefichero);
    dataFile = SD.open(snombrefichero, FILE_WRITE);
    dataFile.println("File Opened");
    dataFile.close();
  #endif
  pinMode(0, OUTPUT); // Activation PIN for triggering speaker
  TriggerSTOP();
}

void loop() {
  // put your main code here, to run repeatedly:
  int16_t AcX=0,AcY=0,AcZ=0,Tmp=0,GyX=0,GyY=0,GyZ=0; // declare accellerometer and gyro variables TWO Bytes each
  char cadena[250];
  bool LedOn = false;

  //MPU6050_ReadData(&AcX, &AcY, &AcZ, &Tmp, &GyX, &GyY, &GyZ);
  MPU6050_AcquiringData(&AcX, &AcY, &AcZ, &Tmp, &GyX, &GyY, &GyZ);
  LedOn = MPU6050_ProcessingData(AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, AcX_Init, AcY_Init, AcZ_Init, Tmp_Init, GyX_Init, GyY_Init, GyZ_Init);
  if (LedOn)
    TriggerSTOP(); 
    
  #ifdef DEBUG_TO_USB /* Y VISUALIZARLO POR EL PUERTO SERIE**/
    MPU6050_ShowtoUSB(AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ);
  #endif
    
  #ifdef DEBUG_TO_SD
    sprintf(cadena,"P = %d | AcX = %d | AcY = %d | AcZ = %d | TMP = %.2f | GyX = %d | GyY = %d | GyZ = %d", millis(), AcX, AcY, AcZ, Tmp/340.00+36.53, GyX, GyY, GyZ);    
  /*RUTINA PARA ALMACENAR LOS DATOS EN LA SD CARD Y VISUALIZARLO POR EL PUERTO SERIE*//*SOLO PARA RECOGER DATOS NO AL PRODUCTO FINAL*/  
    dataFile = SD.open(snombrefichero, FILE_WRITE);
    if (dataFile){ //Si no existe el fichero, lo crea. Trunca la longitud a
      dataFile.println(cadena);
      dataFile.close();
   }
    /*END RUTINA PARA ALMACENAR LOS DATOS EN LA SD CARD Y VISUALIZARLO POR EL PUERTO SERIE*//*SOLO PARA RECOGER DATOS NO AL PRODUCTO FINAL*/    
  #endif

    /*RUTINA PARA RESETEAR EL NOMBRE DEL FICHERO Y RESETEAR EL LED EL FICHERO DONDE SE VAN A GUARDAR LOS DATOS*//*SOLO PARA RECOGER DATOS NO AL PRODUCTO FINAL*/
    #ifdef DEBUG_TO_SD
    previousMillis = millis();
    if (isButtonPressed()) {
      // If so, turn the LED from on to off, or from off to on:
      Serial.println("Creando nuevo fichero");
      iCuentaFichero++;
      sprintf(snombrefichero, "DF%d.txt", iCuentaFichero);  
      Serial.println(snombrefichero);
      dataFile = SD.open(snombrefichero, FILE_WRITE);
      dataFile.println("File Opened");
      dataFile.close();
      //delay(5);
    }
    #endif

}


bool MPU6050_ProcessingData(int16_t AcX,int16_t AcY,int16_t AcZ,int16_t Tmp,int16_t GyX,int16_t GyY,int16_t GyZ,int16_t AcX_Init, 
int16_t AcY_Init,int16_t AcZ_Init,int16_t Tmp_Init,int16_t GyX_Init,int16_t GyY_Init,int16_t GyZ_Init)
{
  if (abs(AcX - AcX_Init) >= MAXUMBRAL)
    return true;
  else if (abs(AcY - AcY_Init) >= MAXUMBRAL)
    return true;
  else if (abs(AcZ - AcZ_Init) >= MAXUMBRAL)
    return true;
 
  return false;
}

//Reading accellerometer,gyroscope and temperature for a while or a number of time to avoid spureos interrution
void MPU6050_AcquiringData(int16_t *AcX_Init, int16_t *AcY_Init, int16_t *AcZ_Init, int16_t *Tmp_Init, int16_t *GyX_Init, int16_t *GyY_Init, int16_t *GyZ_Init)
{
  double AcXTemp=0, AcYTemp=0, AcZTemp=0, TmpTemp=0, GyXTemp=0, GyYTemp=0, GyZTemp=0;
  int16_t AcXTempUint=0, AcYTempUint=0, AcZTempUint=0, TmpTempUint=0, GyXTempUint=0, GyYTempUint=0, GyZTempUint=0;
  char cadena[250];
  
  #ifdef DEBUG_TO_USB /* Y VISUALIZARLO POR EL PUERTO SERIE**/
    Serial.println("Doing calibration");
  #endif
  for (int i=0;i<MAXNUMBERITERACALIBRA;i++)
  {
    MPU6050_ReadData(&AcXTempUint, &AcYTempUint, &AcZTempUint, &TmpTempUint, &GyXTempUint, &GyYTempUint, &GyZTempUint);
    AcXTemp = AcXTemp + AcXTempUint;
    AcYTemp = AcYTemp + AcYTempUint;
    AcZTemp = AcZTemp + AcZTempUint;
    TmpTemp = TmpTemp + TmpTempUint;
    GyXTemp = GyXTemp + GyXTempUint;
    GyYTemp = GyYTemp + GyYTempUint;
    GyZTemp = GyZTemp + GyZTempUint;
    #ifdef DEBUG_TO_USB /* Y VISUALIZARLO POR EL PUERTO SERIE**/
      sprintf(cadena,"P = %f | AcX = %f | AcY = %f | AcZ = %f | TMP = %.2f | GyX = %f | GyY = %f | GyZ = %f", millis(), AcXTemp, AcYTemp, AcZTemp, TmpTemp/340.00+36.53, GyXTemp, GyYTemp, GyZTemp);    
      Serial.println("Data while for loop for storing whole data");
      Serial.println(cadena);
      delay(10);
     #endif
  }
  
    AcXTemp = AcXTemp/MAXNUMBERITERACALIBRA;
    *AcX_Init = AcXTemp;
    AcYTemp = AcYTemp/MAXNUMBERITERACALIBRA;
    *AcY_Init = AcYTemp;
    AcZTemp = AcZTemp/MAXNUMBERITERACALIBRA;
    *AcZ_Init = AcZTemp;
    TmpTemp = TmpTemp/MAXNUMBERITERACALIBRA;
    *Tmp_Init = TmpTemp;
    GyXTemp = GyXTemp/MAXNUMBERITERACALIBRA;
    *GyX_Init = GyXTemp;
    GyYTemp = GyYTemp/MAXNUMBERITERACALIBRA;
    *GyY_Init = GyYTemp;
    GyZTemp = GyZTemp/MAXNUMBERITERACALIBRA;
    *GyZ_Init = GyZTemp;
    
  #ifdef DEBUG_TO_USB /* Y VISUALIZARLO POR EL PUERTO SERIE**/
    MPU6050_ShowtoUSB(*AcX_Init, *AcY_Init, *AcZ_Init, *Tmp_Init, *GyX_Init, *GyY_Init, *GyZ_Init);
    Serial.println("Data after divide by MAXNUMBERITERACALIBRA");
    delay(1000);
  #endif
}


//Reading accellerometer,gyroscope and temperature
void MPU6050_ReadData(int16_t *AcX,int16_t *AcY,int16_t *AcZ,int16_t *Tmp,int16_t *GyX,int16_t *GyY,int16_t *GyZ)
{
  Wire.beginTransmission(MPU_addr); // get ready to talk to MPU again
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false); // done talking to MPU for the time being
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

//Constructor of MPU6050. Setup register,etc...
void MPU6050_Init()
{
  Wire.begin(); // initiate i2c system
  Wire.beginTransmission(MPU_addr); // be sure we talk to our MPU vs some other device
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true); // done talking over to MPU device, for the moment
}

// Procedure to answer this question: do we need to trigger STOP?
bool MPU6050_ProcessingData()
{
  return false;
}

//send signal to ON STOP
void TriggerSTOP()
{
  for (int i=0; i<NUMBEROFTIMETOLEDON; i++)
  {
    digitalWrite(0, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);
    digitalWrite(0, LOW);   // turn the LED on (HIGH is the voltage level)
    delay(100);
  }
}


// to show acelerometer, giroscop and tm parameters by putty with USB port. PLEASE THIS FUNTCIO HAS A DELAY OF 333ms
void MPU6050_ShowtoUSB(int16_t AcX,int16_t AcY,int16_t AcZ,int16_t Tmp,int16_t GyX,int16_t GyY,int16_t GyZ)
{
  Serial.print("AcX = "); Serial.print(AcX); // share accellerometer values over debug channel 
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX); // share gyroscope values over debug channel
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
  //delay(300); // delay a bit to not overwhelm you the user/programmer as you view the results
}

// to test if button in pressed to update file where data is stored
int isButtonPressed() 
{
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH){
    // turn LED on:
    digitalWrite(ledPin, HIGH);
    return true;
  } else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
    return false;
  }
}


