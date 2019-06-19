//Libraries
#include <MPU6050.h> // MPU Library
#include <SPI.h>
//Constant
const int MPU_addr=0x68;  // I2C address of the MPU-6050
const int chipSelect = 25; // STM32 pin into olimexino
const int MAXTOAVOIDOVERFLOW = 32500; // to avoid overflow to walk in array
const int INITARRAY = 0; // first position of any ARRAY in Arduino
const int MAXNUMBERDATAPROCESSING = 10; //maximum number of data to process
const int FREQSAMPLE = 25;  //to control frequency sample

//Variables
int iContArray=0; // to walk into array AcX_Array.
long AcX_ARRAY[10]= {0,0,0,0,0,0,0,0,0,0}; // for storing and processing data
 

//Creating a class MPU
MPU6050 myMPU(MPU_addr); // This will apply once MPU6050 is defined.

void setup() {
        
  pinMode(PA0, OUTPUT); // Activation PIN for triggering LED 
  pinMode(PA1, OUTPUT); // Activation PIN for triggering LED PA1 is DIMM
  digitalWrite(PA0, HIGH);   // turn the LED on (HIGH is the voltage level) PA0 is enable
  myMPU.TriggerSTOP(true); //Checking for LED
  
  SPI.setModule(2); 
  if (myMPU.begin())
    Serial.println("Error initializing MPU");

  pinMode(PA0, OUTPUT); // Activation PIN for triggering speaker 
  myMPU.TriggerSTOP(true); //Checking for LED
}

void loop() {
  int16_t AcX=0,AcY=0,AcZ=0,Tmp=0,GyX=0,GyY=0,GyZ=0; // declare accellerometer and gyro variables TWO Bytes each
  int idelay = 333; //in milliseconds. It is the time to stop after shown data by serial data.
  long tinitial=0; //to control frequency sample
  int LedOn = -1;
  
  tinitial=millis();//to control frequency sample
  myMPU.ReadData(&AcX, &AcY, &AcZ, &Tmp, &GyX, &GyY, &GyZ);
  myMPU.ShowDataSerial(AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, idelay, LedOn);
  AcX_ARRAY[iContArray%MAXNUMBERDATAPROCESSING]=AcX;
  if (myMPU.ProcessingSignalforSTOP(AcX_ARRAY, MAXNUMBERDATAPROCESSING)){
    myMPU.TriggerSTOP(true);
    LedOn = 0;
    myMPU.ShowDataSerial(AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, idelay, LedOn);
    AcX_ARRAY[0] = 0;AcX_ARRAY[1] = 0;AcX_ARRAY[2] = 0;AcX_ARRAY[3] = 0;AcX_ARRAY[4] = 0;AcX_ARRAY[5] = 0;AcX_ARRAY[6] = 0;AcX_ARRAY[7] = 0;AcX_ARRAY[8] = 0;AcX_ARRAY[9] = 0;//,0,0,0,0,0,0,0,0,0};
  }
  
  // to control overflow of int i which is walking into array.
  iContArray++;
  if (iContArray/MAXNUMBERDATAPROCESSING == 1)
    iContArray=INITARRAY;

  while ((millis()-tinitial) < FREQSAMPLE); //to control frequency sample
}
