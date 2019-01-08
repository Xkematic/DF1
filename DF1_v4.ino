//Libraries
#include <MPU6050.h> // MPU Library
//#include<Wire.h> //allows communication between MPU6050 and STm32 by ISP  
//Define
#define NUMBEROFTIMETOLEDONAc 10 // Number of time to send signal output
//Constant
const int MPU_addr=0x68;  // I2C address of the MPU-6050
const int chipSelect = 25; // STM32 pin into olimexino 
const int OutPinSignal =  0;      // the number of the LED pin to send signal to speaker/LED_Board

/************************SD CARD****************************/
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
/************************SD CARD****************************/

//Variables

//Creating a class MPU
MPU6050 myMPU(MPU_addr); // This will apply once MPU6050 is defined.

void setup() {
  if (myMPU.begin())
    Serial.println("Error initializing MPU");

  #ifdef DEBUG_TO_SD
//    InitCardSD();
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
//  Serial.println("card initialized.");
    sprintf(snombrefichero, "DF%d.txt", iCuentaFichero);
//  Serial.println(snombrefichero);
    dataFile = SD.open(snombrefichero, FILE_WRITE);
    dataFile.println("File Opened");
    dataFile.close();
    pinMode(0, OUTPUT); // Activation PIN for triggering speaker 
  #endif

  // initialize the OutPinSignal as an output:
    pinMode(OutPinSignal, OUTPUT);
    TriggerSTOP();
}

void loop() {
  int16_t AcX=0,AcY=0,AcZ=0,Tmp=0,GyX=0,GyY=0,GyZ=0; // declare accellerometer and gyro variables TWO Bytes each
  int idelay = 333; //in milliseconds. It is the time to stop after shown data by serial data.

  #ifdef DEBUG_TO_SD
    char cadena[250];
    int LedOn = -1;
  #endif
  
  //while (!Serial.available()) continue;// Wait for the user to press a key
  myMPU.ReadData(&AcX, &AcY, &AcZ, &Tmp, &GyX, &GyY, &GyZ);
  #ifndef DEBUG_TO_SD
    myMPU.ShowDataSerial(AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, idelay);
  #endif
  #ifdef DEBUG_TO_SD
    sprintf(cadena,"P = %d | AcX = %d | AcY = %d | AcZ = %d | TMP = %.2f | GyX = %d | GyY = %d | GyZ = %d | LED? = %d", millis(), AcX, AcY, AcZ, Tmp/340.00+36.53, GyX, GyY, GyZ, LedOn);    
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

//send signal to ON STOP to speaker for debugging
void TriggerSTOP()
{
    for (int i=0; i<NUMBEROFTIMETOLEDONAc; i++)
    {
      digitalWrite(0, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1500);
      digitalWrite(0, LOW);   // turn the LED on (HIGH is the voltage level)
      delay(50);
    }
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
