/*
  Rad Heights - Main Build
  Arapahoe Community College
  Created by Robert Belter
 
Initializes sensors and logs to SD card
 
 The circuit: 
 * Geiger counter on pin 9
 * SD card through SPI
 * Tempurature sensor on i2c
 * Pressure sensor on i2c
 * Heater on pin 6
 * RGB Status LED on pins 2,3,4
 
 Ignore below:
 Status Config:
 On turn on, the following events can happen in this order:
 Status RED - 3 seconds - Power on
   Status RED (blinking) - 5 seconds - SD card failiure
 Status BLUE - 1 second - SD Finish
   Status BLUE (blinking) - 5 seconds - Tempurature failiure
   Status YELLOW (blinking) - 5 seconds - Pressure failiure
 Status RED - forever - startup complete with errors
 Status GREEN - forever - successful startup.
 
 */
 
 /*
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4
 */
#include <SdFat.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MS5803_I2C.h>

//Tempurature sensor constants
#define tempRegister B00000000
#define tempAddress B1001000

//Pressure sensor constants
#define pressAddress 0x77
#define PROM1 B10100010
#define PROM2 B10100100
#define PROM3 B10100110
#define PROM4 B10101000
#define PROM5 B10101010
#define PROM6 B10101100
#define D1ADD 0x48
#define D2ADD 0x58
#define ADCREAD 0x00
#define PRESRESET 0x1E

//Heater Pin
#define HEATERPIN 6

//LED PIN Defines
#define REDPIN 2
#define GREENPIN 3
#define BLUEPIN 4

//Codes for display output
#define POWERON 0
#define SDFAILURE 1
#define DONESUCCESS 6

//Information for the sd card
SdFat sd;
SdFile DataFile;
SdFile SensorFile;

//Geiger counter connected on pin 9
SoftwareSerial mySerial(9, 8); // RX, TX

//Vars for reading from sensors
byte msb, lsb;

float temperature_c, temperature_f;
double pressure_abs, pressure_relative, altitude_delta, pressure_baseline;
MS5803 sensor(ADDRESS_LOW);

double base_altitude = 1655.0;

//Vars to store data file names
char fileName[8];
char sensorFileName[8];

void setup()  
{
  //Setup the pins for the led
  pinMode(REDPIN, OUTPUT);
  pinMode(GREENPIN, OUTPUT);
  pinMode(BLUEPIN, OUTPUT);
  
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  
  displayStatus(POWERON);
  
  //Set up the heater
  pinMode(HEATERPIN, OUTPUT);
  
  pinMode(10, OUTPUT);
  digitalWrite(10,HIGH);
  int sdStatus = SDFAILURE;
  //Try 10 times to init sd card
  for(int i = 0; i<3; i++)
    if(sd.begin(10, SPI_HALF_SPEED)){
      sdStatus = DONESUCCESS;
      break;
    }
  displayStatus(sdStatus);
  
  //Initialize files
  int i = 0;
  sprintf(fileName, "g%i.csv", i);
  while(sd.exists(fileName)){
    i++;
    sprintf(fileName, "g%i.csv", i);
  }
  DataFile.open(fileName, O_RDWR | O_CREAT | O_AT_END);
  
  i = 0;
  sprintf(sensorFileName, "f%i.csv", i);
  while(sd.exists(sensorFileName)){
    i++;
    sprintf(sensorFileName, "f%i.csv", i);
  }
  SensorFile.open(sensorFileName, O_RDWR | O_CREAT | O_AT_END);
  SensorFile.println("Millis, Tempurature (C), Pressure (mBar), Delta Altitude (m)");
  SensorFile.close();

  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  
  
  //Configue the pressure sensor
  Wire.begin();
  Serial.println("readingtemp");
  Serial.println(readTemp());
  sensor.reset();
  sensor.begin();  
  pressure_baseline = sensor.getPressure(ADC_4096);
  
}

void loop() // run over and over
{
  //If the geiger counter detected an issue, log the time
  while (mySerial.available()){
    DataFile.open(fileName,O_RDWR | O_CREAT | O_AT_END);
    DataFile.println(millis());
    mySerial.read();
    DataFile.close();  
  } 
  //Gather all sensor data and print it
  SensorFile.open(sensorFileName,O_RDWR | O_CREAT | O_AT_END);
  SensorFile.print(millis());
  SensorFile.print(", ");
  float temp = readTemp();
  SensorFile.print(temp);
  SensorFile.print(", ");
  pressure_abs = sensor.getPressure(ADC_4096);
  SensorFile.print(pressure_abs);
  SensorFile.print(", ");
  SensorFile.println(altitude(pressure_abs, pressure_baseline));
  SensorFile.close();
  
  //Print data to serial (can be disabled)
  printSerial(temp, pressure_abs);
  
  //If the temperature is low, turn on the heater
  if(temp < 10)
    digitalWrite(HEATERPIN, HIGH);
  
  //If the temperature is high enough turn of the heater
  if(temp > 15)
    digitalWrite(HEATERPIN, LOW);
    
  delay(100);
}

float readTemp(){
  Wire.beginTransmission(tempAddress);
  Serial.println("what");
  Wire.write(tempRegister);
  Serial.println("what");
  Wire.endTransmission();
  Serial.println("what");
  Wire.requestFrom(tempAddress, 2);
  Serial.println("what");
  //while(!Wire.available());
  msb = Wire.read();
  lsb = Wire.read();
  int temp = msb<<8 | lsb;  
  float tempFloat = ((float)temp)/16.0*0.0625;
  return tempFloat;
}

double altitude(double P, double P0)
// Given a pressure measurement P (mbar) and the pressure at a baseline P0 (mbar),
// return altitude (meters) above baseline.
{
	return(44330.0*(1-pow(P/P0,1/5.255)));
}

void displayStatus(int toDisp){
  switch(toDisp){
    case POWERON:
      digitalWrite(REDPIN, LOW);
      digitalWrite(GREENPIN, LOW);
      digitalWrite(BLUEPIN, HIGH);
      Serial.println("-------------------------POWER ON--------------------------");
      delay(1000);
      break;
    case SDFAILURE:
      digitalWrite(REDPIN, HIGH);
      digitalWrite(GREENPIN, LOW);
      digitalWrite(BLUEPIN, LOW);
      Serial.println("!!!SD CARD FAILURE!!!");
      break;
    case DONESUCCESS:
      digitalWrite(REDPIN, LOW);
      digitalWrite(GREENPIN, HIGH);
      digitalWrite(BLUEPIN, LOW);
      Serial.println("Initialized successfully");
  }  
}

void printSerial(float temp, float pressure){
  Serial.print("The tempurature is:");
  Serial.print(temp);
  Serial.print("C, The pressure is:");
  Serial.print(pressure);
  Serial.print("mbar, Change in alititude:");
  Serial.print(altitude(pressure_abs, pressure_baseline));
  Serial.println("m");
  Serial.println();
}
  
