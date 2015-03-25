/*
  Software serial multple serial test
 
 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.
 
 The circuit: 
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)
 
 Note:
 Not all pins on the Mega and Mega 2560 support change interrupts, 
 so only the following can be used for RX: 
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69
 
 Not all pins on the Leonardo support change interrupts, 
 so only the following can be used for RX: 
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).
 
 created back in the mists of time
 modified 25 May 2012
 by Tom Igoe
 based on Mikal Hart's example
 
 This example code is in the public domain.
 
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

#define tempRegister B00000000
#define tempAddress B1001000
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


SdFat sd;
SdFile DataFile;
SdFile SensorFile;

SoftwareSerial mySerial(9, 8); // RX, TX

byte msb, lsb;

float temperature_c, temperature_f;
double pressure_abs, pressure_relative, altitude_delta, pressure_baseline;
MS5803 sensor(ADDRESS_LOW);

double base_altitude = 1655.0;

char fileName[8];
char sensorFileName[8];

void setup()  
{
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  pinMode(10, OUTPUT);
  digitalWrite(10,HIGH);
  while(!sd.begin(10, SPI_HALF_SPEED)){}
  Serial.println("sd initialized");
  int i = 0;
  sprintf(fileName, "g%i.txt", i);
  while(sd.exists(fileName)){
    i++;
    sprintf(fileName, "g%i.txt", i);
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
  Serial.println("Calibrating Pressure Sensor");
  sensor.reset();
  sensor.begin();  
  pressure_baseline = sensor.getPressure(ADC_4096);
}

void loop() // run over and over
{
  if (mySerial.available()){
    DataFile.open(fileName,O_RDWR | O_CREAT | O_AT_END);
    DataFile.println(millis());
    mySerial.read();
    DataFile.close();  
  } 
  
  SensorFile.open(sensorFileName,O_RDWR | O_CREAT | O_AT_END);
  SensorFile.print(millis());
  SensorFile.print(", ");
  SensorFile.print(readTemp());
  SensorFile.print(", ");
  pressure_abs = sensor.getPressure(ADC_4096);
  SensorFile.print(pressure_abs);
  SensorFile.print(", ");
  SensorFile.println(altitude(pressure_abs, pressure_baseline));
  SensorFile.close();
  delay(100);
}

float readTemp(){
  Wire.beginTransmission(tempAddress);
  Wire.write(tempRegister);
  Wire.endTransmission();
  Wire.requestFrom(tempAddress, 2);
  while(!Wire.available());
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
