#include<Wire.h>
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

unsigned int C1;
unsigned int C2;
unsigned int C3;
unsigned int C4;
unsigned int C5;
unsigned int C6;

byte msb, lsb;
byte mmsb, mlsb;

float temperature_c, temperature_f;
double pressure_abs, pressure_relative, altitude_delta, pressure_baseline;
MS5803 sensor(ADDRESS_LOW);

double base_altitude = 1655.0;

void setup(){
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Calibrating Pressure Sensor");
  //Retrieve calibration constants for conversion math.
  sensor.reset();
  sensor.begin();
  
  pressure_baseline = sensor.getPressure(ADC_4096);
}

void loop(){
  Serial.print("Current Tempurature is: ");
  Serial.print(readTemp());
  Serial.println("C");
  Serial.print("Current Pressure is: ");
  pressure_abs = sensor.getPressure(ADC_4096);
  Serial.print(pressure_abs);
  Serial.println("mBar");
  Serial.print("Change in Altitude is: ");
  Serial.println(altitude(pressure_abs, pressure_baseline));
  delay(1000);
}

float readPress(){
  //Read D1
  Wire.beginTransmission(pressAddress);
  Wire.write(D1ADD);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(pressAddress);
  Wire.write(ADCREAD);
  Wire.endTransmission();
  Wire.requestFrom(pressAddress, 3);
  while(!Wire.available());
  delay(7);
  mlsb = Wire.read();
  msb = Wire.read();
  lsb = Wire.read();
  unsigned long D1 = mlsb<<16 | msb<<8 | lsb;
  
  //Read D2
  Wire.beginTransmission(pressAddress);
  Wire.write(D2ADD);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(pressAddress);
  Wire.write(ADCREAD);
  Wire.endTransmission();
  Wire.requestFrom(pressAddress, 3);
  while(!Wire.available());
  delay(7);
  mlsb = Wire.read();
  msb = Wire.read();
  lsb = Wire.read();
  unsigned long D2 = mlsb<<16 | msb<<8 | lsb;
  
  Serial.print("D1:");
  Serial.print(D1);
  Serial.print(" D2:");
  Serial.println(D2);
  //calculate dT
  long dT = D2 - (C5 * pow(2, 8));
  
  //calculate actual tempurature
  long TEMP = 2000 + dT * C6/pow(2, 23);
  
  //Calculate offset
  int64_t OFF = C2*pow(2, 16) + (C4*dT)/pow(2,7);
  
  //Calculate sensitivity
  int64_t SENS = C1*pow(2,15) + (C3*dT)/pow(2,8);
  
  //Calculate the Pressure (units of 0.1mBar)
  int64_t P = (D1*SENS/pow(2,21) - OFF)/pow(2,15);
  //Return a float in units of mBar
  return ((float) P)/10;
}

void calibratePress(){
  //Read C1
  Wire.beginTransmission(pressAddress);
  Wire.write(PROM1);
  Wire.endTransmission();
  Wire.requestFrom(pressAddress, 2);
  while(!Wire.available());
  msb = Wire.read();
  lsb = Wire.read();
  C1 = msb<<8 | lsb;
  
  //Read C2
  Wire.beginTransmission(pressAddress);
  Wire.write(PROM2);
  Wire.endTransmission();
  Wire.requestFrom(pressAddress, 2);
  while(!Wire.available());
  msb = Wire.read();
  lsb = Wire.read();
  C2 = msb<<8 | lsb;
  
  //Read C3
  Wire.beginTransmission(pressAddress);
  Wire.write(PROM3);
  Wire.endTransmission();
  Wire.requestFrom(pressAddress, 2);
  while(!Wire.available());
  msb = Wire.read();
  lsb = Wire.read();
  C3 = msb<<8 | lsb;
  
  //Read C4
  Wire.beginTransmission(pressAddress);
  Wire.write(PROM1);
  Wire.endTransmission();
  Wire.requestFrom(pressAddress, 2);
  while(!Wire.available());
  msb = Wire.read();
  lsb = Wire.read();
  C4 = msb<<8 | lsb;
  
  //Read C5
  Wire.beginTransmission(pressAddress);
  Wire.write(PROM5);
  Wire.endTransmission();
  Wire.requestFrom(pressAddress, 2);
  while(!Wire.available());
  msb = Wire.read();
  lsb = Wire.read();
  C5 = msb<<8 | lsb;
  
  //Read C6
  Wire.beginTransmission(pressAddress);
  Wire.write(PROM6);
  Wire.endTransmission();
  Wire.requestFrom(pressAddress, 2);
  while(!Wire.available());
  msb = Wire.read();
  lsb = Wire.read();
  C6 = msb<<8 | lsb;
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

/*
void writeI2C (int deviceAddress, byte regAddr, byte val) {
  Wire.beginTransmission(deviceAddress);           // Begin transmission 
  Wire.write(regAddr);                     // Open communications
  Wire.write(val);                         // write value
  Wire.endTransmission();                 // End transmission
}

int readI2C (int deviceAddress, byte regAddr) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddr);                     // Register address to read
  Wire.endTransmission();                 // Terminate request
  Wire.requestFrom(deviceAddress, 1);              // Read a byte
  while(!Wire.available()) {
    //Serial.println("I am trapped in a loop!!");
  }          // Wait for receipt
  return(Wire.read());                 // Get result
}
*/
