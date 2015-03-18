#include<Wire.h>
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
#define D2ADD 0x00

unsigned int C1;
unsigned int C2;
unsigned int C3;
unsigned int C4;
unsigned int C5;
unsigned int C6;

byte msb, lsb;
byte mmsb, mlsb;

void setup(){
  Wire.begin();
  Serial.begin(9600);
  calibratePress();
  Serial.println("Calibration Data:");
  Serial.print(C1);
  Serial.print(", ");
  Serial.print(C2);
  Serial.print(", ");
  Serial.print(C3);
  Serial.print(", ");
  Serial.print(C4);
  Serial.print(", ");
  Serial.print(C5);
  Serial.print(", ");
  Serial.print(C6);
  Serial.println();
}

void loop(){
  Serial.print("Current Tempurature is: ");
  Serial.print(readTemp());
  Serial.println("C");
  Serial.println("Current Pressure is: ");
  Serial.print(readPress());
  Serial.println("mBar");
  delay(100);
}

float readPress(){
  //Read D1
  Wire.beginTransmission(pressAddress);
  Wire.write(D1ADD);
  Wire.endTransmission();
  Wire.requestFrom(pressAddress, 2);
  while(!Wire.available());
  mlsb = Wire.read();
  msb = Wire.read();
  lsb = Wire.read();
  unsigned long D1 = (mlsb<<8 | msb)<<8 | lsb;
  
  //Read D2
  Wire.beginTransmission(pressAddress);
  Wire.write(D2ADD);
  Wire.endTransmission();
  Wire.requestFrom(pressAddress, 2);
  while(!Wire.available());
  mlsb = Wire.read();
  msb = Wire.read();
  lsb = Wire.read();
  unsigned long D2 = (mlsb<<8 | msb)<<8 | lsb;
  
  //calculate dT
  long dT = D2 - C5*256;
  
  //calculate actual tempurature
  long TEMP = 2000 + dT * C6/8388608;
  
  //Calculate offset
  int64_t OFF = C2*65536 + (C4*dT)/128;
  
  //Calculate sensitivity
  int64_t SENS = C1*32768 + (C3*dT)/256;
  
  //Calculate the Pressure (units of 0.1mBar)
  long P = (D1*SENS/2147483648 - OFF)/32768;
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
  float tempFloat = (float)temp/16.0;
  return tempFloat;
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
