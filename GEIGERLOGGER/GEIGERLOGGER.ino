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
char fileName[8];
#include <SdFat.h>
SdFat sd;
SdFile DataFile;
#include <SoftwareSerial.h>

SoftwareSerial mySerial(9, 8); // RX, TX

void setup()  
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  pinMode(10, OUTPUT);
  digitalWrite(10,HIGH);
  while(!sd.begin(10, SPI_HALF_SPEED)){}
  Serial.println("sd initialized");
  int i = 0;
  sprintf(fileName, "f%i.csv", i);
  while(sd.exists(fileName)){
    i++;
    sprintf(fileName, "f%i.csv", i);
  }
  DataFile.open(fileName, O_RDWR | O_CREAT | O_AT_END);

  Serial.println("Goodnight moon!");

  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
}

void loop() // run over and over
{
  DataFile.open(fileName,O_RDWR | O_CREAT | O_AT_END);
  if (mySerial.available()){
    DataFile.println(millis());
    mySerial.read();
  }
  DataFile.close();    
}

