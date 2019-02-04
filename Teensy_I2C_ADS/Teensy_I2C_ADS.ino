/*
   Author: Justin Francis
   CDate: 1/11/19
   EDate: 1/15/19
   Version: 1.0

   @ read data from hall sensors via ADS1115 at 500Hz and 16 bit resolution
   Test: Commit 3
*/

#include <Wire.h>

int ADS_Address = 0x48; //Analog to digital converter addy
byte ADS_ConfigRegister = 0x01; //internal addy for config Reg
byte ADS_ConversionRegister = 0x00; //internatl addy for conversion reg
byte MSB_A0 = 0xC4; // AIN0 to GND: 1.100.010.0.LSB
byte MSB_A1 = 0xD4; // AIN1 to GND: 1.101.010.0.LSB
byte LSB = 0x83; // AIN0 and AIN1 to GND: 100.0.0.0.11

void setup() {
  Wire.begin(); //open up comms
  Serial.begin(9600); //start serial comms at 9600 baud
//  Serial.println("Attempting to communicate with ADS"); //let user know program has started
}

int pinRead = 0; //determines which pin is read during the cycle
int pinOut = 0; //part of pinRead functionality
char pinString[2]; //debug message

signed char dataBytes[2]; //holds receivd data bytes as signed bytes
float data; //volt signed integer representation of received data bytes

unsigned long startTime = millis();
unsigned long elapsedTime = 0;
float desiredFreq = 500; //Hz
//unsigned long freqStartTime = millis();
//unsigned long freqStopTime;
//float freq = 500;
//int count = 0;

void loop() {

  elapsedTime = millis();
  if (elapsedTime - startTime >= (1 / desiredFreq * 1000))
  {
    //code used to determine loop freq, should be relatively stable so no need for continuous update unless its important *ASK LUKAS*
//    if (millis() % 1000 == 0)
//    {
//      freqStopTime = millis();
//      Serial.print("Loop Frequency: ");
//      freq = count / ((freqStopTime - freqStartTime) / 1000.0);
//      Serial.println(freq);
//      count = 0;
//      freqStartTime = millis();
//    }
//    startTime = millis();
//    count++;

    //register setup
    if (pinRead == 0) {
      Wire.beginTransmission(ADS_Address); //first byte: call to ADS
      Wire.write(ADS_ConfigRegister); //Second byte: call to internal config reg
      Wire.write(MSB_A0); //third byte: MSB of config reg to be written
      Wire.write(LSB); //fourth byte: LSB of config reg to be written
      Wire.endTransmission();
//      pinString[0] = 'A'; pinString[1] = '0'; //for Serial monitor purposes ONLY
      pinOut = pinRead++; //switch which sensor to read from
    }
    else {
      Wire.beginTransmission(ADS_Address); //first byte: call to ADS
      Wire.write(ADS_ConfigRegister); //Second byte: call to internal config reg
      Wire.write(MSB_A1); //third byte: MSB of config reg to be written
      Wire.write(LSB); //fourth byte: LSB of config reg to be written
      Wire.endTransmission();
//      pinString[0] = 'A'; pinString[1] = '1'; //for serial monitor purposes ONLY
      pinOut = pinRead--; //switch which senso to read from
    }

    //request write data
    Wire.beginTransmission(ADS_Address); //first byte: call to ADS
    Wire.write(ADS_ConversionRegister); //sec byte: point to conver reg
    Wire.endTransmission();

    Wire.requestFrom(ADS_Address, 2); //receive 16 bit(2byte) from ads

    //------------------------------------------------------------------
    // Print User-Friendly shit

//      for(int i = 0; i < 2; i++){
//        Serial.print(pinString[i]);
//      }
//      Serial.print(": ");

    //------------------------------------------------------------------

    //pull and interpret data
    while (Wire.available())
    {
      dataBytes[0] = Wire.read();
      dataBytes[1] = Wire.read();
      data = dataBytes[0] * 256 + dataBytes[1]; //convert bytes to int
      data = data * (2.048 * 2) / pow(2, 16); //map data to devise resolution
//          Serial.print(data); //for serial monitor purposes 
//          Serial.println(" V"); //for serial monitor purposes ONLY
    }
  }
}
