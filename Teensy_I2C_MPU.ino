/*
   Author: Justin Francis
   CDate: 1/15/19
   EDate: 1/15/19
   Version: 1.0

   @ read data from MPU 9250 at 500 Hz and 16 bit resolution
*/

#include <Wire.h>

int MPU_Address = 0x69;

//Configuration Register Addresses
byte MPU_PwrConfigReg = 0x6B; //manage wake up, time out, etc
byte MPU_ConfigReg = 0x1A; 
byte MPU_GyroConfig = 0x1B; //self test, scale select (dps), fchoice
byte MPU_AccelConfig = 0x1C; //self test, scale select (g)
byte MPU_AccelConfig2 = 0x1D;//fchoice, low pass filter setting
byte MPU_SampleRateDividerConfigReg = 0x19;

//set up bytes to send to each config register
byte pwrByte = 0x00;
byte configByte = 0x00;
byte gyroByte = 0x00; 
byte accelByte = 0x00;
byte accel2Byte = 0x00;
byte samRateDividerByte = 0x00;


void setup() {
  Wire.begin(); //open up comms
  Serial.begin(9600); //start serial comms at 9600 baud

//  Serial.println("Attempting to communicate with MPU"); //let user know program has started

  //setup power register
  Wire.beginTransmission(MPU_Address);
  Wire.write(MPU_PwrConfigReg);
  Wire.write(pwrByte);
  Wire.endTransmission();

  //setup Config Reg
  Wire.beginTransmission(MPU_Address);
  Wire.write(MPU_ConfigReg);
  Wire.write(configByte);
  Wire.endTransmission();

  //setup Gyro Reg
  Wire.beginTransmission(MPU_Address);
  Wire.write(MPU_GyroConfig);
  Wire.write(gyroByte);
  Wire.endTransmission();

  //setup accel reg
  Wire.beginTransmission(MPU_Address);
  Wire.write(MPU_AccelConfig);
  Wire.write(accelByte);
  Wire.endTransmission();

  //set up accel2 reg
  Wire.beginTransmission(MPU_Address);
  Wire.write(MPU_AccelConfig2);
  Wire.write(accel2Byte);
  Wire.endTransmission();

  //setup sample rate divider config reg
  Wire.beginTransmission(MPU_Address);
  Wire.write(MPU_SampleRateDividerConfigReg);
  Wire.write(samRateDividerByte);
  Wire.endTransmission();

//  Serial.println("Setting Up Configuration...");
}

//Output Sensor Registers
byte ACCEL_XOUT_H = 0x3B;
byte GYRO_XOUT_H = 0x43;
//data storage
signed char accelDataBytes[6];
float accel[3];
signed char gyroDataBytes[6];
float gyro[3];
//timing vars
unsigned long startTime = millis();
unsigned long elapsedTime = 0;
float desiredFreq = 500; //Hz
//needed for tuning frequency
//unsigned long freqStartTime = millis();
//unsigned long freqStopTime;
//float freq = 500;
//int count = 0;

void loop() {

  elapsedTime = millis();
  if (elapsedTime - startTime >= (1 / desiredFreq * 1000))
  {
    //    //code used to determine loop freq, should be relatively stable so no need for continuous update unless its important *ASK LUKAS*
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
//    delay(10);
    Wire.beginTransmission(MPU_Address); //first byte: call to MPU
    Wire.write(ACCEL_XOUT_H); //sec byte: point to sensor reg
    Wire.endTransmission();

    Wire.requestFrom(MPU_Address, 6); //receive data bytes

    //store accelerometer data
    int i = 0;
    while (Wire.available())
    {
      accelDataBytes[i++] = Wire.read();
    }

    Wire.beginTransmission(MPU_Address); //first byte: call to MPU
    Wire.write(GYRO_XOUT_H); //sec byte: point to sensor reg
    Wire.endTransmission();

    Wire.requestFrom(MPU_Address, 6); //receive data bytes (6)

    //store gyroscope data
    int j = 0;
    while (Wire.available())
    {
      gyroDataBytes[j++] = Wire.read();
    }

    //convert bits to decimal [dps or g's]
    for (int i = 0; i < 6; i += 2)
    {
      signed int accelData =  accelDataBytes[i] * 256 + accelDataBytes[i + 1]; //zero offset 
      accel[i / 2] = accelData * 4.0 / pow(2, 16); //4.0 => +-2.0g's (setup in accel config), 2^16 (determined from datasheet)
      signed int gyroData = gyroDataBytes[i] * 256 + gyroDataBytes[i + 1]; //zero offset
      gyro[i / 2] = gyroData * 250.0 / pow(2, 16); //250.0 => +250.0 dps (setup in gyro config), 2^16 (determined from datasheet)
    }


//    for (i = 0; i < 3; i++)
//    {
      //    Serial.print("Gyro ");
      //    Serial.print(i + " ");
      //    Serial.print(gyro[i]);
      //    Serial.println(" dps");
      ////
//      Serial.print("Accel ");
//      Serial.print(i);
//      Serial.print(" ");
//      Serial.print(accel[i]);
//      Serial.println(" g");
//    }
    //  delay(5000);
  }
}
