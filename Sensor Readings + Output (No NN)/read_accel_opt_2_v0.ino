/*
===Hardware===
- Arduino Uno R3
- MPU-6050 /GY-521
- DC MOTOR
- L293D

===Software===
- Arduino IDE v1.6.9
- Arduino Wire library
- I2Cdev.h --> https://github.com/jrowberg/i2cdevlib
- MPU6050.h -->  https://github.com/jrowberg/i2cdevlib
 */

///* INCLUDE LIBRARIES */
//#include <Wire.h>
//#include<I2Cdev.h>
//#include<MPU6050.h>
//
///* Declare accelerometer/gyroscope */
//MPU6050 mpu;
//
///* SET PARAMETER NAMES */
//
//int16_t ax, ay, az;
//int16_t gx, gy, gz;
//
///* DEFINE DIGITAL PIN INPUTS */
//#define pin1 3
//#define pin2 5
//
///* SETUP */
//void setup(){
//Serial.begin(9600);
//
//Serial.println("Initialize MPU");
//mpu.initialize();
////Serial.println(mpu.testConnection() ? "Connected" : "Connection failed"); pinMode(pin1,OUTPUT);
//pinMode(pin2,OUTPUT);
//
//}
//
//void loop(){
//  
//mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//
//ax = map(ax, -17000, 17000, -1500, 1500);
//
////Serial.println(ax);
//if(ax > 0){
//if(ax<255){
//Serial.println(ax);
//analogWrite(pin2,ax);
//}
//else{
//Serial.println("+255");
//analogWrite(pin2,255);
//}
//
//}
//if(ax<0){
//if(ax>-255){
//Serial.println(ax);
//analogWrite(pin1, ax-ax-ax);
//}
//else{
//Serial.println("-255");
//analogWrite(pin1, 255);
//}
//}
//delay(1000);
//
//
//// void printData() {        /* PRINT DATA */
//  Serial.print("Gyro (deg)");
//  Serial.print(" X=");
//  Serial.print(ax);
//  Serial.print(" Y=");
//  Serial.print(ay);
//  Serial.print(" Z=");
//  Serial.print(az);
//  Serial.print(" Accel (g)");
//  Serial.print(" X=");
//  Serial.print(gx);
//  Serial.print(" Y=");
//  Serial.print(gy);
//  Serial.print(" Z=");
//  Serial.println(gz);
////  }

// ================================================================================================

/*
   MPU-6050 Test

   This simple program reads and prints to the Serial Monitor window
   the raw X/Y/Z values for the accelerometer and the gyro
   It also calculates the pitch and roll values as well as the temperature
   in F and C.
    
   Connect VDD to 5V and GND to ground on the MCU
   Connect SCL to SCL on MCU and SDA to SDA on MCU

  Note that the correction values can be used to put in an offset to adjust the
  values toward 0 or in the case of the temperature to adjust it to match a
  reference temperature measurement device.
*/
#include<Wire.h>
#include <math.h>
const int MPU=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
double pitch,roll, yaw;
//===============================================================================
//  Initialization
//===============================================================================
void setup(){
Wire.begin();
Wire.beginTransmission(MPU);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
Serial.begin(9600);
}
//===============================================================================
//  Main
//===============================================================================
void loop(){
Wire.beginTransmission(MPU);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU,14,true);

int AcXoff,AcYoff,AcZoff,GyXoff,GyYoff,GyZoff;
int temp,toff;
double t,tx,tf;

//Acceleration data correction
AcXoff = -250+500;
AcYoff = 36+400;
AcZoff = 1200-17444;

//Temperature correction
toff = -1400;

//Gyro correction
GyXoff = -335+482-38;
GyYoff = 250-350;
GyZoff = 170-140;

//read accel data and apply correction
AcX=(Wire.read()<<8|Wire.read()) + AcXoff;
AcY=(Wire.read()<<8|Wire.read()) + AcYoff;
AcZ=(Wire.read()<<8|Wire.read()) + AcZoff;

//read temperature data & apply correction
temp=(Wire.read()<<8|Wire.read()) + toff;

//read gyro data & apply correction
GyX=(Wire.read()<<8|Wire.read()) + GyXoff;
GyY=(Wire.read()<<8|Wire.read()) + GyYoff;
GyZ=(Wire.read()<<8|Wire.read()) + GyZoff;

// Calculate and convert temperature
tx=temp;
t = tx/340 + 36.53;     // Formula from data sheet
tf = (t * 9/5) + 32;    // Standard C to F conversion

//get pitch/roll
getAngle(AcX,AcY,AcZ);

//send the data out the serial port
Serial.print("Angle: ");
Serial.print("Pitch = "); Serial.print(pitch);
Serial.print(" | Roll = "); Serial.println(roll);
Serial.print(" | Yaw = "); Serial.println(yaw);

Serial.print("Temp: ");
// Serial.print("Temp(F) = "); Serial.print(tf);
Serial.print(" | Temp(C) = "); Serial.println(t);

Serial.print("Accelerometer: ");
Serial.print("X = "); Serial.print(AcX);
Serial.print(" | Y = "); Serial.print(AcY);
Serial.print(" | Z = "); Serial.println(AcZ);

Serial.print("Gyroscope: ");
Serial.print("X = "); Serial.print(GyX);
Serial.print(" | Y = "); Serial.print(GyY);
Serial.print(" | Z = "); Serial.println(GyZ);
Serial.println(" ");
delay(1000);
}
//===============================================================================
//  GetAngle - Converts accleration data to pitch & roll
//===============================================================================
void getAngle(int Vx,int Vy,int Vz) {
double x = Vx;
double y = Vy;
double z = Vz;
pitch = atan(x/sqrt((y*y) + (z*z)));
roll = atan(y/sqrt((x*x) + (z*z)));
yaw = atan(z/sqrt((x*x) + (z*z)));
//convert radians into degrees
pitch = pitch * (180.0/3.14);
roll = roll * (180.0/3.14) ;
yaw = yaw * (180.0/3.14) ;
}
