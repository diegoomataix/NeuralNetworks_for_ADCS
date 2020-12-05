/*
===Hardware===
- Arduino Uno R3
- MPU-6050 /GY-521
- DC MOTOR
- L293D

===Software===
- Arduino IDE v1.8.42
- Arduino Wire library
- I2Cdev.h --> https://github.com/jrowberg/i2cdevlib
- MPU6050.h -->  https://github.com/jrowberg/i2cdevlib
 */

/* INCLUDE LIBRARIES */
#include <Wire.h>
#include<I2Cdev.h>
#include<MPU6050.h>
#include <math.h>

/* SET PARAMETER NAMES */

MPU6050 mpu;

//long gyroX, gyroY, gyroZ;
//float rotX, rotY, rotZ;
//int16_t ax, ay, az;
//int16_t gx, gy, gz;
long accelX, accelY, accelZ;

float gForceX, gForceY, gForceZ;
int16_t gyroX, gyroY, gyroZ, gyroRateX, gyroRateY, gyroRateZ, accX, accY, accZ; 
float gyroAngleX, gyroAngleY, gyroAngleZ, accAngleX, accAngleY, accAngleZ; // double //gyroAngleX=0
float currentAngleX, currentAngleY, currentAngleZ, prevAngleX=0, prevAngleY=0, prevAngleZ=0, error, prevError=0, errorSum=0; //
unsigned long currTime, prevTime=0, loopTime;

#define sampleTime 0.005

// Motor A connections
#define enA 9
#define in1 8
#define in2 7
// Motor B connections
#define enB 3
#define in3 5
#define in4 4

//// Motor A connections
//int enA = 9;
//int in1 = 8;
//int in2 = 7;
//// Motor B connections
//int enB = 3;
//int in3 = 5;
//int in4 = 4;

/* VOID SETUP */
void setup() {
  mpu.initialize();
          /////////////////////////////////////// ACCEL
  mpu.setXAccelOffset(-2842); // from calibration routine
  mpu.setYAccelOffset(-21); // from calibration routine
  mpu.setZAccelOffset(1088); // from calibration routine
  
        ////////////////////////////////////// GYRO
  mpu.setXGyroOffset(25); // from calibration routine
  mpu.setYGyroOffset(-24); // from calibration routine
  mpu.setZGyroOffset(5); // from calibration routine
  
  Serial.begin(115200);
  Wire.begin();
 
  //setupMPU();
  //pinMode(pin2,OUTPUT);

  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

}

/* -------------------------- SETUP -------------------------- */
void loop() {
  setupMPU();
  mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ); ////  Assign MPU6050 readings to the respective int16_t
  recordAccelGryoRegisters();
  complementaryFilter();
  directionControl();
  printData();
}

/* ------------------------------ setupMPU ---------------------------------- */
void setupMPU(){

  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;
  
  accX = map(accX, -17000, 17000, -1500, 1500);
  accY = map(accY, -17000, 17000, -1500, 1500);
  accZ = map(accZ, -17000, 17000, -1500, 1500);
}

/* -------------------------- recordAcceGyrolRegisters -------------------------- */
void recordAccelGryoRegisters() {
    // Calculate Angle of Inclination
/* atan2(y,z) function gives the angle in radians between the positive z-axis of a plane
and the point given by the coordinates (z,y) on that plane, with positive sign for 
counter-clockwise angles (right half-plane, y > 0), and negative sign for clockwise 
angles (left half-plane, y < 0). */
  accAngleX = atan2(accY, accZ)*RAD_TO_DEG; 
  accAngleY = atan2(accX, accZ)*RAD_TO_DEG;
  accAngleZ = atan2(accY, accX)*RAD_TO_DEG;

  gyroRateX = map(gyroX, -32776, 32776, -250, 250);
  gyroRateY = map(gyroY, -32776, 32776, -250, 250);
  gyroRateZ = map(gyroZ, -32776, 32776, -250, 250);
  
  //gyroAngleX = gyroAngleX + (float)gyroRateX*loopTime/1000;
  gyroAngleX = (float)gyroRateX*sampleTime;
  gyroAngleY = (float)gyroRateY*sampleTime;
  gyroAngleZ = (float)gyroRateZ*sampleTime;
  
  // Calculate Acceleration
  gForceX = accX / 16388.0;  // 16388 is the value that the accel would show when subject to 1g acceleration
  gForceY = accY / 16388.0; 
  gForceZ = accZ / 16388.0;
  
  delay(100);
}

/* -------------------------- complementaryFilter -------------------------- */
void complementaryFilter() {
/*We have two measurements of the angle from two different sources. The measurement from accelerometer gets 
 * affected by sudden horizontal movements and the measurement from gyroscope gradually drifts away from actual value. 
 * In other words, the accelerometer reading gets affected by short duration signals and the gyroscope reading by long 
 * duration signals.  */
 
/* HPF --> gyroscope, LPF --> accelerometer to filter out the drift and noise from the measurement.
0.9934 and 0.0066 are filter coefficients for a filter time constant of 0.75s. The low pass filter allows any signal longer 
than this duration to pass through it and the high pass filter allows any signal shorter than this duration to pass through. 
The response of the filter can be tweaked by picking the correct time constant. Lowering the time constant will allow more 
horizontal acceleration to pass through. */
  currentAngleX = 0.9934*(prevAngleX + gyroAngleX) + 0.0066*(accAngleX); 
  currentAngleY = 0.9934*(prevAngleY + gyroAngleY) + 0.0066*(accAngleY);
  currentAngleZ = 0.9934*(prevAngleZ + gyroAngleZ) + 0.0066*(accAngleZ);
}

/* -------------------------- directionControl -------------------------- */
void directionControl() {
  // Set motors to maximum speed
  // For PWM maximum possible values are 0 to 255
    analogWrite(enA, 255);
    analogWrite(enB, 255);

    /* ---------- Control X-direction ----------*/
    if(accX > 0){
    if(accX < 255){
      //Serial.println(accX);
      analogWrite(in2,accX);
      analogWrite(enA, 255);
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      //delay(2000);
    }
    else{
      //Serial.println("+255");
      analogWrite(in2,255);
      //analogWrite(enA, 0);
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }
    
    }
    if(accX < 0){
    if(accX > -255){
      //Serial.println(accX);
      analogWrite(in1, accX-accX-accX);
      analogWrite(enA, 255);
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
    else{
      //analogWrite(enA, 0);
      //Serial.println("-255");
      //analogWrite(in1, 255);
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }
    } 

    /* ---------- Control Y-direction ----------*/
    if(accY > 0){
    if(accY < 255){
      //analogWrite(enB, 255);
      //Serial.println(accY);
      analogWrite(in4,accY);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      //delay(2000);
      }
    else{
      //analogWrite(enB, 0);  
      //Serial.println("+255");
      analogWrite(in4,255);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
    }
    
    }
    if(accY < 0){
    if(accY > -255){
      //analogWrite(enB, 255);
      //Serial.println(accY);
      analogWrite(in3, accY-accY-accY);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
    }
    else{
      //analogWrite(enB, 255);
      //Serial.println("-255");
      analogWrite(in3, 255);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
    }
    } 
    
    delay(1);
}

/* -------------------------- printData -------------------------- */
void printData() {        /* PRINT DATA */


  Serial.print(" acc (RAW): ");
  Serial.print (" X = ");
  Serial.print( accX);
  Serial.print (" Y = ");
  Serial.print( accY);
  Serial.print (" Z = ");
  Serial.print( accZ);

  Serial.println (" ");
  Serial.print(" Angle, accel (deg): ");
  Serial.print (" X = ");
  Serial.print( accAngleX);
  Serial.print (" Y = ");
  Serial.print( accAngleY);
  Serial.print (" Z = ");
  Serial.print( accAngleZ);

  Serial.println (" ");

  Serial.print(" Angle, gyro (deg): ");
  Serial.print (" X = ");
  Serial.print( gyroAngleX);
  Serial.print (" Y = ");
  Serial.print( gyroAngleY);
  Serial.print (" Z = ");
  Serial.print( gyroAngleZ);

  Serial.println (" ");
  Serial.println (" ");

  Serial.print(" Angle, filtered (deg): ");
  Serial.print (" X = ");
  Serial.print( currentAngleX);
  Serial.print (" Y = ");
  Serial.print( currentAngleY);
  Serial.print (" Z = ");
  Serial.print( currentAngleZ);

  Serial.println (" ");
  
  Serial.print(" Accel (g): ");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.println(gForceZ);

  Serial.println (" ");

  Serial.println (" ---------------------------------------------------------- ");

    delay(100);
}
