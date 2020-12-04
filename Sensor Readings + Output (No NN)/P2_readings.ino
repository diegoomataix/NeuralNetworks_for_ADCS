////////////////////////////////////////////// Libraries
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

///////////////////////////////////////////// Definitions
MPU6050 mpu;

int16_t gyroX, gyroY, gyroZ, gyroRateX, gyroRateY, gyroRateZ,accX, accY, accZ; 

float gyroAngleX, gyroAngleY, gyroAngleZ, accAngleX, accAngleY, accAngleZ; // double //gyroAngleX=0

float currentAngleX, currentAngleY, currentAngleZ, prevAngleX=0, prevAngleY=0, prevAngleZ=0, error, prevError=0, errorSum=0; //

float gForceX, gForceY, gForceZ;

int motorPower;

unsigned long currTime, prevTime=0, loopTime;


#define sampleTime 0.005


///////////////////////////////////////////// Void Setup
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


  Serial.begin(115200); // set COMM port setting
}

// Void Loop
void loop() {
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;

/////////////////////////////////////////////  Assign MPU6050 readings to the respective int16_t
    // Gyro
  gyroX = mpu.getRotationX();
  gyroY = mpu.getRotationX();
  gyroZ = mpu.getRotationX();
    // Acc
  accX = mpu.getAccelerationX();
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();
  
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
  gForceX = accX / 16388.0;  // 16388 is the value that the accel would show under 1g
  gForceY = accY / 16388.0; 
  gForceZ = accZ / 16388.0;
  
///////////////////////////////////////////// Complementary Filter  (Combine accel + gyro data)
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

/*
float nnInputX = currentAngleX ;
float nnInputY = currentAngleY ;
float nnInputZ = currentAngleZ ;

// set motor power after constraining it //

motorPower = Output[0] * 100; // convert from 0 to 1

// if (motorPower < 50) motorPower = motorPower * -1;

motorPower = map(motorPower,0, 100, -300, 300);

motorPower = motorPower + (motorPower * 6.0); // need multiplier to get wheels spinning fast enough when close to balance point

//Serial.print("motorPower=");Serial.println(motorPower);

motorPower = constrain(motorPower, -255, 255);

prevAngle = currentAngle;

previousMillis = currentMillis;

} // end millis loop

// if (abs(error) > 30) motorPower = 0; // if fall over then shut off motors

//motorPower = motorPower + error;

setMotors(motorPower , motorPower );

}

} //end of drive_nn() function
  */ 

///////////////////////////////////////////// PRINT DDATA

  Serial.print(" Angle (deg): ");
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

  Serial.println (" ");

    delay(1000);
  }
