/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// neural net program using sigmoid function and applied to simple self balancing robot

// created by Jim Demello, Shangluo University, May 2018

// adapted Sean Hodgins neural net code: https://www.instructables.com/id/Arduino-Neural-Ne...

// adapted midhun_s self balancing robot code: https://www.instructables.com/id/Arduino-Self-Bala...

// built my own self balancing robot

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "MPU6050.h"

#include "math.h"

/******************************************************************

Network Configuration - customized per network

******************************************************************/

const int PatternCount = 2;

const int InputNodes = 1;

const int HiddenNodes = 3;

const int OutputNodes = 1;

const float LearningRate = 0.3;

const float Momentum = 0.9;

const float InitialWeightMax = 0.5;

const float Success = 0.0015;

float Input[PatternCount][InputNodes] = {

{ 0 }, // left lean

{ 1 } // right lean

// { -1} // left lean

// { 0, 1, 1, 0 }, // LIGHT ON LEFT AND RIGHT

// { 0, 1, 0, 0 }, // LIGHT ON LEFT

// { 1, 1, 1, 0 }, // LIGHT ON TOP, LEFT, and RIGHT

};

const float Target[PatternCount][OutputNodes] = {

{ 0, }, // left lean

{ 1, } // right lean

//{ -1, } // movement on left

// { 0.65, 0.55 }, //LEFT MOTOR SLOW

// { 0.75, 0.5 }, //LEFT MOTOR FASTER

};

/******************************************************************

End Network Configuration

******************************************************************/

int i, j, p, q, r;

int ReportEvery1000;

int RandomizedIndex[PatternCount];

long TrainingCycle;

float Rando;

float Error = 2;

float Accum;

float Hidden[HiddenNodes];

float Output[OutputNodes];

float HiddenWeights[InputNodes + 1][HiddenNodes];

float OutputWeights[HiddenNodes + 1][OutputNodes];

float HiddenDelta[HiddenNodes];

float OutputDelta[OutputNodes];

float ChangeHiddenWeights[InputNodes + 1][HiddenNodes];

float ChangeOutputWeights[HiddenNodes + 1][OutputNodes];

#define leftMotorPWMPin 6

#define leftMotorDirPin 7

#define rightMotorPWMPin 5

#define rightMotorDirPin 4

#define sampleTime 0.005

MPU6050 mpu;

int16_t accY, accZ, gyroX;

int motorPower, gyroRate;

float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;

byte count=0;

long previousMillis = 0;

unsigned long currentMillis;

long loopTimer = 4;

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {

// Serial.print("leftMotorSpeed= ");Serial.print(leftMotorSpeed); Serial.print("rightMotorSpeed= ");Serial.println(rightMotorSpeed);

if(leftMotorSpeed >= 0) {

analogWrite(leftMotorPWMPin, leftMotorSpeed);

digitalWrite(leftMotorDirPin, LOW);

}

else { // if leftMotorSpeed is < 0 then set dir to reverse

analogWrite(leftMotorPWMPin, 255 + leftMotorSpeed);

digitalWrite(leftMotorDirPin, HIGH);

}

if(rightMotorSpeed >= 0) {

analogWrite(rightMotorPWMPin, rightMotorSpeed);

digitalWrite(rightMotorDirPin, LOW);

}

else {

analogWrite(rightMotorPWMPin, 255 + rightMotorSpeed);

digitalWrite(rightMotorDirPin, HIGH);

}

}

void setup() {

Serial.begin(115200);

Serial.println("Starting program");

randomSeed(analogRead(A1)); //Collect a random ADC sample for Randomization.

ReportEvery1000 = 1;

for ( p = 0 ; p < PatternCount ; p++ ) {

RandomizedIndex[p] = p ;

}

Serial.println("do train_nn");

train_nn();

delay(1000);

// set the motor control and PWM pins to output mode

pinMode(leftMotorPWMPin, OUTPUT);

pinMode(leftMotorDirPin, OUTPUT);

pinMode(rightMotorPWMPin, OUTPUT);

pinMode(rightMotorDirPin, OUTPUT);

// initialize the MPU6050 and set offset values

mpu.initialize();

mpu.setYAccelOffset(2113); // from calibration routine

mpu.setZAccelOffset(1122);

mpu.setXGyroOffset(7);

Serial.print("End Initialize MPU at: ");Serial.println(millis());

}

///////////////

// main loop /

/////////////

void loop() {

drive_nn();

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//USES TRAINED NEURAL NETWORK TO DRIVE ROBOT

void drive_nn()

{

Serial.println("Running NN Drive ");

while (Error < Success) {

currentMillis = millis();

float TestInput[] = {0, 0};

if(currentMillis - previousMillis > loopTimer) { //do calculation every 5 or more milliseconds

Serial.print("currentMillis= ");Serial.println(currentMillis);

////////////////////////////////////////

// calculate the angle of inclination //

////////////////////////////////////////

accY = mpu.getAccelerationY();

accZ = mpu.getAccelerationZ();

gyroX = mpu.getRotationX();

accAngle = atan2(accY, accZ)*RAD_TO_DEG;

gyroRate = map(gyroX, -32768, 32767, -250, 250);

gyroAngle = (float)gyroRate*sampleTime;

///////////////////////////////////////////////////////////////////

// complementary filter ///////////////////////////////////////////

//////////////////////////////////////////////////////////////////

currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);

//Serial.print("currentAngle= ");Serial.print(currentAngle);Serial.print(" error=");Serial.println(error);

//error = currentAngle - targetAngle; // not used

float nnInput = currentAngle ;

//Serial.print(" nnInput=");Serial.println(nnInput);

nnInput = map(nnInput,-30 ,30 , 0, 100); // map tilt angle range to 0 to 100

TestInput[0] = float(nnInput) / 100; //convert to 0 to 1

// Serial.print("testinput=");Serial.println(TestInput[0]);

InputToOutput(TestInput[0]); //INPUT to ANN to obtain OUTPUT

//Serial.print("output=");Serial.println(Output[0]);

///////////////////////////////////////////

// set motor power after constraining it //

///////////////////////////////////////////

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

//DISPLAYS INFORMATION WHILE TRAINING

void toTerminal()

{

for ( p = 0 ; p < PatternCount ; p++ ) {

Serial.println();

Serial.print (" Training Pattern: ");

Serial.println (p);

Serial.print (" Input ");

for ( i = 0 ; i < InputNodes ; i++ ) {

Serial.print (Input[p][i], DEC);

Serial.print (" ");

}

Serial.print (" Target ");

for ( i = 0 ; i < OutputNodes ; i++ ) {

Serial.print (Target[p][i], DEC);

Serial.print (" ");

}

/******************************************************************

Compute hidden layer activations

******************************************************************/

for ( i = 0 ; i < HiddenNodes ; i++ ) {

Accum = HiddenWeights[InputNodes][i] ;

for ( j = 0 ; j < InputNodes ; j++ ) {

Accum += Input[p][j] * HiddenWeights[j][i] ;

}

Hidden[i] = 1.0 / (1.0 + exp(-Accum)) ; // activation function

}

/******************************************************************

Compute output layer activations and calculate errors

******************************************************************/

for ( i = 0 ; i < OutputNodes ; i++ ) {

Accum = OutputWeights[HiddenNodes][i] ;

for ( j = 0 ; j < HiddenNodes ; j++ ) {

Accum += Hidden[j] * OutputWeights[j][i] ;

}

Output[i] = 1.0 / (1.0 + exp(-Accum)) ;

}

Serial.print (" Output ");

for ( i = 0 ; i < OutputNodes ; i++ ) {

Serial.print (Output[i], 5);

Serial.print (" ");

}

}

}

void InputToOutput(float In1)

{

float TestInput[] = {0};

TestInput[0] = In1;

// TestInput[1] = In2; // not used

// TestInput[2] = In3; // not used

// TestInput[3] = In4; // not used

/******************************************************************

Compute hidden layer activations

******************************************************************/

for ( i = 0 ; i < HiddenNodes ; i++ ) {

Accum = HiddenWeights[InputNodes][i] ;

for ( j = 0 ; j < InputNodes ; j++ ) {

Accum += TestInput[j] * HiddenWeights[j][i] ;

}

Hidden[i] = 1.0 / (1.0 + exp(-Accum)) ;

}

/******************************************************************

Compute output layer activations and calculate errors

******************************************************************/

for ( i = 0 ; i < OutputNodes ; i++ ) {

Accum = OutputWeights[HiddenNodes][i] ;

for ( j = 0 ; j < HiddenNodes ; j++ ) {

Accum += Hidden[j] * OutputWeights[j][i] ;

}

Output[i] = 1.0 / (1.0 + exp(-Accum)) ;

}

//#ifdef DEBUG

Serial.print (" Output ");

for ( i = 0 ; i < OutputNodes ; i++ ) {

Serial.print (Output[i], 5);

Serial.print (" ");

}

//#endif

}

//TRAINS THE NEURAL NETWORK

void train_nn() {

/******************************************************************

Initialize HiddenWeights and ChangeHiddenWeights

******************************************************************/

int prog_start = 0;

Serial.println("start training...");

//digitalWrite(LEDYEL, LOW);

for ( i = 0 ; i < HiddenNodes ; i++ ) {

for ( j = 0 ; j <= InputNodes ; j++ ) {

ChangeHiddenWeights[j][i] = 0.0 ;

Rando = float(random(100)) / 100;

HiddenWeights[j][i] = 2.0 * ( Rando - 0.5 ) * InitialWeightMax ;

}

}

//digitalWrite(LEDYEL, HIGH);

/******************************************************************

Initialize OutputWeights and ChangeOutputWeights

******************************************************************/

//digitalWrite(LEDRED, LOW);

for ( i = 0 ; i < OutputNodes ; i ++ ) {

for ( j = 0 ; j <= HiddenNodes ; j++ ) {

ChangeOutputWeights[j][i] = 0.0 ;

Rando = float(random(100)) / 100;

OutputWeights[j][i] = 2.0 * ( Rando - 0.5 ) * InitialWeightMax ;

}

}

//digitalWrite(LEDRED, HIGH);

//SerialUSB.println("Initial/Untrained Outputs: ");

//toTerminal();

/******************************************************************

Begin training

******************************************************************/

for ( TrainingCycle = 1 ; TrainingCycle < 2147483647 ; TrainingCycle++) {

/******************************************************************

Randomize order of training patterns

******************************************************************/

for ( p = 0 ; p < PatternCount ; p++) {

q = random(PatternCount);

r = RandomizedIndex[p] ;

RandomizedIndex[p] = RandomizedIndex[q] ;

RandomizedIndex[q] = r ;

}

Error = 0.0 ;

/******************************************************************

Cycle through each training pattern in the randomized order

******************************************************************/

for ( q = 0 ; q < PatternCount ; q++ ) {

p = RandomizedIndex[q];

/******************************************************************

Compute hidden layer activations

******************************************************************/

//digitalWrite(LEDYEL, LOW);

for ( i = 0 ; i < HiddenNodes ; i++ ) {

Accum = HiddenWeights[InputNodes][i] ;

for ( j = 0 ; j < InputNodes ; j++ ) {

Accum += Input[p][j] * HiddenWeights[j][i] ;

}

Hidden[i] = 1.0 / (1.0 + exp(-Accum)) ;

}

//digitalWrite(LEDYEL, HIGH);

/******************************************************************

Compute output layer activations and calculate errors

******************************************************************/

//digitalWrite(LEDRED, LOW);

for ( i = 0 ; i < OutputNodes ; i++ ) {

Accum = OutputWeights[HiddenNodes][i] ;

for ( j = 0 ; j < HiddenNodes ; j++ ) {

Accum += Hidden[j] * OutputWeights[j][i] ;

}

Output[i] = 1.0 / (1.0 + exp(-Accum)) ;

OutputDelta[i] = (Target[p][i] - Output[i]) * Output[i] * (1.0 - Output[i]) ;

Error += 0.5 * (Target[p][i] - Output[i]) * (Target[p][i] - Output[i]) ;

}

// Serial.println(Output[0]*100);

//digitalWrite(LEDRED, HIGH);

/******************************************************************

Backpropagate errors to hidden layer

******************************************************************/

//digitalWrite(LEDYEL, LOW);

for ( i = 0 ; i < HiddenNodes ; i++ ) {

Accum = 0.0 ;

for ( j = 0 ; j < OutputNodes ; j++ ) {

Accum += OutputWeights[i][j] * OutputDelta[j] ;

}

HiddenDelta[i] = Accum * Hidden[i] * (1.0 - Hidden[i]) ;

}

//digitalWrite(LEDYEL, HIGH);

/******************************************************************

Update Inner-->Hidden Weights

******************************************************************/

//digitalWrite(LEDRED, LOW);

for ( i = 0 ; i < HiddenNodes ; i++ ) {

ChangeHiddenWeights[InputNodes][i] = LearningRate * HiddenDelta[i] + Momentum * ChangeHiddenWeights[InputNodes][i] ;

HiddenWeights[InputNodes][i] += ChangeHiddenWeights[InputNodes][i] ;

for ( j = 0 ; j < InputNodes ; j++ ) {

ChangeHiddenWeights[j][i] = LearningRate * Input[p][j] * HiddenDelta[i] + Momentum * ChangeHiddenWeights[j][i];

HiddenWeights[j][i] += ChangeHiddenWeights[j][i] ;

}

}

//digitalWrite(LEDRED, HIGH);

/******************************************************************

Update Hidden-->Output Weights

******************************************************************/

//digitalWrite(LEDYEL, LOW);

for ( i = 0 ; i < OutputNodes ; i ++ ) {

ChangeOutputWeights[HiddenNodes][i] = LearningRate * OutputDelta[i] + Momentum * ChangeOutputWeights[HiddenNodes][i] ;

OutputWeights[HiddenNodes][i] += ChangeOutputWeights[HiddenNodes][i] ;

for ( j = 0 ; j < HiddenNodes ; j++ ) {

ChangeOutputWeights[j][i] = LearningRate * Hidden[j] * OutputDelta[i] + Momentum * ChangeOutputWeights[j][i] ;

OutputWeights[j][i] += ChangeOutputWeights[j][i] ;

}

}

//digitalWrite(LEDYEL, HIGH);

}

/******************************************************************

Every 100 cycles send data to terminal for display and draws the graph on OLED

******************************************************************/

ReportEvery1000 = ReportEvery1000 - 1;

if (ReportEvery1000 == 0)

{

int graphNum = TrainingCycle / 100;

int graphE1 = Error * 1000;

int graphE = map(graphE1, 3, 80, 47, 0);

Serial.print ("TrainingCycle: ");

Serial.print (TrainingCycle);

Serial.print (" Error = ");

Serial.println (Error, 5);

toTerminal();

if (TrainingCycle == 1)

{

ReportEvery1000 = 99;

}

else

{

ReportEvery1000 = 100;

}

}

/******************************************************************

If error rate is less than pre-determined threshold then end

******************************************************************/

if ( Error < Success ) break ;

}

}
