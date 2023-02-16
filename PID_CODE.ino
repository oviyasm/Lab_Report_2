#include <Wire.h>


// I2C address of the Arduino Nano
#define NANO_ADDRESS 0x04

// ADC Pin assignments for sensors
#define SENSOR1 34 //Left sensor
#define SENSOR2 35 //Left sensor
#define SENSOR3 25 //Middle left
#define SENSOR4 26 //Middle right
#define SENSOR5 27 //Right
#define SENSOR6 14 //Right

int baseSpeed = 150; 

int leftSpeed = 0;
int rightSpeed = 0;
int servoPos = 63;


int midServo = 63; //EEEBot center steering

int setpoint = 0; //Centre of car as setpoint

float K = 0.166;  //Motor speed scaling factor

// Distance between sensor and centre of car

float weight1 = -39.0;
float weight2 = -24.5;
float weight3 = -8.5;
float weight4 = 8.5;
float weight5 = 24.5;
float weight6 = 39.0;

// Calibration variables
int min_1, min_2, min_3, min_4, min_5, min_6;
int max_1, max_2, max_3, max_4, max_5, max_6;
 
// PID variables
double Setpoint;//, Input, Output;
double Kp= 116.5, Ki=0, Kd=0;




void setup() {
  // Start the I2C communication
  Wire.begin();
  Serial.begin(9600);

  // Set the pin modes for the sensor pairs
  pinMode(SENSOR1, INPUT);
  pinMode(SENSOR2, INPUT);
  pinMode(SENSOR3, INPUT);
  pinMode(SENSOR4, INPUT);
  pinMode(SENSOR5, INPUT);
  pinMode(SENSOR6, INPUT);

 //Calibrated max and min values
  min_1 = 423;
  min_2 = 659;
  min_3 = 584;
  min_4 = 437;
  min_5 = 791;
  min_6 = 582;

  max_1 = 4095;
  max_2 = 4095;
  max_3 = 4095;
  max_4 = 4095;
  max_5 = 4095;
  max_6 = 4095;


  Setpoint = 0;
}

void loop() {


  // Read the values from all 6 sensors
  int sensor1Value = analogRead(SENSOR1);
  int sensor2Value = analogRead(SENSOR2);
  int sensor3Value = analogRead(SENSOR3);
  int sensor4Value = analogRead(SENSOR4);
  int sensor5Value = analogRead(SENSOR5);
  int sensor6Value = analogRead(SENSOR6);

    //Constrains the values to a specified range
  sensor1Value = constrain(sensor1Value, min_1, max_1);
  sensor2Value = constrain(sensor2Value, min_2, max_2);
  sensor3Value = constrain(sensor3Value, min_3, max_3);
  sensor4Value = constrain(sensor4Value, min_4, max_4);
  sensor5Value = constrain(sensor5Value, min_5, max_5);
  sensor6Value = constrain(sensor6Value, min_6, max_6);

  //Maps the values into the 0 to 255 range
  sensor1Value = map(sensor1Value, min_1, max_1, 0, 255);
  sensor2Value = map(sensor2Value, min_2, max_2, 0, 255);
  sensor3Value = map(sensor3Value, min_3, max_3, 0, 255);
  sensor4Value = map(sensor4Value, min_4, max_4, 0, 255);
  sensor5Value = map(sensor5Value, min_5, max_5, 0, 255);
  sensor6Value = map(sensor6Value, min_6, max_6, 0, 255);

  // Calculate the weighted average of the sensor values
  float num = (sensor1Value * weight1 + sensor2Value * weight2 + sensor3Value * weight3 + sensor4Value * weight4 + sensor5Value * weight5 + sensor6Value * weight6);
  float denum = (sensor1Value + sensor2Value + sensor3Value + sensor4Value + sensor5Value + sensor6Value);

  float weightedAvg = num/denum;

  float error = Setpoint - weightedAvg;
  float sum = 0;
  sum = sum + error;
  float PrevError = 0;
  float dErr = error - PrevError;
  float output = Kp*error + Ki*sum + Kd*dErr;
  PrevError = error;
 

  // Calculate the new speed and steering values
  //Right turns are minus values
  //Left turns are positive values
  
  
  leftSpeed = baseSpeed + (K * output);
  rightSpeed = baseSpeed - (K * output);
  servoPos = midServo + output;

  // Constrain the values to the appropriate range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  servoPos = constrain(servoPos, 25, 85);

  transmitArduino(leftSpeed, rightSpeed, servoPos);
}

  // Send the values to the Arduino Nano

void transmitArduino(int leftSpeed, int rightSpeed, int servoPos)

{

  Wire.beginTransmission(NANO_ADDRESS);  // transmit to device #4

  Wire.write((byte)((leftSpeed & 0x0000FF00) >> 8));  // first byte of leftMotor, containing bits 16 to 9

  Wire.write((byte)(leftSpeed & 0x000000FF));  // second byte of leftMotor, containing the 8 LSB - bits 8 to 1

  Wire.write((byte)((rightSpeed & 0x0000FF00) >> 8));  // first byte of rightMotor, containing bits 16 to 9

  Wire.write((byte)(rightSpeed & 0x000000FF));  // second byte of rightMotor, containing the 8 LSB - bits 8 to 1

  Wire.write((byte)((servoPos & 0x0000FF00) >> 8));  // first byte of rightMotor, containing bits 16 to 9

  Wire.write((byte)(servoPos & 0x000000FF));  // second byte of rightMotor, containing the 8 LSB - bits 8 to 1

  Wire.endTransmission();  // stop transmitting
}
