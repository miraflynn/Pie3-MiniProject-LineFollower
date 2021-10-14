#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);

int leftSensor = A0;
int rightSensor = A1;

bool seesTape(int sensorPin){
  int sv = analogRead(sensorPin);
  if(sv > 512){
    return true;
  } else {
    return false;
  }
}

void turnRight(){
  leftMotor->setSpeed(100);
  rightMotor->setSpeed(100);
  leftMotor->run(BACKWARD);
  rightMotor->run(FORWARD);
}

void turnLeft(){
  leftMotor->setSpeed(100);
  rightMotor->setSpeed(100);
  leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);
}

void driveForward(){
  leftMotor->setSpeed(100);
  rightMotor->setSpeed(100);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}

void setup() {
  // put your setup code here, to run once:
  AFMS.begin();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  //turnRight();
int SensorReading = analogRead(leftSensor);
Serial.println(SensorReading);
  
  if (seesTape(leftSensor)){
   Serial.println("sees tape");
  }
   else {
   Serial.println("no tape");
   
   }
   delay(200);
   
  }
  