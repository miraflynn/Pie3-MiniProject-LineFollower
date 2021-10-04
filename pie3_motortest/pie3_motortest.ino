#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);

void setup() {
  // put your setup code here, to run once:
  AFMS.begin();
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);

}

void loop() {
  // put your main code here, to run repeatedly:
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}
