#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);

//Inititalize Inputs
int leftSensor = A0;
int rightSensor = A1;

//Set Initial Constants: 
int  fSpeed = 30;
int calibrationdifference = -32;
float turnFactor = 0.01; //Higher means more sensitive/turns more

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete


void drive(int lMotor, int rMotor){
  leftMotor->setSpeed(lMotor);
  rightMotor->setSpeed(rMotor);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}



void setup() {
  // put your setup code here, to run once:
  AFMS.begin();
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  inputString.reserve(200); // reserve 200 bytes = 200 characters for the input string
}


void loop() {
//Serial Port Controls:
  uint8_t incoming;
  incoming = Serial.read();
  
  if (incoming == '+'){
    fSpeed = fSpeed + 10;
    Serial.print("New Speed = ");
    Serial.println(fSpeed);
  }

   if (incoming == '-'){
    fSpeed = fSpeed - 10;
    Serial.print("New Speed = ");
    Serial.println(fSpeed);
   }

 if (incoming == 'u'){
    turnFactor = turnFactor + 0.01;
    Serial.print("New P = ");
    Serial.println(turnFactor);
  }

if (incoming == 'd'){
    turnFactor = turnFactor - 0.01;
    Serial.print("New P = ");
    Serial.println(turnFactor);
  }

  
  // put your main code here, to run repeatedly:
  int ls = analogRead(leftSensor);
  int rs = analogRead(rightSensor);
  int diff = (ls-rs-calibrationdifference);
  int turnSpeed = diff*fSpeed*turnFactor;

  drive(fSpeed - turnSpeed, fSpeed+turnSpeed);
  //Serial.print("driving");
 // Serial.print(forwardSpeed-turnSpeed);
  //Serial.println(forwardSpeed+turnSpeed);

  if (stringComplete) {
    Serial.print(inputString);
    // clear the string:
    inputString = "";
    stringComplete = false;
    Serial.println("Hello computer!");
  }
}
