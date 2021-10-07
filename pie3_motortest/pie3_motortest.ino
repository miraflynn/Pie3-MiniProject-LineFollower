#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);

int leftSensor = A0;
int rightSensor = A1;

int turnSpeed = 50;
int forwardSpeed = 50;

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

bool seesTape(int sensorPin){
  int sv = analogRead(sensorPin);
  if(sv > 800){
    return true;
  } else {
    return false;
  }
}

void turnRight(){
  leftMotor->setSpeed(turnSpeed);
  rightMotor->setSpeed(turnSpeed);
  leftMotor->run(BACKWARD);
  rightMotor->run(FORWARD);
}

void turnLeft(){
  leftMotor->setSpeed(turnSpeed);
  rightMotor->setSpeed(turnSpeed);
  leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);
}

void driveForward(){
  leftMotor->setSpeed(forwardSpeed);
  rightMotor->setSpeed(forwardSpeed);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
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
  // put your main code here, to run repeatedly:
  turnRight();

  bool onTapeLeft = seesTape(leftSensor);
  bool onTapeRight = seesTape(rightSensor);
  if(onTapeLeft){
    turnRight();
  } else if (onTapeRight()){
    turnLeft();
  } else {
    driveForward();
  }

  if (stringComplete) {
    Serial.print(inputString);
    // clear the string:
    inputString = "";
    stringComplete = false;
    Serial.println("Hello computer!");
  }
}
