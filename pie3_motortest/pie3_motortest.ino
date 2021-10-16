#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);

//Inititalize Input Pins:
int leftSensor = A0;
int rightSensor = A1;



float turnFactor = -0.01/30;

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

//Set Initial Constants: 
int  fSpeed = 30; //forward speed motors are run at when driving straight
int calibrationdifference = -32; //Manually set calibration factor that accounts for the resting difference between the incoming value of the two sensors
float turnFactor = 0.001; //Contant to determine feedback sensitivity of turning based on sensor input. Higher means more sensitive/turns more

//Function to drive the motors
void drive(int lMotor, int rMotor){
  leftMotor->setSpeed(abs(lMotor));
  rightMotor->setSpeed(abs(rMotor));
  if(lMotor >= 0){
    leftMotor->run(FORWARD);
  } else {
    leftMotor->run(BACKWARD);
  }
  if(rMotor >= 0){
    rightMotor->run(FORWARD);
  } else {
    rightMotor->run(BACKWARD);
  }
}


void setup() {
  // put your setup code here, to run once:
  
  int leftCal = analogRead(leftSensor);
  int rightCal = analogRead(rightSensor);
  calibrationDifference = leftCal-rightCal;
  
  AFMS.begin();
  Serial.begin(9600);
//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }
// inputString.reserve(200); // reserve 200 bytes = 200 characters for the input string

}


void loop() {

  // put your main code here, to run repeatedly:
  int ls = analogRead(leftSensor);
  int rs = analogRead(rightSensor);
  int diff = (ls-rs-calibrationDifference);
  int turnSpeed = diff*fSpeed*turnFactor;

  Serial.println(turnSpeed);

  //Serial Port Controls:
  uint8_t incoming;
  incoming = Serial.read();

// Change the Speed
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

// Change the Turn Factor
 if (incoming == 'u'){
    turnFactor = turnFactor + 0.001;
    Serial.print("New P = ");
    Serial.println(turnFactor);
  }

if (incoming == 'd'){
    turnFactor = turnFactor - 0.001;
    Serial.print("New P = ");
    Serial.println(turnFactor);
  }

//Read Both Sensor Values and Find Differnce
  int ls = analogRead(leftSensor);
  int rs = analogRead(rightSensor);
  int diff = (ls-rs-calibrationdifference); //calibrated diffence will be how much ls is bigger than rs when both seeing the same terrain
//Calculate turn speed based on total speed and differnece between sensor values and sensitivity to change
  int turnSpeed = diff*fSpeed*turnFactor;

//Drive based on calculated Turn Speed:
  drive(fSpeed-turnSpeed, fSpeed+turnSpeed);
  
 //Serial.print("driving");
 // Serial.print(forwardSpeed-turnSpeed);
 //Serial.println(forwardSpeed+turnSpeed);
}
