
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);

//Inititalize Input Pins:
int leftSensor = A0;
int rightSensor = A1;

//Set Initial Constants:
int  fSpeed = 30; //forward speed motors are run at when driving straight
int calibrationdifference = 3; //Manually set calibration factor that accounts for the resting difference between the incoming value of the two sensors
float turnFactor = 0.0017;//Contant to determine feedback sensitivity of turning based on sensor input. Higher means more sensitive/turns more

//Function to drive the motors
void drive(int lMotor, int rMotor){
  leftMotor->setSpeed(lMotor);
  rightMotor->setSpeed(rMotor);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}


void setup() {
  AFMS.begin();
  Serial.begin(9600);
//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }
// inputString.reserve(200); // reserve 200 bytes = 200 characters for the input string



  int ls = analogRead(leftSensor);
  int rs = analogRead(rightSensor);
  int diff = (ls-rs-calibrationdifference);
  Serial.print("left:");
  Serial.println(ls) ;
  Serial.print("right:");
  Serial.println(rs);
  Serial.print("raw diff:");
  Serial.println(ls-rs);
  Serial.print("claibrated dff (should be 0):");
  Serial.println(diff);
  Serial.print("speed:");
  Serial.println(fSpeed);
  Serial.print("p:");
  Serial.println(turnFactor, 4);

}


void loop() {
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
  drive((fSpeed-turnSpeed)*16/17, fSpeed+turnSpeed);

 //Print Motor Speeds:
Serial.print("Left_Motor:");
Serial.print((fSpeed-turnSpeed)); //also multiplied by 16/17 but this is to account for a hardware difference
Serial.print(", ");
Serial.print("Right_Motor:");
Serial.print(fSpeed+turnSpeed);
Serial.print(", ");
Serial.print("Left_Sensor:");
Serial.print(ls);
Serial.print(", ");
Serial.print("Right_Sensor:");
Serial.println(rs);

 

}
