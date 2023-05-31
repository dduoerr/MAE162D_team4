
#include <boarddefs.h>
#include <IRremote.h>
#include <IRremoteInt.h>
#include <ir_Lego_PF_BitStreamEncoder.h>

#include <avr/wdt.h>
#include <stdio.h>
#include <string.h>
#include <Servo.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include "DeviceDriverSet_xxx0.h"

// pins for the encoder inputs
#define RH_ENCODER_A 18  // ENCODER BROKEN
#define RH_ENCODER_B 39
#define LH_ENCODER_A 19
#define LH_ENCODER_B 38

DeviceDriverSet_Motor myMotors; 
DeviceDriverSet_IRrecv IRremote;

uint8_t IRrecv_button; 

int itemNum; // USED which item to pick up
int turn1dis; // UNUSED where to make first turn (encoder distance value) for picking up

bool turn1dir; // direction for the first turn: left or right
int turn2dis; //UNUSED where to make the second turn for dropping
int turnCount90 = 0; //USED count of turn 90 deg
int turnCount45 = 0; //USED count of turn 45 deg
bool haveItem = false; //USED if already grab the item

// variables for counting lines
int lineCount = 0;
int lineLimitPickup = 0;
int lineLimitDropOff = 0;
boolean lineDetected = false;

// Create a new servo object:
Servo ultrasonic_servo;
Servo slider_servo;
Servo lifting_servo;
Servo gripper_servo;

// COUNTS FOR ENCODER COUNTS both right and left
long Rcounts = 0;
long Lcounts = 0;

//  //Initialize all variable with data type appears in the void loop
long ultrasonicVal, encoderVal;
int turnIRVal_L,turnIRVal_R;

enum {startUp, pickingUp, moving90, moving45, navigateObstacles,droppingOff, endStop, remoteControl };
unsigned char robotState = startUp;


//moving, turning45, pickingup, dropping off


void readEncoder_R() //this function is triggered by the encoder CHANGE, and increments the encoder counter
{ 
  if(digitalRead(RH_ENCODER_B) == digitalRead(RH_ENCODER_A) )
  {
    Rcounts = Rcounts-1; //you may need to redefine positive and negative directions
  }
  else
  {
    Rcounts = Rcounts+1;
  }
}

void readEncoder_L() //this function is triggered by the encoder CHANGE, and increments the encoder counter
{ 
  if(digitalRead(LH_ENCODER_B) == digitalRead(LH_ENCODER_A) )
  {
    Lcounts = Lcounts-1; //you may need to redefine positive and negative directions
  }
  else
  {
    Lcounts = Lcounts+1;
  }
}


void setup(){
  // put your setup code here, to run once:
  Serial.begin(115200); 
  //enable IR communication first 
  IRremote.DeviceDriverSet_IRrecv_Init();
  //ENCODER 
  pinMode(RH_ENCODER_A,INPUT);//initialze encoder pins
  pinMode(RH_ENCODER_B,INPUT);
  pinMode(LH_ENCODER_A,INPUT);
  pinMode(LH_ENCODER_B,INPUT);
  digitalWrite(RH_ENCODER_A, LOW);//initialize pin states
  digitalWrite(RH_ENCODER_B, LOW);
  digitalWrite(LH_ENCODER_A, LOW);
  digitalWrite(LH_ENCODER_B, LOW);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), readEncoder_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), readEncoder_L, CHANGE);

//  ultrasonic_servo.attach(ultrasonicServoPin);
  slider_servo.attach(servoSlider);
  lifting_servo.attach(servoLift);
  gripper_servo.attach(servoGripper);
  
  robotState = startUp;
  ultrasonicVal = 20;
}


//test
void loop(){
  slider_servo.write(0); //drive mechanism forward
  delay(2500);
  slider_servo.write(90); //rest
  delay(1000);

  gripper_servo.write(180); //fully open
  delay(1000);

//  lifting_servo.write(180); //drive mechanism up
//  delay(3000);
//  lifting_servo.write(90); //rest
//  delay(1000);
  


}
