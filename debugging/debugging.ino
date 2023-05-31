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

//Functions

void lineTracking(int slow, int medium, int high){
 
  //go forward
  if((analogRead(R_S) > 100)&&(analogRead(L_S) > 100)){
    myMotors.DeviceDriverSet_Motor_control(true, medium, true, medium, true);
  }

  //accelerate right
  else if(analogRead(R_S) < 100){
    myMotors.DeviceDriverSet_Motor_control(true, slow, true, high, true);
  }

  //accelerate left
  else if(analogRead(L_S) < 100){
    myMotors.DeviceDriverSet_Motor_control(true, high, true, slow, true);
  }

}

void turn45(bool dir){ //only ever turns 45 degrees in the left direction, but left for robustness

    myMotors.DeviceDriverSet_Motor_control(true, 150, true, 150, true);
    delay(500);
    myMotors.DeviceDriverSet_Motor_control(dir, 150, !dir, 150, true);
    delay(2250);
    myMotors.DeviceDriverSet_Motor_control(true, 150, true, 150, true);


}

void turn90(bool dir){ //UPDATE FOR NO WHITE CHECK
    
    //turn 90 degrees, where dir = true is turn left
    //if((analogRead(M_S) < 100)&&(analogRead(R_S) < 100)&&(analogRead(L_S) < 100)){ // seeing white
   // if(digitalRead(ir_sensor_R)){ //ir sensor on the right see a black line
        myMotors.DeviceDriverSet_Motor_control(true, 150, true, 150, true);
        delay(700);
        myMotors.DeviceDriverSet_Motor_control(dir, 150, !dir, 150, true);
        delay(4500);
        myMotors.DeviceDriverSet_Motor_control(true, 150, true, 150, true);
        turnCount90++;
    //}
}

int getUltrasonic(){
   long duration;
   pinMode(ultrasonic_ping, OUTPUT);
   digitalWrite(ultrasonic_ping, LOW);
   delayMicroseconds(2);
   digitalWrite(ultrasonic_ping, HIGH);
   delayMicroseconds(10);
   digitalWrite(ultrasonic_ping, LOW);
   pinMode(ultraonisc_echo, INPUT);
   duration = pulseIn(ultraonisc_echo, HIGH);

   //convert to centimeters
   return duration / 29 / 2;
}


void grabItem(){
  //fill up angle later

  //steps: drive slider servo forward, drive lifter servo up, grab item, then retract slider servo

  slider_servo.write(180); //drive mechanism forward
  delay(2500);
  slider_servo.write(90); //rest
  delay(1000);
//  
//
//  lifting_servo.write(45); // drive down
//  delay(3000);
//  lifting_servo.write(90);
//  delay(1000);
  lifting_servo.write(180); // drive up
  delay(3000);
  lifting_servo.write(90);
  delay(1000);

  gripper_servo.write(180); //fully open
  delay(1000);
  
  
  // progressively close
//  for (int angle = 180; angle >= 0; angle--){
//    if(analogRead(pressurePin) > 600){
//      break;
//    }
//    else{
//      gripper_servo.write(angle);
//      delay(15);
//    }
//    Serial.println(analogRead(pressurePin));
//  } 
//  delay(3000);
//
//
//  gripper_servo.write(180); //let go
//  delay(100);3
//  
//  slider_servo.write(0); //drive mechanism backward
//  delay(3000);
//  slider_servo.write(90); //rest
//  delay(1000);  
//
//  lifting_servo.write(0); //bring gripper down
//  delay(3000);
//  lifting_servo.write(90); //rest
//  delay(1000);


  haveItem = true;
}
void dropItem(){}
//void dropItem(){
//  slider_servo.write(??);
//  delay(?);
//  gripper_servo.write(??);
//  slider_servo.write(??);
//  delay(?);
//  haveItem = false;
//}

void uTurn(){
    myMotors.DeviceDriverSet_Motor_control(true, 150, false, 150, true); //two wheels in different directions
    delay(9000); //time to turn
    myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true); //stop
    delay(1000); // wait to be stable
    myMotors.DeviceDriverSet_Motor_control(true, 150, true, 150, true); //go straight
}

void stopEnd(){ //when blackline gone at the end, stop the robot
  if((analogRead(M_S) < 100)&&(analogRead(R_S) < 100)&&(analogRead(L_S) < 100)){
    myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true); //stop
    delay(100000); //long enough to power off manually 
  }
}

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
  
}


//test
void loop(){
  
  //ultrasonicVal = getUltrasonic();
//  encoderVal = (Rcounts+Lcounts)/2; //encoder value is the average of left and righr encoder
//  turnIRVal_L = digitalRead(ir_sensor_L);
//  turnIRVal_R = digitalRead(ir_sensor_R);
//  Serial.print(turnIRVal_L);
//  Serial.print(robotState);
//  Serial.print(itemNum);
//  Serial.print(" ");
//  Serial.println(turnIRVal_R);
//  lineTracking(130,150,170);
//  Serial.print(analogRead(L_S));
//  Serial.print(" ");
//  Serial.println(analogRead(R_S));
  
//  myMotors.DeviceDriverSet_Motor_control(true, 100, true, 100, true);
  delay(50);


//  int x = analogRead(pressurePin);
//  int x = digitalRead(pressurePin);
//  Serial.print(x);
    
//  IRremote.DeviceDriverSet_IRrecv_Get(&IRrecv_button); 
//  if(IRrecv_button > 6) // emergency stop button
//  if(robotState == startUp)
//  {
//      grabItem();
//      robotState = endStop;
//  }

//    lineTracking(130,150,170);
  
//  IRremote.DeviceDriverSet_IRrecv_Get(&IRrecv_button); 
//  if(IRrecv_button > 6) // emergency stop button
//  {
//    robotState = remoteControl;
//  }
//  
//  switch (robotState){
//    case(remoteControl):
//
//    
//      break;
//    case(startUp):
//        IRremote.DeviceDriverSet_IRrecv_Get(&IRrecv_button); 
//        switch(IRrecv_button) { //determine itemNum, turn1dis, turn1dir, turn2dis, lineLimit, lineLimitDropOff
//        // put   myMotors.DeviceDriverSet_Motor_Init();  inside each case to know if remote control works
//        case 1:
//          itemNum = 1;
//          lineLimitPickup = 1;
//          lineLimitDropOff = 0;
//          robotState = pickingUp;
//          turn1dir = true;
//          
//          break;
//        case 2:
//          itemNum = 2;
//          lineLimitPickup = 2;
//          lineLimitDropOff = 1;
//          robotState = pickingUp;
//          turn1dir = true;
//
//          break;
//        case 3:
//          itemNum = 3;
//          lineLimitPickup = 3;
//          lineLimitDropOff = 2;
//          robotState = pickingUp;
//          turn1dir = true;
//
//          break;
//        case 4:
//          itemNum = 4;
//          lineLimitPickup = 1;
//          lineLimitDropOff = 3;
//          robotState = pickingUp;
//          turn1dir = false;
//
//          break;
//        case 5:
//          itemNum = 5;
//          lineLimitPickup = 2;
//          lineLimitDropOff = 4;
//          robotState = pickingUp;
//          turn1dir = false;
//
//          break;
//        case 6:
//          itemNum = 6;
//          lineLimitPickup = 3;
//          lineLimitDropOff = 5;
//          robotState = pickingUp;
//          turn1dir = false;
//          break;
//        default:
//          break;
//        }
//      break;


  }
