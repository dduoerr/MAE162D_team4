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
//  lifting_servo.write(??); // 0 and 180 mean 2 directions.
//  delay(?);
//  slider_servo.write(??); 
//  delay(?);

    gripper_servo.write(90);
    delay(1000);
//    gripper_servo.write(180);
    delay(1000);
    gripper_servo.write(0);
    delay(1000);
//  gripper_servo.write(90);//gripper fully open 
  for (int angle = 0; angle <= 90; angle+=1){
//    if(analogRead(pressurePin) > 1000){
//      break;
//    }
//    else{
//      gripper_servo.write(angle);
//      delay(15);
//    }
//  } 
//  delay(5000);
//  gripper_servo.write(0);
  delay(15);
////  silder_servo.write(??);
//  delay(?);
//  lifting_servo.write(??);
//  delay(?)
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

  ultrasonic_servo.attach(ultrasonicServoPin);
  slider_servo.attach(servoSlider);
  lifting_servo.attach(servoLift);
  gripper_servo.attach(servoGripper);
  robotState = startUp;
  
}


//test
void loop(){
  
  //ultrasonicVal = getUltrasonic();
  encoderVal = (Rcounts+Lcounts)/2; //encoder value is the average of left and righr encoder
  turnIRVal_L = digitalRead(ir_sensor_L);
  turnIRVal_R = digitalRead(ir_sensor_R);
  Serial.print(turnIRVal_L);
//  Serial.print(robotState);
//  Serial.print(itemNum);
  Serial.print(" ");
  Serial.println(turnIRVal_R);
  delay(500);

    
//  IRremote.DeviceDriverSet_IRrecv_Get(&IRrecv_button); 
//  if(IRrecv_button > 6) // emergency stop button
  if(robotState == startUp)
  {
      grabItem();
      robotState = endStop;
  }

  
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
//    case(pickingUp): //from start to picking up item
//      lineTracking(130, 150, 170);
//      if(lineDetected == false && turnIRVal_L == 1 && lineCount < lineLimitPickup) //detect first intersection
//      {
//        lineCount++;
//        lineDetected = true;
//      }
//      else if (lineDetected == true && turnIRVal_L == 0 && lineCount < lineLimitPickup) //no longer seeing intersections
//      {
//        lineDetected = false;
//      }
//
//      if(lineCount == lineLimitPickup) //reached intersection for picking up item
//      {
//        if (turnCount90 == 1 && !haveItem){ //robot allready make the turn and do not have the item, and go straight to item
//
//          lineTracking(50, 70, 100);
//          if (ultrasonicVal < 20){ //robot is x distance away from wall
//              myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true); //stop
//              delay(1000); // wait to be stable
//              grabItem();
//              uTurn();
//              lineCount = 0;
//              turnCount90 = 0;
//              robotState = moving90;
//        }
//        else {
//          turn90(turn1dir); //change boolean based on juice box position
//        }
//      }
//      else{
//          delay(1300); //give time to either pass intersection or turn.
//      }
//      break;
//    case(moving90): //from after picking up item to completing 90 degree turn
//      lineTracking(50, 70, 100);
//      if(turnIRVal_L == 1) { //not sure if 1 is black or white
//
//        turn90(turn1dir); //DETERMINE DIRECTION BASED ON ITEM NUMBER
//        robotState = moving45;
//      }
//      break;
//    case(moving45): //from straight path to completing 45 degree turn
//      lineTracking(130, 140, 150);
//
//
//      if((turnIRVal_L == 1) && (turnIRVal_R == 0)) {
//        turn45(true); // navigate turn45 in one go
//        for(int i = 0; i < 100; i++) // 10 seconds to adjust
//        {
//          lineTracking(130,150,170);
//          delay(100);
//        }
//        turn45(true);
//  
//  
//        
//        robotState = navigateObstacles;
//      }
//  
//      break;
//    case(navigateObstacles): //for bumpy road and dynamic obstacle
//      
//      lineTracking(50, 70, 100); 
//      int directions[] = {45, 135};
////      for(int idx = 0; idx < 2; idx++)
////      {
////        ultrasonic_servo.write(directions[idx]); //look left then right
////
////        //use ultrasonic data to detect if object is not in front of the vehicle;
////        delay(15);
////        int ultrasonicValPrev = getUltrasonic();
////        delay(50);
////        int ultrasonicValNow = getUltrasonic();
////        int rateOfChange = ultrasonicValNow - ultrasonicValPrev;
////        if(ultrasonicValNow < 10 && rateOfChange < 0) { //actually detecting obstacle and not the wall/something else
////          //obstacles is moving away, so speed past the object
////          lineTracking(50, 70, 100);
////        }
////        else {//dynamic obstacle is getting closer. slow down
////          lineTracking(20, 30, 40); 
////        }
////        delay(100);
////
////      }
//      
//      //go slow in bumpy region
////      lineTracking(130, 150, 170);
//    
//      if(lineDetected == false && turnIRVal_L == 1) { //detect first intersection, transition to drop off mode
//        robotState = droppingOff;
//        lineCount = 0;
//      }
//      
//      break;
//    case(droppingOff): 
//
//        
//      if (itemNum != 1){ // item 1 is the special case, can go straight up to dropping point without turning
//        lineTracking(130,150,170);
//      }
//      else{
//        lineTracking(130,150,170);
//        turn90(false); //turn left
//      }
//
//      if((lineDetected == false) && (turnIRVal_R == 1)&& (lineCount < lineLimitPickup)) //detect first intersection
//      {
//        lineCount++;
//        lineDetected = true;
//      }
//      else if ((lineDetected == true) && (turnIRVal_R == 0) && (lineCount < lineLimitPickup)) //no longer seeing intersections
//      {
//        lineDetected = false;
//      }
//
//      if(lineCount == lineLimitDropOff) //reached intersection for dropping off item
//      {
//        lineTracking(50,70,100); //slow down, we're approaching drop off
//
//        if (( lineLimitDropOff>=0) && (turnCount90 == 3)){
//            lineTracking(50, 70, 100);
//            turn90(false); //always turn right
//        }
//        else if((analogRead(R_S) < 100)&& (analogRead(R_S)< 100) && (analogRead(M_S)<100))// no longer seeing black line
//        {
//            myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true); //stop
//            delay(1000); // wait to be stable
//            dropItem();
//            uTurn();
//            robotState = endStop;
//        }
//
//  
//        
//      }
//      break;
//    case(endStop):
//      lineTracking(130,150,170);
//      stopEnd();
//      break;
//    default:
//      break;  
//    }    
//   
//
//        


  }


  
//  turnIRVal = digitalRead(ir_sensor_L); 
//  lineTracking(130, 150, 170);
//
////  Serial.print(turnIRVal);
////  delay(500);
//      if(turnIRVal == 1) { //not sure if 1 is black or white
//
//        turn90(true); //DETERMINE DIRECTION BASED ON ITEM NUMBER
//        delay(5000);
//        uTurn();
//      }








//void loop() {
//  // put your main code here, to run repeatedly:
//
////  get IR sensor data, encoder data, ultrasonic data every loop
//  ultrasonicVal = getUltrasonic();
//  encoderVal = (Rcounts+Lcounts)/2; //encoder value is the average of left and righr encoder
////  turnIRVal = digitalRead(ir_sensor_L);
//
//
//
//  switch (robotState){
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
//
//    case(pickingUp): //from start to picking up item
//      lineTracking(50, 70, 100);
//      if(lineDetected == false && turnIRVal == 1 && lineCount < lineLimitPickup) //detect first intersection
//      {
//        lineCount++;
//        lineDetected = true;
//      }
//      else if (lineDetected == true && turnIRVal == 0 && lineCount < lineLimitPickup) //no longer seeing intersections
//      {
//        lineDetected = false;
//      }
//
//      if(lineCount == lineLimitPickup) //reached intersection for picking up item
//      {
//        if (turnCount90 == 1 && !haveItem){ //robot allready make the turn and do not have the item, and go straight to item
//
//          lineTracking(50, 70, 100);
//          if (ultrasonicVal < ~0){ //robot is approaching the wall
//              myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true); //stop
//              delay(1000); // wait to be stable
//              grabItem();
//              uTurn();
//              lineCount = 0;
//              turnCount90 = 0;
//              robotState = moving90;
//          }
//        else {
//          turn90(turn1dir); //change boolean based on juice box position
//          turnCount90++;
//        }
//      }
//      else{
//          delay(500); //give time to either pass intersection or turn. needs be tuned
//      }
//
//    case(moving90): //from after picking up item to completing 90 degree turn
//      lineTracking(50, 70, 100);
//      if(turnIRVal == 1) { //not sure if 1 is black or white
//
//        turn90(turn1dir); //DETERMINE DIRECTION BASED ON ITEM NUMBER
//        robotState = moving45;
//      }
//
//    case(moving45): //from straight path to completing 45 degree turn
//        lineTracking(50, 70, 100);
//      if(turnIRVal == 1) {
//        turn45(true);
//        delay(1000);
//        lineTracking(130,150,170);;
//        delay(1000);
//        turn45(true);
//        robotState = navigateObstacles;
//      }
//  
//
//    case(navigateObstacles): //for bumpy road and dynamic obstacle
//      
//      lineTracking(50, 70, 100); 
//      int directions[] = {45, 135};
//      for(int idx = 0; idx < 2; idx++)
//      {
//        ultrasonic_servo.write(directions[idx]); //look left then right
//
//        //use ultrasonic data to detect if object is not in front of the vehicle;
//        delay(15);
//        int ultrasonicValPrev = getUltrasonic();
//        delay(50);
//        int ultrasonicValNow = getUltrasonic();
//        int rateOfChange = ultrasonicValNow - ultrasonicValPrev;
//        if(ultrasonicValNow < 10 && rateOfChange < 0) { //actually detecting obstacle and not the wall/something else
//          //obstacles is moving away, so speed past the object
//          lineTracking(50, 70, 100);
//        }
//        else {//dynamic obstacle is getting closer. slow down
//          lineTracking(20, 30, 40); 
//        }
//        delay(100);
//
//      }
//      
//      //go slow in bumpy region
//      // lineTracking(30, 40, 50);
//
//      if(lineDetected == false && turnIRVal == 1) { //detect first intersection, transition to drop off mode
//        robotState = droppingOff;
//        lineCount = 0;
//      }
//      
//    
//    case(droppingOff): 
//
//        
//        if (itemNum != 1){ // item 1 is the special case, can go straight up to dropping point without turning
//          lineTracking(130,150,170);;
//        }
//        else{
//          lineTracking(130,150,170);;
//          turn90(false); //turn left
//        }
//
//        if(lineDetected == false && turnIRVal == 1 && lineCount < lineLimitPickup) //detect first intersection
//        {
//          lineCount++;
//          lineDetected = true;
//        }
//        else if (lineDetected == true && turnIRVal == 0 && lineCount < lineLimitPickup) //no longer seeing intersections
//        {
//          lineDetected = false;
//        }
//
//        if(lineCount == lineLimitDropOff) //reached intersection for dropping off item
//        {
//          if (ultrasonicVal < ~0){ //robot is approaching the wall
//            myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true); //stop
//            delay(1000); // wait to be stable
//            dropItem();
//            uTurn();
//            robotState = endStop;
//          }
//          else{
//              lineTracking(50, 70, 100);
//              turn90(false); //always turn right
//          }
//          
//        }
//    case(endStop):
//      lineTracking(130,150,170);;
//      stopEnd();
//      
//    }
//
//        
//
//
//  }
//
//}
