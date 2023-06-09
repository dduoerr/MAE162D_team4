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

bool turn1dir; // UNUSED direction for the first turn: left or right
int turn2dis; //UNUSED where to make the second turn for dropping
int turnCount90 = 0; //USED count of turn 90 deg
bool haveItem = false; //USED if already grab the item

// variables for counting lines
int lineCount = 0;
int lineLimitPickup = 0;
int lineLimitDropOff = 0;
boolean lineDetected = false;

//// Create a new servo object:
//Servo ultrasonic_servo;
Servo slider_servo;
Servo lifting_servo;
//Servo gripper_servo;

// COUNTS FOR ENCODER COUNTS both right and left
long Rcounts = 0;
long Lcounts = 0;

//  //Initialize all variable with data type appears in the void loop
long ultrasonicVal, encoderVal;
int turnIRVal;

enum {pickingUp, moving90, moving45, navigateObstacles,dropppingOff, endStop };
unsigned char robotState = pickingUp;


//moving, turning45, pickingup, dropping off

//Functions

void lineTracking(int slow, int medium, int high){
 
  //go forward
  if((analogRead(R_S) > 100)&&(analogRead(L_S) > 100)){
    myMotors.DeviceDriverSet_Motor_control(true, medium, true, medium, true);
  }

  //accelerate right
  else if(analogRead(L_S) < 100){
    myMotors.DeviceDriverSet_Motor_control(true, slow, true, high, true);
  }

  //accelerate left
  else if(analogRead(R_S) < 100){
    myMotors.DeviceDriverSet_Motor_control(true, high, true, slow, true);
  }

}

void turn45(bool dir){ //only ever turns 45 degrees in the left direction, but left for robustness

    myMotors.DeviceDriverSet_Motor_control(true, 70, true, 70, true);
    delay(500);
    myMotors.DeviceDriverSet_Motor_control(dir, 70, !dir, 70, true);
    delay(400);
    myMotors.DeviceDriverSet_Motor_control(true, 70, true, 70, true);


}

void turn90(bool dir){ //UPDATE FOR NO WHITE CHECK
    
    //turn 90 degrees, where dir = True is turn left
    //if((analogRead(M_S) < 100)&&(analogRead(R_S) < 100)&&(analogRead(L_S) < 100)){ // seeing white
    if(digitalRead(ir_sensor_R)){ //ir sensor on the right see a black line
        myMotors.DeviceDriverSet_Motor_control(true, 70, true, 70, true);
        delay(500);
        myMotors.DeviceDriverSet_Motor_control(dir, 70, !dir, 70, true);
        delay(800);
        myMotors.DeviceDriverSet_Motor_control(true, 70, true, 70, true);
        turnCount90++;
    }
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

//void grabItem(){
//  //fill up angle later
//  lifting_servo.write(??);
//  slider_servo.write(??);
//  gripper_servo.write(??);//gripper fully open 
//  for (int angle = ?; angle> = 0; angle-=1){
//    if(analogRead(pressurePin) > 1000){
//      break;
//    }
//    else{
//      gripper_servo.write(angle);
//      delay(15);
//    }
//  } 
//  silder_servo.write(??);
//  lifting_servo.write(??);
//  haveItem = true;
//}

//void dropItem(){
//  slider_servo.write(??);
//  gripper_servo.write(??);
//  slider_servo.write(??);
//  haveItem = false;
//}

void uTurn(){
    myMotors.DeviceDriverSet_Motor_control(true, 70, false, 70, true); //two wheels in different directions
    delay(1600); //time to turn
    myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true); //stop
    delay(1000); // wait to be stable
    myMotors.DeviceDriverSet_Motor_control(true, 70, true, 70, true); //go straight
}

void stopEnd(){ //when blackline gone at the end, stop the robot
  if((analogRead(M_S) < 100)&&(analogRead(R_S) < 100)&&(analogRead(L_S) < 100)){
    myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true); //stop
    delay(100000); //long enough to power off manually 
  }
}

void readEncoder_R() //this function is triggered by the encoder CHANGE, and increments the encoder counter  //ENCODER BROKEN!
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
  IRremote.DeviceDriverSet_IRrecv_Init();
  myMotors.DeviceDriverSet_Motor_Init();
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
  //enable IR communication first 

//  ultrasonic_servo.attach(ultrasonicServoPin);
  slider_servo.attach(servoSlider);
  lifting_servo.attach(servoLift);
//  gripper_servo.attach(servoGripper);

  
}


void loop(){
  lifting_servo.write(0);
  delay(500);
  lifting_servo.write(90);
  delay(3000);



  
  //IRremote.DeviceDriverSet_IRrecv_Get(&IRrecv_button); 
//   lineTracking(50, 70, 100);
////    Serial.print(digitalRead(LH_ENCODER_A));
////    Serial.print(digitalRead(LH_ENCODER_B));
////    Serial.print(digitalRead(39));
////    Serial.print(digitalRead(32));
////    Serial.print(digitalRead(33));
//  Serial.print("lEncoder: ");
//  Serial.print(Lcounts);
//    Serial.print(" ");
//  Serial.print("rEncoder: ");
//  Serial.print(Rcounts);
//  Serial.print(" ");
//  delay(500);
//   Serial.print("LEncoder: ");
//  Serial.print(Rcounts);
//  encoderVal = (Rcounts+Lcounts)/2;
//  Serial.print("Encoder: ");
  //Serial.print(encoderVal);
//  Serial.print("IR: ");
//  Serial.print(analogRead(R_S));
//  Serial.print("itemNumber: ");
//  Serial.print(itemNum);
  //delay(2000);
}
//void loop() {
//  // put your main code here, to run repeatedly:
//
////  get IR sensor data, encoder data, ultrasonic data every loop
//  ultrasonicVal = getUltrasonic();
//  encoderVal = (Rcounts+Lcounts)/2; //encoder value is the average of left and righr encoder
//// !!!!NEED TO UPDATE turnIRVal = digitalRead(Turn_S);
//
//
//
//    switch (robotState){
//
//      case(pickingUp): //from start to picking up item
//        lineTracking(50, 70, 100);
//        if(lineDetected == False && turnIRVal == 1 && lineCount < lineLimitPickup) //detect first intersection
//        {
//          lineCount++;
//          lineDetected = True;
//        }
//        else if (lineDetected == True && turnIRVal == 0 && lineCount < lineLimitPickup) //no longer seeing intersections
//        {
//          lineDetected = False;
//        }
//
//        if(lineCount == lineLimitPickup) //reached intersection for picking up item
//        {
//           if (turnCount90 == 1 && !haveItem){ //robot allready make the turn and do not have the item, and go straight to item
//                lineTracking(50, 70, 100);
//                if (ultrasonicVal < ~0){ //robot is approaching the wall
//                    myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true); //stop
//                    delay(1000); // wait to be stable
//                    grabItem();
//                    uTurn();
//                    lineCount = 0;
//                    turnCount90 = 0;
//                    robotState = moving90;
//                }
//          else {
//            turn90(True); //change boolean based on juice box position
//            turnCount90++;
//          }
//        }
//        else{
//           delay(300); //give time to either pass intersection or turn
//        }
//
//      case(moving90): //from after picking up item to completing 90 degree turn
//        lineTracking(50, 70, 100);
//        if(turnIRVal == 1) { //not sure if 1 is black or white
//
//          turn90(dir); //DETERMINE DIRECTION BASED ON ITEM NUMBER
//          robotState = moving45;
//        }
//
//      case(moving45): //from straight path to completing 45 degree turn
//         lineTracking(50, 70, 100);
//        if(turnIRVal == 1) {
//          turn45(True);
//          delay(1000);
//          lineTracking(50,70,100);
//          delay(1000);
//          turn45(True);
//          robotState = navigateObstacles;
//        }
//    
//
//      case(navigateObstacles): //for bumpy road and dynamic obstacle
//        
//        lineTracking(50, 70, 100); 
//        int directions[] = {45, 135};
//        for(int idx = 0; i < 2; i++)
//        {
//          ultrasonic_servo.write(directions[idx]); //look left then right
//
//          //use ultrasonic data to detect if object is not in front of the vehicle;
//          delay(15);
//          ultrasonicValPrev = getUltrasonic();
//          delay(50);
//          ultrasonicValNow = getUltrasonic();
//          rateOfChange = ultrasonicValNow - ultrasonicValPrev;
//          if(ultrasonicValNow < 10 && rateOfChange < 0) { //actually detecting obstacle and not the wall/something else
//            //obstacles is moving away, so speed past the object
//            lineTracking(50, 70, 100);
//          }
//          else {//dynamic obstacle is getting closer. slow down
//            lineTracking(20, 30, 40); 
//          }
//          delay(100);
//
//        }
//        
//        //go slow in bumpy region
//        lineTracking(30, 40, 50);
//
//        if(lineDetected == False && turnIRVal == 1) { //detect first intersection, transition to drop off mode
//          robotState = droppingOff;
//          lineCount = 0;
//        }
//        
//      
//      case(droppingOff): 
//
//          
//          if (itemNum != 1){ // item 1 is the special case, can go straight up to dropping point without turning
//            lineTracking(50,70,100);
//          else{
//            lineTracking(50,70,100);
//            turn90(false); //turn left
//          }
//
//          if(lineDetected == False && turnIRVal == 1 && lineCount < lineLimitPickup) //detect first intersection
//          {
//            lineCount++;
//            lineDetected = True;
//          }
//          else if (lineDetected == True && turnIRVal == 0 && lineCount < lineLimitPickup) //no longer seeing intersections
//          {
//            lineDetected = False;
//          }
//
//          if(lineCount == lineLimitDropOff) //reached intersection for dropping off item
//          {
//            if (ultrasonicVal < ~0){ //robot is approaching the wall
//              myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true); //stop
//              delay(1000); // wait to be stable
//              dropItem();
//              uTurn();
//              robotState = endStop;
//            }
//            else{
//                lineTracking(50, 70, 100);
//                turn90(true); //always turn right
//            }
//            
//          }
//      case(endStop):
//        lineTracking(50,70,100);
//        stopEnd();
//        
//      }
//
//
//
//  }
//
//}
