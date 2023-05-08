#include <avr/wdt.h>
#include <stdio.h>
#include <string.h>
#include "DeviceDriverSet_xxx0.h"

DeviceDriverSet_Motor myMotors; 
DeviceDriverSet_IRrecv IRremote;

uint8_t IRrecv_button; 

int itemNum; // which item to pick up
int turn1dis; // where to make first turn (encoder distance value) for picking up

bool turn1dir; // direction for the first turn: left or right
int turn2dis; //where to make the second turn for dropping
int turnCount90 = 0; //count of turn 90 deg
bool haveItem = false; //if already grab the item

// variables for counting lines
int lineCount = 0;
int lineLimitPickup = 0;
int lineLimitDropOff = 0;
boolean lineDetected = false;

enum {pickingUp, moving90, moving45, navigateObstacles,dropppingOff };
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

void turn90(bool dir){
    
    //turn 90 degrees, where dir = True is turn right
    if((analogRead(M_S) < 100)&&(analogRead(R_S) < 100)&&(analogRead(L_S) < 100)){ // seeing white
        myMotors.DeviceDriverSet_Motor_control(true, 70, true, 70, true);
        delay(500);
        myMotors.DeviceDriverSet_Motor_control(dir, 70, !dir, 70, true);
        delay(800);
        myMotors.DeviceDriverSet_Motor_control(true, 70, true, 70, true);
        turnCount90++;
    }
}

int getUltrasonic(){
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
  /*
}
  lifting servo move;//up
  slider servo move;//forward
  gripper servo move;
        using while loop, while (servo angle < ?), read pressure sensor, if reading < ?, break(stop closing);
    slider servo move;//backward
  lifting servo move;//down
  haveItem = true;
  */
}

void dropItem(){
  slider servo move;//forward
  gripper servo move(release);
    slider servo move;//backward
  haveItem = false;
}

void uTurn(){
    myMotors.DeviceDriverSet_Motor_control(true, 70, false, 70, true); //two wheels in different directions
    delay(1600); //time to turn
    myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true); //stop
    delay(1000); // wait to be stable
    myMotors.DeviceDriverSet_Motor_control(true, 70, true, 70, true); //go straight
}

void endStop(){ //when blackline gone at the end, stop the robot
  if((analogRead(M_S) < 100)&&(analogRead(R_S) < 100)&&(analogRead(L_S) < 100)){
    myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true); //stop
    delay(100000); //long enough to power off manually 
  }
}


void setup(){
  // put your setup code here, to run once:
  Serial.begin(9600); 
  //enable IR communication first 
  IRremote.DeviceDriverSet_IRrecv_Init();

  switch(IRrecv_button) { //determine itemNum, turn1dis, turn1dir, turn2dis, lineLimit, lineLimitDropOff
  // put   myMotors.DeviceDriverSet_Motor_Init();  inside each case to know if remote control works
  case1:
  case2:
  case3:
  case4:
  case5:
  case6:
  
  }
}

void loop() {
  // put your main code here, to run repeatedly:

//  get IR sensor data, encoder data, ultrasonic data every loop
    
//  ultrasonicVal = getUltrasonic();
//  encoderVal = digitalRead(encoder);
//  turnIRVal = digitalRead(Turn_S);
    switch (robotState): {

      case(pickingUp):
        if(lineDetected == False && turnIRVal == 1) //detect first intersection
        {
          lineCount++;
          lineDetected = True;
        }
        else if (lineDetected == True && turnIRVal == 0) //no longer seeing intersections
        {
          lineDetected = False;
        }

        if(!haveItem && lineCount == lineLimitPickup) //reached intersection for picking up item
        {
           if (turnCount90 == 1 && !haveItem){ //robot allready make the turn and do not have the item, and go straight to item
                lineTracking(50, 70, 100);
                if (ultrasonicVal < ~0){ //robot is approaching the wall
                    myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true); //stop
                    delay(1000); // wait to be stable
                    grabItem();
                    lineCount = 0;
                    robotState = moving;
                }
        }

      case(moving):

      case(navigateObstacles):
        
        //use ultrasonic data to 
        

        //TO DO:
        //stop when we see obstacle, then move forward once it is out of view
            //use servo angle and ultrasonic to determine when the obstacle is not in the way of the forward direction
        
        //go slow in bumpy region
        lineTracking(30, 40, 50);

        if(lineDetected == False && turnIRVal == 1) //detect first intersection, transition to drop off mode
        {
          robotState = droppingOff;
//          lineCount++;
//          lineDetected = True;
        }
        
      
      case(droppingOff): 
        
        if(haveItem && lineCount == lineLimitDropOff) //reached intersection for dropping off item
        {

          
        }

        // if (itemNum == 1){ // item 1 is the special case, can go straight up to dropping point without turning
        //   if (turnCount90 == 2 && haveItem){ //only turn 2 times before
        //     lineTracking(50, 70, 100);
        //     if (ultrasonicVal < ~0){ //robot is approaching the wall
        //       myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true); //stop
        //       delay(1000); // wait to be stable
        //       dropItem();
        //       uTurn();
        //     }
        //   }
        //   else{
        //       lineTracking(50, 70, 100);
        //       turn90(true);
        //   }
        // else{ //item 2,3,4,5,6
        //   if (turnCount90 == 4 && haveItem){ //turn 4 times before
        //     lineTracking(50, 70, 100);
        //     if (ultrasonic < ~0){ //robot is approaching the wall
        //       myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true); //stop
        //       delay(1000); // wait to be stable
        //       dropItem();
        //       uTurn();
        //     }
        //   }
        //   else{
        //     lineTracking(50, 70, 100);
        //     turn90(true);
        //   }
        // }
      }
  


    
    if (encoder in a range of turn1dis){//for first 90turn and picking up
       
        uTurn();
      }
    }
    else{
      lineTracking();
      turn90(turn1dir);
    }
  }
  else if(encoder in a range of two 45deg turn){
    //not sure whether lineTracking is enough, otherwise, add a turn45() function
    lineTracking();
  }
  else if(encoder in the range of 90deg turn after rough terrain){
    if (itemNum != 1){ // item 1 is the special case, can go straight up to dropping point without turning
      lineTracking();
    else{
      lineTracking();
      turn90(false); //turn left
    }
 // else if(encoder in a range of turn2dis){//for 90turn and dropping off

  }
  else if(encoder in the range of stop at the end){
    lineTracking();
    endStop();
  }
  else{ // go straight
    lineTracking();
  }
}
