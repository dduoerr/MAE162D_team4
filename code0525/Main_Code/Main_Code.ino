
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
int count = 0;
long t_0 = 0;
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
long encoderVal;
int turnIRVal_L,turnIRVal_R;

enum {startUp, pickingUp, moving90, navigateObstacles,droppingOff, endStop, remoteControl, finished, resetServo};
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


void turnDegree(bool dir, int degree){ //turn designated number of degrees

    Serial.println("turning " + degree);
//    Rcounts = 0;
//    Lcounts = 0;
    long threshold = 0;
    int lineCount = 0; //number of times we see white
    if(degree == 90)
    {
      threshold = 3350;
      turnCount90++;
      lineCount = 1;
    }
    else if(degree == 180)
    {
      threshold = 6600;
      lineCount = 1;
    }

    bool seeWhite = false;
    myMotors.DeviceDriverSet_Motor_control(dir, 150, !dir, 150, true);
    for(int i = 0; i < lineCount;) //assume we start on black line
    {
      if(!seeWhite && (analogRead(R_S) < 100)&&(analogRead(L_S) < 100) && (analogRead(M_S) < 100)) // now seeing all white line 
      { 
        seeWhite = true;
      }
      else if ( seeWhite && (analogRead(R_S) > 100)&&(analogRead(L_S) > 100) && (analogRead(M_S) > 100)) // now seeing all black lines
      {
        i++;
        seeWhite = false;
      }
      
    }
    delay(700);
    myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true); //stop
    delay(1000); // wait to be stable
        
//    delay(threshold);
      
//    if(dir) // turn left
//    {
//      long start_R = Rcounts;
//      while(Rcounts - start_R < threshold){
//        Serial.println("Rcounts: " + Rcounts);
//        myMotors.DeviceDriverSet_Motor_control(dir, 150, !dir, 150, true);
//      }
//    }
//    else{ // turn right
//      long start_L = Lcounts;
//      while(Lcounts - start_L < threshold){
//        Serial.println("Lcounts: " + Lcounts);
//        myMotors.DeviceDriverSet_Motor_control(dir, 150, !dir, 150, true);
//      }
//    }
//    Serial.println("finished turning");
}

double getUltrasonic(){
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
   return duration / 29.0 / 2;
}


void grabItem(){
  //fill up angle later

  //steps: drive slider servo forward, drive lifter servo up, grab item, then retract slider servo

  slider_servo.write(180); //drive mechanism forward
  delay(4500);
  slider_servo.write(90); //rest
  delay(1000);

  gripper_servo.write(180); //fully open
  delay(1000);
  
  
  // progressively close
  for (int angle = 180; angle >= 0; angle--){
    if(analogRead(pressurePin) > 1000){
      break;
    }
    else{
      gripper_servo.write(angle);
      delay(15);
    }
  } 
  delay(2000);

  turnDegree(true,180);

  lifting_servo.write(45);
  delay(500);
  lifting_servo.write(90);

  haveItem = true;
}
void dropItem(){
  lifting_servo.write(0); //lifter down
  delay(8000);
  lifting_servo.write(90); //rest
  delay(1000);
  for(int i = gripper_servo.read(); i < 180; i++)
  {
    gripper_servo.write(i);
    delay(15);
  }

  myMotors.DeviceDriverSet_Motor_control(false,100, false, 100, true); //back up
  turnDegree(true,180);
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

  slider_servo.attach(servoSlider);
  lifting_servo.attach(servoLift);
  gripper_servo.attach(servoGripper);
  
  robotState = startUp;
  
  gripper_servo.write(180); //fully open
  delay(1000);
}

//test
void loop(){
  
  encoderVal = (Rcounts+Lcounts)/2; //encoder value is the average of left and righr encoder

    
//  IRremote.DeviceDriverSet_IRrecv_Get(&IRrecv_button); 
//  if(IRrecv_button > 6) // emergency stop button
//  if(robotState == startUp)
//  {
//      grabItem();
//      robotState = endStop;
//  }

  
  IRremote.DeviceDriverSet_IRrecv_Get(&IRrecv_button); 
  if(IRrecv_button == 15) // emergency stop button, press "ok"
  {
    robotState = finished;
  }
  else if(IRrecv_button == 11) //start over, press "up"
  {
    robotState = startUp;
  }
  else if(IRrecv_button == 12) //test from desired state, press "down"
  {
    robotState = remoteControl;
  }
  else if(IRrecv_button > 12 && IRrecv_button < 15) // any other button, tell device to reset
  {
    robotState = resetServo;
  }
  
  switch (robotState){
    case(remoteControl):

      robotState = navigateObstacles;
      myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true); //stop
      delay(1000); // wait to be stable
      break;
      
    case(startUp):
        IRremote.DeviceDriverSet_IRrecv_Get(&IRrecv_button); 
        switch(IRrecv_button) { //determine itemNum, turn1dis, turn1dir, turn2dis, lineLimit, lineLimitDropOff
          case 1:
            itemNum = 1;
            lineLimitPickup = 1;
            lineLimitDropOff = 0;
            robotState = pickingUp;
            turn1dir = true;
            break;
          case 2:
            itemNum = 2;
            lineLimitPickup = 2;
            lineLimitDropOff = 1;
            robotState = pickingUp;
            turn1dir = true;
            break;
          case 3:
            itemNum = 3;
            lineLimitPickup = 3;
            lineLimitDropOff = 2;
            robotState = pickingUp;
            turn1dir = true;
  
            break;
          case 4:
            itemNum = 4;
            lineLimitPickup = 1;
            lineLimitDropOff = 3;
            robotState = pickingUp;
            turn1dir = false;
  
            break;
          case 5:
            itemNum = 5;
            lineLimitPickup = 2;
            lineLimitDropOff = 4;
            robotState = pickingUp;
            turn1dir = false;
  
            break;
          case 6:
            itemNum = 6;
            lineLimitPickup = 3;
            lineLimitDropOff = 5;
            robotState = pickingUp;
            turn1dir = false;
            break;
          default:
            break;
        }
      break;
    case(pickingUp): //from start to picking up item
      Serial.println("asdf");
//      turnIRVal_L = digitalRead(ir_sensor_L);
//      turnIRVal_R = digitalRead(ir_sensor_R);
//      Serial.print(turnIRVal_L + " " + turnIRVal_R);
      if(digitalRead(ir_sensor_L))
      {
        Serial.println("detecting");
      }
      lineTracking(130, 150, 170);
      if((lineDetected == false) && digitalRead(ir_sensor_L) && (lineCount < lineLimitPickup)) //detect first intersection
      {
        lineCount++;
        lineDetected = true;
      }
      else if ((lineDetected == true) && (!digitalRead(ir_sensor_L)) && (lineCount < lineLimitPickup)) //no longer seeing intersections
      {
        lineDetected = false;
      }

      if(lineCount == lineLimitPickup) //reached intersection for picking up item
      {

        if (turnCount90 == 1 && !haveItem){ //robot allready make the turn and do not have the item, and go straight to item]

          lineTracking(50, 70, 90);
          double ultrasonicVal = getUltrasonic();
//          ultrasonicVal--;
          Serial.println(ultrasonicVal);  
          if (ultrasonicVal > 0.5 && ultrasonicVal < 2){ //robot is x distance away from wall
//              lineTracking(40, 60, 80);
              myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true); //stop
              delay(3000); // wait to be stable
              grabItem();
              lineCount = 0;
              turnCount90 = 0;
              lineDetected = false;
              robotState = moving90;
          }
        }
        else {
          Serial.println("turning");
//          delay(1000);
          turnDegree(turn1dir, 90); //change boolean based on juice box position
        }
      }
      else{
          delay(1300); //give time to either pass intersection or turn.
      }
      break;
    case(moving90): //from after picking up item to completing 90 degree turn
      lineTracking(130, 150, 170);
      if( digitalRead(ir_sensor_L) && digitalRead(ir_sensor_R)) {
        delay(800);
        turnDegree(turn1dir, 90); //DETERMINE DIRECTION BASED ON ITEM NUMBER
        
        robotState = navigateObstacles;
        t_0 = millis();
      }
      break;
    case(navigateObstacles): //for bumpy road and dynamic obstacle
    {
        
      //use ultrasonic data to detect if object is not in front of the vehicle;
      double ultrasonicVal = getUltrasonic();
     

      
      if(ultrasonicVal > 10) { //actually detecting obstacle and not the wall/something else
        //obstacles is moving away, so speed past the object
        lineTracking(10, 70, 150);
      }
      else {//dynamic obstacle is present. Stop.
           myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true);
      }

      if((millis() - t_0 > 60000) && digitalRead(ir_sensor_L) && digitalRead(ir_sensor_R)) { //detect first intersection, transition to drop off mode
        robotState = droppingOff;
        lineCount = 0;
        if (itemNum == 1){ // item 1 is the special case, can go straight up to dropping point without turning
          lineTracking(130,150,170);
          delay(500);
        }
        else{
          lineTracking(130,150,170);
          turnDegree(true, 90); //turn left
          delay(500);
          lineTracking(130,150,170);
        }
      }
      
      break;
    }
    case(droppingOff): 

    
      lineTracking(130, 150, 170);
      
      if((lineDetected == false) && (digitalRead(ir_sensor_R))&& (lineCount < lineLimitDropOff)) //detect first intersection
      {
        lineCount++;
        lineDetected = true;
      }
      else if ((lineDetected == true) && (!digitalRead(ir_sensor_R)) && (lineCount < lineLimitDropOff)) //no longer seeing intersections
      {
        lineDetected = false;
      }

      if(lineCount == lineLimitDropOff) //reached intersection for dropping off item
      {
        lineTracking(50,70,100); //slow down, we're approaching drop off

        if (( lineLimitDropOff>0)){
            lineTracking(50, 70, 100); //move one more time
            turnDegree(false, 90); //always turn right
            lineTracking(50, 70, 100); //move one more time
            myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true); //stop
            delay(1000); // wait to be stable
            dropItem();
            robotState = endStop;
        }
        else{ //special case for item 1
          lineTracking(50, 70, 100); //move one more time
          myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true); //stop
          delay(1000); // wait to be stable
          dropItem();
          robotState = endStop;
        }
      }
      else{
        delay(1000);
      }
      break;
      
    case(endStop): // from after dropping off item to end of course
    {
      lineTracking(130,150,170);
      if(digitalRead(ir_sensor_L) && digitalRead(ir_sensor_R)) //sees intersection
      {
        turnDegree(false, 90); //always turn right
      }
      double ultrasonicVal = getUltrasonic();
      if(ultrasonicVal < 5) //close to wall
        robotState = finished;
      
      break;
    }
    case(finished): //completed state
      myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true); //stop
      break;
    case(resetServo):
      slider_servo.write(0); //drive mechanism back
      delay(2500);
      slider_servo.write(90); //rest
      delay(1000);
      
      gripper_servo.write(180); //fully open
      delay(500);
      
      lifting_servo.write(180); //drive mechanism up
      delay(3000);
      lifting_servo.write(90); //rest
      delay(1000);

    default:
      break;  
  }   

}

//      
//    case(moving45): //from straight path to completing 45 degree turn
//      lineTracking(130, 150, 170);
////
////      if((digitalRead(ir_sensor_L)) && (!digitalRead(ir_sensor_R))) {
////        myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true); //stop
////        delay(1000); // wait to be stable
//           Serial.println("About to turn");
////        turnDegree(true, 45); // navigate turn45 in one go
//////        turn45(true);
//// 
////      }
////      else{
////        delay(200);
////      }
////
////      if(turnCount45 >=2){
////         robotState = navigateObstacles;
////      }
//      
//      break;
