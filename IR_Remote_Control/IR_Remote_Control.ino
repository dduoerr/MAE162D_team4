#include <avr/wdt.h>
#include <stdio.h>
#include <string.h>
#include "DeviceDriverSet_xxx0.h"

DeviceDriverSet_Motor myMotors; 
DeviceDriverSet_IRrecv IRremote;

uint8_t IRrecv_button; 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  IRremote.DeviceDriverSet_IRrecv_Init();
  myMotors.DeviceDriverSet_Motor_Init(); 
  pinMode(L_S, INPUT);
  pinMode(M_S, INPUT);
  pinMode(R_S, INPUT);
}

void loop() {

  //line tracking and turn 90 degree
  if((analogRead(R_S) > 100)&&(analogRead(L_S) > 100)){myMotors.DeviceDriverSet_Motor_control(true, 70, true, 70, true);}

  else if(analogRead(L_S) < 100){
    myMotors.DeviceDriverSet_Motor_control(true, 50, true, 100, true);
    
    }

  else if(analogRead(R_S) < 100){myMotors.DeviceDriverSet_Motor_control(true, 100, true, 50, true);}
  
  if((analogRead(M_S) < 100)&&(analogRead(R_S) < 100)&&(analogRead(L_S) < 100)){
    myMotors.DeviceDriverSet_Motor_control(true, 70, true, 70, true);
    delay(500);
    myMotors.DeviceDriverSet_Motor_control(true, 70, false, 70, true);
    delay(800);
    myMotors.DeviceDriverSet_Motor_control(true, 70, true, 70, true);
  }

  //remote control to go forward/back/left/right
  IRremote.DeviceDriverSet_IRrecv_Get(&IRrecv_button); 
  switch(IRrecv_button) 
  {
    case 1: // Forward
      myMotors.DeviceDriverSet_Motor_control(true, 70, true, 70, true);
      break; 
    case 2: // Backward
      myMotors.DeviceDriverSet_Motor_control(false, 70, false, 70, true);
      break;
    case 3: // Left
      myMotors.DeviceDriverSet_Motor_control(true, 70, false, 70, true);
      break;  
    case 4: // Right
      myMotors.DeviceDriverSet_Motor_control(false, 70, true, 70, true);
      break;  
    case 5: // Stop
      myMotors.DeviceDriverSet_Motor_control(3, 0, 3, 0, true);
      break;  
    
    default: 
      Serial.println("Nothing"); 
      break; 
    }

  //Servo sensor data
}
