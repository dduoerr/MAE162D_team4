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

  //myMotors.DeviceDriverSet_Motor_control(true, 70, true, 70, true);
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

}
