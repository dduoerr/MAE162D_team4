#include <Servo.h>
#include <avr/wdt.h>
#include <stdio.h>
#include <string.h>
#include "DeviceDriverSet_xxx0.h"

DeviceDriverSet_Motor myMotors; 
DeviceDriverSet_IRrecv IRremote;

#define sensorPin A0
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  IRremote.DeviceDriverSet_IRrecv_Init();
  myMotors.DeviceDriverSet_Motor_Init(); 

}

void loop() {
 int x = analogRead(sensorPin);
 Serial.println(x);
 delay(50);
}
