#include <Servo.h>
#include <avr/wdt.h>
#include <stdio.h>
#include <string.h>
#include "DeviceDriverSet_xxx0.h"

DeviceDriverSet_Motor myMotors; 
DeviceDriverSet_IRrecv IRremote;

Servo myservo;  // create servo object to control a servo

int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin

uint8_t IRrecv_button; 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  IRremote.DeviceDriverSet_IRrecv_Init();
  myMotors.DeviceDriverSet_Motor_Init(); 
  
  myservo.attach(9); // attaches the servo on pin 9 to the servo object
}


void loop() {
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  Serial.println(val);
  //val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  //myservo.write(val);                  // sets the servo position according to the scaled value
  delay(15);                           // waits for the servo to get there
}
