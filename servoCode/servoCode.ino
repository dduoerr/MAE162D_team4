/* Servo motor with Arduino example code. Position and sweep. More info: https://www.makerguides.com/ */

// Include the servo library:
#include <Servo.h>

// Create a new servo object:
Servo myservo;

// Define the servo pin:
#define servoPin 46

// Create a variable to store the servo position:
int angle = 0;

void setup() {
  // Attach the Servo variable to a pin:
  myservo.attach(servoPin);

  //reset position
//  for (angle = 0; angle <= 90; angle += 1) {
//    myservo.write(angle);
//    delay(15);
//  }
}

void loop() {
  // Tell the servo to go to a particular angle:

//  short the control pin going to the board with the power pin allows it to move

//    power pin on the slider is shorted to control pin to the board

  for (int angle = 180; angle >= 0; angle--){
//    if(analogRead(pressurePin) > 1200){
//      break;
//    }
//    else{
    myservo.write(angle);
    delay(15);
  }
  delay(1000);  

//  myservo.write(180);
//  delay(2500);
//  myservo.write(90);
//  delay(1000);
//  myservo.write(0);
//  delay(2500);
 
// Sweep from 0 to 180 degrees:
//  for (angle = 0; angle <= 150; angle += 1) {
//    myservo.write(angle);
//    delay(30);
//  }
//
//  // And back from 180 to 0 degrees:
//  for (angle = 150; angle >= 0; angle -= 1) {
//    myservo.write(angle);
//    delay(30);
//  }

//    delay(2000);
}
