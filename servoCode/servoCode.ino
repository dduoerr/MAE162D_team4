/* Servo motor with Arduino example code. Position and sweep. More info: https://www.makerguides.com/ */

// Include the servo library:
#include <Servo.h>

// Create a new servo object:
Servo myservo;

// Define the servo pin:
#define servoPin 9

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

//  myservo.write(0);
//  delay(2000);
//  myservo.write(180);

 // Sweep from 0 to 180 degrees:
  for (angle = 0; angle <= 150; angle += 1) {
    myservo.write(angle);
    delay(30);
  }

  // And back from 180 to 0 degrees:
  for (angle = 150; angle >= 0; angle -= 1) {
    myservo.write(angle);
    delay(30);
  }

//    delay(2000);
}
