int ir_sensor_pin = 27; // IR sensor pin
void setup()
{
  Serial.begin(9600); // Serial monitor at baud rate 9600
  pinMode(ir_sensor_pin, INPUT); // Pin set as input
}


void loop() {
    if (digitalRead(ir_sensor_pin)) // if Pin logic is HIGH
    {
      Serial.println("Object detected"); // display on Serial monitor when object detected 
    }
  
    else {
     Serial.println("Object not detected"); // display on Serial when object not detected 
    }

    //Serial.println(analogRead(ir_sensor_pin)); 
//  Serial.println(digitalRead(ir_sensor_pin));
//  delay(1000);
}
