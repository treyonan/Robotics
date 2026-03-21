#include <Servo.h>

Servo motor;

void setup() {

  delay(10000);
  motor.attach(10);
  motor.writeMicroseconds(1500);
  delay(3000);
    
}

void loop() {

  motor.writeMicroseconds(2000);
  delay(15);
    
}
