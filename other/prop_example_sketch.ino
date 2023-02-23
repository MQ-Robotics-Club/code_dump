#include <Servo.h>
Servo prop;
//define prop as servo to use PWN through servo library

void setup() {
prop.attach(3);
//attach on pin 3
  
prop.writeMicroseconds(1000);
//output 1000ms PWM for 7 seconds to arm Electronic Speed Controller (ESC0
//7 seconds not always necessary (product dependant)
 
  delay(7000);
}

void loop() {
  
prop.writeMicroseconds(1500);
//Run motor at half speed, (range is 1000 - 2000)
//have to use writeMicroseconds() to control with the spicific range (not sure if servo library even covers the range with it's 0 - 255, so just use this)

}
