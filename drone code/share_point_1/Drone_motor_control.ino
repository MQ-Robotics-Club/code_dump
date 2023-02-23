

#include <Wire.h>
#include <Servo.h>
#include <WiFiUdp.h>

//declare motors as servos
Servo north_prop;
Servo south_prop;
Servo east_prop;
Servo west_prop;


void setup() {

//attach motors to PWM pins (make sure you change these to the right wires)
  north_prop.attach(7);
  south_prop.attach(6);
  east_prop.attach(5);
  west_prop.attach(4);
  
  //Run Motors at 0 speed for 7seconds to Arm
  north_prop.writeMicroseconds(1000);
  south_prop.writeMicroseconds(1000);
  east_prop.writeMicroseconds(1000);
  west_prop.writeMicroseconds(1000);
 
  delay(7000);

}

void loop() {

  //run motors on 20% throttle
 north_prop.writeMicroseconds(1200);          
south_prop.writeMicroseconds(1200);          
east_prop.writeMicroseconds(1200);   
west_prop.writeMicroseconds(1200);  


}
