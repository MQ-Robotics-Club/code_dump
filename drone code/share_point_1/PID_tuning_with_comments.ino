/* code hacked together quickly to tune the PID loop
 * 
 */

#include "PIDclass.h"
#include <Wire.h>
#include <Servo.h>

//declaring a PID loop (it's a seperate class cause it's nice to have less cluttered code)
PIDclass Xaxis(0, 0, 0); //starting constants

//declaring props as servos
Servo north_prop;
Servo south_prop;
const byte NORTH_PROP_pin = 6;
const byte SOUTH_PROP_pin = 5;



unsigned long elapsedTime, timePrev;
unsigned long time = 0;


////////////////////////////////get time function - used for PID//////////////////////////////////
void get_time(){
  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = time - timePrev;
}


///////////////////////////////////////////////////////IMU stuff (getting gyro angles from the MPU6050)///////////////////////////////////////////////////////////
const float rad_to_deg = 180/3.141592654;
int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];

void read_IMU(){
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true);
   
  Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
  Acc_rawY=Wire.read()<<8|Wire.read();
  Acc_rawZ=Wire.read()<<8|Wire.read();

  /*---X---*/
  Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
  /*---Y---*/
  Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
   
  Wire.beginTransmission(0x68);
  Wire.write(0x43); //Gyro data first adress
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,4,true); //Just 4 registers
   
  Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shift and sum
  Gyr_rawY=Wire.read()<<8|Wire.read();

  /*---X---*/
  Gyro_angle[0] = Gyr_rawX/131.0;
  /*---Y---*/
  Gyro_angle[1] = Gyr_rawY/131.0;
  /*---X axis angle---*/
  Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
  /*---Y axis angle---*/
  Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
   
  /*Now we have our angles in degree and values from -10º0 to 100º aprox*/
  //////////////////////end of IMU stuff --- angles stores in total_angle[1 and 2]////////////////////////////////////////
}



void setup(){
  Serial.begin(250000); //debug
   
  //IMU comunication
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  //initialise motors
  north_prop.attach(NORTH_PROP_pin);
  south_prop.attach(SOUTH_PROP_pin);

  north_prop.writeMicroseconds(1000);
  south_prop.writeMicroseconds(1000);

  //delay to inisilise ESCs
  delay(7000);
  Serial.print("start");
}



void loop(){
  get_time();

  read_IMU();

  Xaxis.calculate_PID(elapsedTime, 0, Acceleration_angle[0]);

  long power = 1500;
  north_prop.writeMicroseconds(power+Xaxis.PID);
  south_prop.writeMicroseconds(power-Xaxis.PID);



//adjusting PID constants based on 3 seperate potentiometers during tuning (just an easy way of doing it tbh)//
  //P = A1
  //I = A2
  //D = A3

  Xaxis._kp = (float)analogRead(A1)/500;
  Xaxis._ki = (float)analogRead(A2)/500;
  Xaxis._kd = (float)analogRead(A3)/500;

  if (Xaxis._ki < 0.001) Xaxis.pid_i=0;


//debugging prints
  Serial.print(Xaxis._kp);
  Serial.print(", ");
  Serial.print(Xaxis._ki);
  Serial.print(", ");
  Serial.print(Xaxis._kd);
  Serial.print(", ");
  Serial.println(Xaxis.PID);
}
