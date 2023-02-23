
#include <Wire.h>
#include <Servo.h>
#include <WiFiUdp.h>


Servo north_prop;
Servo south_prop;
Servo east_prop;
Servo west_prop;

//radio control stuff
byte PWM_X = 12;
byte PWM_Y = 10;
byte PWM_THROTTLE = 11;
 
int pwm_value_x;
int pwm_value_y;
int pwm_value_throttle;
 
int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];




float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

//X axis PID values //
float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;

//Y axis PID values //
float PID2, pwmLeft2, pwmRight2, error2, previous_error2;
float pid_p2=0;
float pid_i2=0;
float pid_d2=0;

/////////////////PID CONSTANTS/////////////////
double kp=0;
double ki=0;
double kd=0;

double kp2=0;
double ki2=0;
double kd2=0;



double throttle=1300; //initial value of throttle to the motors
float desired_X_angle = -9.7;
float desired_Y_angle = 3.9;


void setup() {
  //radio stuff
  pinMode(PWM_X, INPUT);
  pinMode(PWM_Y, INPUT);
  pinMode(PWM_THROTTLE, INPUT);
 
 
  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(250000);
  north_prop.attach(7);
  south_prop.attach(6);
  east_prop.attach(5);
  west_prop.attach(4);
 

  time = millis(); //Start counting time in milliseconds

  north_prop.writeMicroseconds(1000);
  south_prop.writeMicroseconds(1000);
  east_prop.writeMicroseconds(1000);
  west_prop.writeMicroseconds(1000);
 
  delay(7000);
}

void loop() {

//change constants with potentiometers
mapK(analogRead(A1),analogRead(A2),analogRead(A3));





  //radio stuff
  pwm_value_x = pulseIn(PWM_X, HIGH);
  pwm_value_y = pulseIn(PWM_Y, HIGH);
  pwm_value_throttle = pulseIn(PWM_THROTTLE, HIGH);
 
  desired_X_angle = map(pwm_value_x,800,2200,-40,40);
  desired_Y_angle = map(pwm_value_y,800,2200,-40,40);
  throttle = pwm_value_throttle;

/////////////////////////////I M U/////////////////////////////////////
    timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    elapsedTime = (time - timePrev) / 1000;
   
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
   
   Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
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
    //Serial.println(Total_angle[1]);

   
 
/*///////////////////////////P I D XXXXXXXX///////////////////////////////////*/

error = Total_angle[0] - desired_X_angle;
pid_p = kp*error;
if(-3 <error <3)
{
  pid_i = pid_i+(ki*error);  
}
pid_d = kd*((error - previous_error)/elapsedTime);
PID = (pid_p + pid_i + pid_d);

if(PID < -1000){
  PID= -1000;}
if(PID > 1000){
  PID= 1000;}

pwmLeft = throttle + PID;
pwmRight = throttle - PID;

//Right
if(pwmRight < 1000){
  pwmRight= 1000;}
if(pwmRight > 2000){
  pwmRight=2000;}
//Left
if(pwmLeft < 1000){
  pwmLeft= 1000;}
if(pwmLeft > 2000){
  pwmLeft=2000;}

////////////////////////////P  I  D  2  /  YYYYYYYY ///////////////////////////
 
error2 = Total_angle[1] - desired_Y_angle;
pid_p2 = kp2*error2;
if(-3 <error2 <3)
{
  pid_i2 = pid_i2+(ki2*error2);  
}
pid_d2 = kd2*((error2 - previous_error2)/elapsedTime);
PID2 = (pid_p2 + pid_i2 + pid_d2);

if(PID2 < -1000){
  PID2= -1000;}
if(PID2 > 1000){
  PID2= 1000;}


pwmLeft2 = throttle + PID2;
pwmRight2 = throttle - PID2;

//Right
if(pwmRight2 < 1000){
  pwmRight2= 1000;}
if(pwmRight2 > 2000){
  pwmRight2=2000;}
//Left
if(pwmLeft2  < 1000){
  pwmLeft2 = 1000;}
if(pwmLeft2  > 2000){
  pwmLeft2 =2000;}
//PID safety if throttle low
if (throttle < 1200){
  pwmLeft2 = 1000;
  pwmRight2 = 1000;
  pwmLeft = 1000;
  pwmRight = 1000;
}

north_prop.writeMicroseconds(pwmRight2);           //pwmLeft2);
south_prop.writeMicroseconds(pwmLeft2);           //pwmRight2);
east_prop.writeMicroseconds(pwmLeft);   //x axis
west_prop.writeMicroseconds(pwmRight);    //x axis

previous_error = error; //Remember to store the previous error.
previous_error2 = error2;




Serial.print("X angle input = ");
Serial.print(desired_X_angle);

Serial.print("   Throttle input = ");
Serial.print(throttle);
 
Serial.print("   X axis angle = ");
Serial.print(Total_angle[0]);

Serial.print("   PID math output value = ");
Serial.print(PID);

Serial.print("   motor input left = ");
Serial.print(pwmLeft);
Serial.print("       motor input right = ");
Serial.print(pwmRight);

Serial.print("    throttle value");
Serial.println(pwm_value_throttle);
//PID constants (currently x axis only)
Serial.print("    PID constants");
Serial.println(kp);
Serial.print(", ");
Serial.println(ki);
Serial.print(", ");
Serial.println(kd);

}//end of loop void

/////////////////////////////////PID constant adjustment - in analog, out change PID ////////////////////
void mapK(float P, float I, float D){
P = P*0.01;       //map(P,0,1024,0,10);
I = I*0.01;     //map(I,0,1024,0,10);
D = D*0.01  ;     //map(D,0,1024,0,10);
kp=P;
ki=I;
kd=D;
kp2=P;
ki2=I;
kd2=D;
  
}
