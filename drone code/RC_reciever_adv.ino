#include <RF24.h>
#include <SPI.h>

float data[2];
float Ypower=0;
float leftPower;
float rightPower;
boolean neutral=true;

 RF24 radio (10, 9);

byte address[][6] = {"00001", "00002"};

int motorL[] = {4,2,3};
int motorR[] = {7,8,6};

void forward (){
  digitalWrite (motorL[0], LOW);
  digitalWrite (motorL[1], HIGH);
  digitalWrite (motorR[0], LOW);
  digitalWrite (motorR[1], HIGH);
}

void reverse () {
  digitalWrite (motorL[0], HIGH);
  digitalWrite (motorL[1], LOW);
  digitalWrite (motorR[0], HIGH);
  digitalWrite (motorR[1], LOW);
}

void power(int powerL, int powerR){
  analogWrite (motorL[2], powerL);
  analogWrite (motorR[2], powerR);
}


void setup() {
    for (int i=0; i<3; i++){
      pinMode (motorL[i], OUTPUT);
       pinMode (motorR[i], OUTPUT);
    }

    Serial.begin(9600);

    radio.begin();
    radio.openWritingPipe (address[1]);
    radio.openReadingPipe (1, address [0]);
    radio.setPALevel (RF24_PA_MIN);
}

void loop() {
  radio.startListening();
  if (radio.available ()){
    while (radio.available()){
      radio.read(&data, sizeof(data));
      /*Serial.print ("X is ");
      Serial.println (data[0]);
      Serial.print ("Y is ");
      Serial.println (data[1]);*/
    }
  }
    if (data[0]<507) {
    reverse();
    Ypower = map (data[0], 507, 0, 0,255);
  } else if (data[0]>507){
    forward();
    Ypower = map(data[0], 507, 1023, 0, 255);
  }

  if (data[1]<507){
    rightPower = (map (data[1], 507.0, 0.0, 100, 0))/100.0;
    leftPower=0.75;
  } else if (data[1]>507){
    leftPower = (map (data[1], 507.0, 1023.0, 100, 0))/100.0;
    rightPower=0.75;
  }


 Serial.print("Ypower ");
 Serial.println(Ypower);
 Serial.print("leftPower ");
 Serial.println(leftPower);
 Serial.print ("rightPower ");
 Serial.println (rightPower); 
  
  power (Ypower*rightPower , Ypower*leftPower);
}
