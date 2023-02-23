#include <RF24.h>
#include <SPI.h>

RF24 radio (7,8);

byte address [][6] = {"00001", "00002"};

float data [2];
int n=0;


void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address[0]);
  radio.openReadingPipe (1, address [1]);
  radio.setPALevel (RF24_PA_MIN);
  

}

void loop() {
  radio.stopListening();
  data [0] = analogRead (A1); //X 
  data [1] = analogRead (A2); //Y
  Serial.println(data[0]);
  Serial.println(data[1]);
  while (!radio.write (&data, sizeof(data))){
  }

}
