#include <Arduino.h>
#include "HardwareSerial.h"
#include <Wire.h>

#define RXp2 19
#define TXp2 18

void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXp2, TXp2);
  while(!Serial2); //wait untill it opens
  Serial.println("start");
}

void loop() {
  while (not Serial2.available()){}
  int hoge = Serial2.read();
  if (hoge == 10){
    Serial2.write(10);
    Serial.println("received 10 and sent 10");
  } else {
    Serial.print(hoge);
    Serial.println(",not 10 was sent");
  }
}
