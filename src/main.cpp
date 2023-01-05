#include <Wire.h>

#define SLAVE_ADDRESS 0x04

int8_t datafromEV3 = 5;
byte sendtoEV3[] = {10,20};

void setup() {
  Serial.begin(115200);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Serial.println("I2C slave ready!");
}

void loop() {
// 何もしない
}

// マスターからのデータ受信イベント
void receiveEvent(int howMany) {
  while (Wire.available() > 0) {
    datafromEV3 = Wire.read();
    Serial.print(datafromEV3);
  }
  Serial.println();
}

// マスターからのデータ要求イベント
void requestEvent() {
  if (datafromEV3 == 5){}
  else {
    Wire.write(sendtoEV3,2);
  }
}