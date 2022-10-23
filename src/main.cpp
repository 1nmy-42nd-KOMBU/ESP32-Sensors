#include <Arduino.h>
#include "Wire.h"

void loop() {}

void onReceive(int len){
    Serial.printf("onReceive[%d]: ", len);
    while(Wire.available()){
        Serial.print(Wire.read());
    }
    Serial.println();
}

void onRequest(){
    Serial.println("onRequest");
    Wire.write(1);
}

void setup()
{
    Serial.begin(9600);
    Serial.setDebugOutput(true);
    Wire.onReceive(onReceive);
    Wire.onRequest(onRequest);
    Wire.begin(0x04);
}
