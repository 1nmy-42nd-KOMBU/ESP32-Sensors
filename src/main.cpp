#include <Arduino.h>
#include "Wire.h"

const int MicSw1 = 32;
const int MicSw2 = 33;
#define I2C_DEV_ADDR 0x04
byte read_byte = 0x00;
int byte_count = 0;
int instruction[8] = {0,0,0,0,0,0,0,0};
uint32_t i = 0;

void loop() {}

void onRequest(){
    Wire.write(1);
    Serial.println("onRequest");
}

void onReceive(int len){
    Serial.printf("onReceive[%d]: ", len);
    while(Wire.available()){
        Serial.print(Wire.read());
    }
    Serial.println();
    // read_byte = bytesIn;
    // byte_count = 0;
    // while(1 < Wire.available()) // loop through all but the last
    // {
    //     read_byte = Wire.read(); 
        
    //     instruction[byte_count] = read_byte;
        
    //     byte_count++;
    // }
    // int x = Wire.read(); // Read the last dummy byte (has no meaning, but must read it)
    // Serial.print("receive");
    // Serial.print(instruction[0]);
    // Serial.println(instruction[1]);
}

void setup()
{
    pinMode( MicSw1, INPUT_PULLUP );
    pinMode( MicSw2, INPUT_PULLUP );
    Serial.begin(9600);
    Serial.setDebugOutput(true);
    Wire.onReceive(onReceive);
    Wire.onRequest(onRequest);
    Wire.begin((uint8_t)I2C_DEV_ADDR);

#if CONFIG_IDF_TARGET_ESP32
    char message[64];
    snprintf(message, 64, "%u Packets.", i++);
    Wire.slaveWrite((uint8_t *)message, strlen(message));
#endif
}
