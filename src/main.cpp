#include "HardwareSerial.h"
// this sample code provided by www.programmingboss.com
#define RXp2 16
#define TXp2 17
int8_t list[] = {0,0,0,0};
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RXp2, TXp2);
}
void loop() {
    if (Serial2.available() > 0){
        char byte_count = 0;
        Serial.print(Serial2.available());
        while (Serial2.available() > 0) {
            list[byte_count] = Serial2.read();
            byte_count += 1;
        }
        Serial.print(list[0]);
        Serial.print(list[1]);
        Serial.print(list[2]);
        Serial.println(list[3]);
    }
    delay(10);
}