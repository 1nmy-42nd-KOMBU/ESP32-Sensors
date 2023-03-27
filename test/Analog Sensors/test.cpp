#include <Arduino.h>
#include <Wire.h>

const uint8_t Left_Light_Sensor = 27;
const uint8_t Right_Light_Sensr = 14;
const uint8_t Central_Light_Sensor = 12;
const uint8_t RescueKit_Sensor = 26;

void setup() {
  Serial.begin(115200);
  delay(100);

  analogSetAttenuation(ADC_6db);
  pinMode(12, ANALOG);
  pinMode(14, ANALOG);
  pinMode(27, ANALOG);
  pinMode(26, ANALOG);
}
void loop() {
  Serial.print("12central; ");
  Serial.print(analogRead(Central_Light_Sensor));
  Serial.print(", 14right; ");
  Serial.print(analogRead(Right_Light_Sensr));
  Serial.print(", 27left; ");
  Serial.print(analogRead(Left_Light_Sensor));
  Serial.print(", 26kit; ");
  Serial.println(analogRead(RescueKit_Sensor));
  delay(500);
}