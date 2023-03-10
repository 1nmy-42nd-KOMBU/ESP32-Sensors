#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(100);

  analogSetAttenuation(ADC_6db);
  pinMode(39, ANALOG);
  pinMode(34, ANALOG);
}
void loop() {
  Serial.print("kit; ");
  Serial.print(analogRead(39));
  Serial.print(", 34(left); ");
  Serial.print(analogRead(34));
  Serial.print(", 35(right); ");
  Serial.print(analogRead(35));
  Serial.print(", 32(central); ");
  Serial.println(analogRead(32));
  delay(500);
}