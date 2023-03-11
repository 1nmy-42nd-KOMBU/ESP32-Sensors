#include <Arduino.h>
#include <Wire.h>

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
  Serial.print("12; ");
  Serial.print(analogRead(12));
  Serial.print(", 14; ");
  Serial.print(analogRead(14));
  Serial.print(", 27; ");
  Serial.print(analogRead(27));
  Serial.print(", 26; ");
  Serial.println(analogRead(26));
  delay(500);
}