void setup() {
  Serial.begin(115200);
  delay(100);
  pinMode(39, ANALOG);
  analogSetAttenuation(ADC_11db);
}
void loop() {
  Serial.println(analogRead(39));
  delay(10);
}