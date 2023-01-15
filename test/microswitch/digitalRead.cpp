const int buttonPin = 26;     // the number of the pushbutton pin

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(115200);
}

void loop() {
  Serial.println(digitalRead(buttonPin));
}