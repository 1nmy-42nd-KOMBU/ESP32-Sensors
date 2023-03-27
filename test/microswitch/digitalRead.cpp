const uint8_t Left_Button = 4; // 押されたら1
const uint8_t Right_Button = 0; // 押されたら0
const uint8_t Arm_Left_Button = 15; // 押されたら1
const uint8_t Arm_Right_Button = 2;  // 押されたら0

void setup(){
  Serial.begin(9600);
  pinMode(Left_Button,INPUT_PULLUP);
  pinMode(Right_Button,INPUT_PULLUP);
  pinMode(Arm_Left_Button,INPUT_PULLUP);
  pinMode(Arm_Right_Button,INPUT_PULLUP);
}

void loop() {
  Serial.print("Left;");
  Serial.print(digitalRead(Left_Button));
  Serial.print(" Right;");
  Serial.print(digitalRead(Right_Button));
  Serial.print(" Arm Left;");
  Serial.print(digitalRead(Arm_Left_Button));
  Serial.print(" Arm Right;");
  Serial.println(digitalRead(Arm_Right_Button));
  delay(100);
}