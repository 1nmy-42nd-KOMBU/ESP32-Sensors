void setup(){
  Serial.begin(9600);
  pinMode(4,INPUT_PULLUP);
  pinMode(0,INPUT_PULLUP);
  pinMode(2,INPUT_PULLUP);
  pinMode(15,INPUT_PULLUP);
}

void loop() {
  Serial.print("4;");
  Serial.print(digitalRead(4));
  Serial.print(" 0;");
  Serial.print(digitalRead(0));
  Serial.print(" 2;");
  Serial.print(digitalRead(2));
  Serial.print(" 15;");
  Serial.println(digitalRead(15));
  delay(100);
}

const uint8_t Left_Button = 4; // 押されたら1
const uint8_t Right_Button = 0; // 押されたら0
const uint8_t Arm_Left_Button = 15; // 押されたら1
const uint8_t Arm_Right_Button = 2;  // 押されたら0