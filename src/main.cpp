#include <Arduino.h>
#include "HardwareSerial.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// pins of UART
#define RXp2 19
#define TXp2 18

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (50)
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire1);

// pins of I2C
#define Wire1_SDA (33)
#define Wire1_SCL (32)

// pins of Line Sensors
const uint8_t Left_Light_Sensor = 27;
const uint8_t Right_Light_Sensor = 14;
const uint8_t Central_Light_Sensor = 12;
const uint8_t RescueKit_Sensor = 26;

// pins of MicroSwitches
const uint8_t Left_Button = 4; // 押されたら1
const uint8_t Right_Button = 0; // 押されたら0
const uint8_t Arm_Left_Button = 15; // 押されたら1
const uint8_t Arm_Right_Button = 2;  // 押されたら0

// 何もなければ0 左で1 右で2 両方で3
// 1の位がフロント10の位がアーム
volatile uint8_t front_touch_sensor = 0; 
// 何もなければ0 上がって1 下がって2 シーソーで3(予定)
volatile uint8_t gyro_stats = 0;
// 何もなければ0 左が黒で1 右が黒で2 両方黒で3
volatile uint8_t line_sensor_statues = 0;
// 何もなければ0 中心のラインセンサが白で1 レスキューキットで2 両方で3
volatile uint8_t central_line_sensor_AND_rescue_kit = 0;

volatile bool should_turn_180 = false;

// MultiThread
TaskHandle_t thp[2]; //ToFを追加して3になる予定
// 優先順位はUART5→ジャイロ4→ToF3→タッチ&ライン-
void turn_180(void){
  sensors_event_t event;
  bno.getEvent(&event);

  int16_t start_angle = event.orientation.x;
  int16_t target_angle_range_min = start_angle + 170; // 目標角の範囲の最小値
  int16_t target_angle_range_max = start_angle + 180; // 目標角の範囲の最大値

  if (target_angle_range_max <360){
     // そのままでOK
    while (1){
      sensors_event_t event;
      bno.getEvent(&event);
      int angle_now = event.orientation.x;
      if (target_angle_range_min <= angle_now && angle_now < target_angle_range_max){break;}
      delay(BNO055_SAMPLERATE_DELAY_MS);
    }
  } else if (target_angle_range_max >= 360 && target_angle_range_min < 360){
    target_angle_range_max -= 360;
    while (1){
      sensors_event_t event;
      bno.getEvent(&event);
      int angle_now = event.orientation.x;
      if (target_angle_range_min <= angle_now && angle_now < 360 || 0 <= angle_now && angle_now <= target_angle_range_max){break;}
      delay(BNO055_SAMPLERATE_DELAY_MS);
    }
  } else if (target_angle_range_min >= 360){
    target_angle_range_max -= 360;
    target_angle_range_min -= 360;
    while (1){
      sensors_event_t event;
      bno.getEvent(&event);
      int angle_now = event.orientation.x;
      if (target_angle_range_min <= angle_now && angle_now < target_angle_range_max){break;}
      delay(BNO055_SAMPLERATE_DELAY_MS);
    }
  }

  while (not Serial2.available()){
    Serial2.write(180);
    delay(10);
  }
  int hoge = Serial2.read();
}

void BNO055(void *args) {//サブCPU(Core0)で実行するプログラム
  while (1) {
    // 180度回転とか
    if (should_turn_180){
      turn_180();
    }
    // 通常業務
    sensors_event_t event;
    bno.getEvent(&event);

    float deg_for_hill = (float)event.orientation.y;
    if (deg_for_hill > 5){
      gyro_stats = 1;
    } else if (deg_for_hill < -5){
      gyro_stats = 2;
    } else {
      gyro_stats = 0;
    }

    delay(BNO055_SAMPLERATE_DELAY_MS);
  }
}

void UART(void *args) {
  while (1) 
  {
    while (not Serial2.available()){
      delay(1);
    }
    int hoge = Serial2.read();
    if (hoge == 10){
      char listforEV3[4] = {line_sensor_statues,
                            front_touch_sensor,
                            central_line_sensor_AND_rescue_kit,
                            gyro_stats};
      Serial2.write(listforEV3,4);
      Serial.println("received 10 and sent 10");
    } else if (hoge == 180){
      should_turn_180 = true;
      Serial2.write(18);
    } else {
      Serial.print(hoge);
      Serial.println(",not 10 was sent");
    }
  }
}

void setup()
{
  // Serial with PC
  Serial.begin(115200);
  // Serial with EV3
  Serial2.begin(115200, SERIAL_8N1, RXp2, TXp2);
  while(!Serial2); //wait untill it opens

  // MultiThread
  xTaskCreatePinnedToCore(UART, "UART", 4096, NULL, 5, &thp[0], 1); 
  xTaskCreatePinnedToCore(BNO055, "BNO055", 4096, NULL, 5, &thp[1], 0);

  // BNO055
  Wire1.begin(Wire1_SDA, Wire1_SCL);
  /* Initialise the sensor */
  if(!bno.begin()){
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);

  // Line Sensors
  analogSetAttenuation(ADC_6db);
  pinMode(12, ANALOG);
  pinMode(14, ANALOG);
  pinMode(27, ANALOG);
  pinMode(26, ANALOG);

  // MicroSwitches
  pinMode(Left_Button,INPUT_PULLUP);
  pinMode(Right_Button,INPUT_PULLUP);
  pinMode(Arm_Left_Button,INPUT_PULLUP);
  pinMode(Arm_Right_Button,INPUT_PULLUP);

  Serial.println("start");
}

void loop() {
  // 左右のラインセンサ
  int left_light = analogRead(Left_Light_Sensor);
  int right_light = analogRead(Right_Light_Sensor);
  if (left_light <= 500 && right_light <= 500){
    line_sensor_statues = 3;
  } else if (left_light <= 500){
    line_sensor_statues = 1;
  } else if (right_light <= 500){
    line_sensor_statues = 2;
  } else {
    line_sensor_statues = 0;
  }
  // アームのラインセンサ
  int central_light = analogRead(Central_Light_Sensor);
  int rescue_kit = analogRead(RescueKit_Sensor);
  if (central_light >= 350 && rescue_kit >= 1000){
    central_line_sensor_AND_rescue_kit = 3;
  } else if (central_light >= 350){
    central_line_sensor_AND_rescue_kit = 1;
  } else if (rescue_kit >= 1000){
    central_line_sensor_AND_rescue_kit = 2;
  } else {
    central_line_sensor_AND_rescue_kit = 0;
  }
  // Serial.print("12central; ");
  // Serial.print(analogRead(Central_Light_Sensor)); // 350より高いと白
  // Serial.print(", 14right; ");
  // Serial.print(analogRead(Right_Light_Sensor)); // 500切ったら黒
  // Serial.print(", 27left; ");
  // Serial.print(analogRead(Left_Light_Sensor));
  // Serial.print(", 26kit; ");
  // Serial.println(analogRead(RescueKit_Sensor)); // 1000超えたらレスキューキット

  // Serial.print("Left;");
  // Serial.print(digitalRead(Left_Button));
  // Serial.print(" Right;");
  // Serial.print(digitalRead(Right_Button));
  // Serial.print(" Arm Left;");
  // Serial.print(digitalRead(Arm_Left_Button));
  // Serial.print(" Arm Right;");
  // Serial.println(digitalRead(Arm_Right_Button));
  // delay(100);

  // バンパーのタッチセンサ
  if (digitalRead(Left_Button) == 1 && digitalRead(Right_Button) == 0){
    front_touch_sensor = 3;
  } else if (digitalRead(Left_Button) == 1){
    front_touch_sensor = 1;
  } else if (digitalRead(Right_Button) == 0){
    front_touch_sensor = 2;
  } else {
    front_touch_sensor = 0;
  }
  // アームのタッチセンサ
  if (digitalRead(Arm_Left_Button) == 1 && digitalRead(Arm_Right_Button) == 0){
    front_touch_sensor += 30;
  } else if (digitalRead(Arm_Left_Button) == 1){
      front_touch_sensor += 10;
  } else if (digitalRead(Arm_Right_Button) == 0){
    front_touch_sensor += 20;
  }
  delay(1);
}