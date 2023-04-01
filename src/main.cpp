#include <Arduino.h>
#include "HardwareSerial.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <VL53L0X.h>

// VL53L0X[0]を左 VL53L0X[1]を右とする
VL53L0X VL53L0X[2];

const int xshut_left = 5;
const int xshut_right = 23;

byte vl_left_address = 0x10; // 左のVLのアドレスは0x10

// pins of UART
#define RXp2 19
#define TXp2 18

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (20)
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

volatile uint8_t gyro_x = 0;
volatile bool should_turn_180 = false;
volatile bool serialwrite = false;
volatile bool notify_seesaw = false;

// 左が壁で1、右が壁で2、両方で3、何もなくて0
volatile uint8_t isWall = 0;
volatile uint8_t vl_distance_mm[4] = {0,0,0,0};

// MultiThread
TaskHandle_t thp[3]; //ToFを追加して3になる予定
// 優先順位はUART5→ジャイロ4→ToF3→タッチ&ライン-

void turn_for_designated_angle(int16_t degree);
void BNO055(void *args);
void UART(void *args);
void vlxReset(void);;
void line_sensors(void *args);
// ================================================================================

void line_sensors(void *args){
  while (1){
  // 左右のラインセンサ
    int left_light = analogRead(Left_Light_Sensor);
    int right_light = analogRead(Right_Light_Sensor);
    if (left_light <= 1000 && right_light <= 1000){
      line_sensor_statues = 3;
    } else if (left_light <= 1000){
      line_sensor_statues = 1;
    } else if (right_light <= 1000){
      line_sensor_statues = 2;
    } else {
      line_sensor_statues = 0;
    }
    // アームのラインセンサ
    int central_light = analogRead(Central_Light_Sensor);
    int rescue_kit = analogRead(RescueKit_Sensor);
    if (central_light >= 380 && rescue_kit >= 3000){
      central_line_sensor_AND_rescue_kit = 3;
    } else if (central_light >= 380){
      central_line_sensor_AND_rescue_kit = 1;
    } else if (rescue_kit >= 3000){
      central_line_sensor_AND_rescue_kit = 2;
    } else {
      central_line_sensor_AND_rescue_kit = 0;
    }
    // Serial.print("12central; ");
    // Serial.println(analogRead(Central_Light_Sensor)); // 350より高いと白
    // delay(500);
    // Serial.print(", 14right; ");
    // Serial.print(analogRead(Right_Light_Sensor)); // 500切ったら黒
    // Serial.print(", 27left; ");
    // Serial.print(analogRead(Left_Light_Sensor));
    Serial.print(", 26kit; ");
    Serial.println(analogRead(RescueKit_Sensor)); // 1000超えたらレスキューキット

    // Serial.print("Left;");
    // Serial.print(digitalRead(Left_Button));
    // Serial.print(" Right;");
    // Serial.print(digitalRead(Right_Button));
    // Serial.print(" Arm Left;");
    // Serial.print(digitalRead(Arm_Left_Button));
    // Serial.print(" Arm Right;");
    // Serial.println(digitalRead(Arm_Right_Button));
    // delay(300);

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
}

// ================================================================================

void turn_for_designated_angle(int16_t degree){
  sensors_event_t event;
  bno.getEvent(&event);

  int16_t start_angle = event.orientation.x;
  int16_t target_angle_range_min = start_angle + degree - 5; // 目標角の範囲の最小値
  int16_t target_angle_range_max = start_angle + degree + 5; // 目標角の範囲の最大値
  Serial.println(start_angle);
  Serial.println(target_angle_range_min);
  Serial.println(target_angle_range_max);
  // 繰り上がりとかを考慮しつつ180度回転するのを待つ
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
  // 終わりを告げる
  serialwrite = true; // セマフォ的な
  delay(1);
  while (not Serial2.available()){
    Serial2.write((char)degree);
    delay(20);
  }
  Serial2.read(); // read 180 or 90
  serialwrite = false;
}

void BNO055(void *args) {
  float previous_y = 0;
  while (1) {
    // 180度回転
    if (should_turn_180){
      turn_for_designated_angle(180);
      should_turn_180 = false;
    }
    // 通常業務
    sensors_event_t event;
    bno.getEvent(&event);

    float deg_for_hill = (float)event.orientation.y;
    if (deg_for_hill - previous_y < -3){
      gyro_stats = 3;
      notify_seesaw = true;
    } else if (deg_for_hill < -3){
      gyro_stats = 2;
    } else if (deg_for_hill > 3){
      gyro_stats = 1;
    } else {
      gyro_stats = 0;
    }
    float z_hill = (float)event.orientation.z;
    if (z_hill >= 3){
      gyro_stats += 10;
    } else if (z_hill <= -3){
      gyro_stats += 20;
    }
    gyro_x = (uint8_t)(event.orientation.x / 2);

    delay(BNO055_SAMPLERATE_DELAY_MS);
  }
}

void UART(void *args) {
  while (1) 
  {
    while ((not Serial2.available()) || serialwrite){
      delay(1);
    }
    int hoge = Serial2.read();
    uint8_t tmp_gyro_stats = gyro_stats;
    if (notify_seesaw){
      tmp_gyro_stats = 3;
      notify_seesaw = false;
    }
    if (hoge == 10){
      char listforEV3_10[4] = {line_sensor_statues,
                            front_touch_sensor,
                            central_line_sensor_AND_rescue_kit,
                            tmp_gyro_stats};
      Serial2.write(listforEV3_10,4);
      Serial.println("received 10 and sent 4 Byte");
    } else if (hoge == 11){ // 障害物
      char listforEV3_11[7] = {vl_distance_mm[0],vl_distance_mm[1],vl_distance_mm[2],vl_distance_mm[3],
                              front_touch_sensor,
                              line_sensor_statues,
                              gyro_x};
      Serial2.write(listforEV3_11,7);
    } else if (hoge == 12){ // レスキュー
      char listforEV3_12[6] = {vl_distance_mm[0],vl_distance_mm[1],vl_distance_mm[2],vl_distance_mm[3],
                              front_touch_sensor,gyro_x};
      Serial2.write(listforEV3_12,6);
      Serial.println("send 6 byte");
    } else if(hoge == 180){
      should_turn_180 = true;
      Serial2.write(18);
    } else {
      Serial.print(hoge);
      Serial.println(",unknown code has been sent");
    }
  }
}

// ================================================================================
void setup()
{
  // Serial with PC
  Serial.begin(115200);
  // Serial with EV3
  Serial2.begin(115200, SERIAL_8N1, RXp2, TXp2);
  while(!Serial2); //wait untill it opens
  Wire.begin();

  // MultiThread ------------------------------------------------------------
  xTaskCreatePinnedToCore(UART, "UART", 4096, NULL, 3, &thp[0], 0); 
  xTaskCreatePinnedToCore(BNO055, "BNO055", 4096, NULL, 5, &thp[1], 0);
  xTaskCreatePinnedToCore(line_sensors,"line_sensors",4096,NULL,2,&thp[2],0);
  // BNO055 ------------------------------------------------------------
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

  // Line Sensors ------------------------------------------------------------
  analogSetAttenuation(ADC_6db);
  pinMode(12, ANALOG);
  pinMode(14, ANALOG);
  pinMode(27, ANALOG);
  pinMode(26, ANALOG);

  // MicroSwitches ------------------------------------------------------------
  pinMode(Left_Button,INPUT_PULLUP);
  pinMode(Right_Button,INPUT_PULLUP);
  pinMode(Arm_Left_Button,INPUT_PULLUP);
  pinMode(Arm_Right_Button,INPUT_PULLUP);

  // vl0x ------------------------------------------------------------
  // まず全てのGPIOをLOW
  pinMode(xshut_left, OUTPUT);
  pinMode(xshut_right, OUTPUT);
  digitalWrite(xshut_left, LOW);
  digitalWrite(xshut_right, LOW);
  delay(1);

  // 1台目
  digitalWrite(xshut_left, HIGH);
  delay(1);
  VL53L0X[0].setTimeout(500);
  if (!VL53L0X[0].init())
  {
    while (1){
      Serial.println("Failed to detect and initialize vl_left!");
      delay(1000);
    }
  }
  VL53L0X[0].setAddress(vl_left_address);
  
  // 射程を広げる
  VL53L0X[0].setSignalRateLimit(0.1);
  VL53L0X[0].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  VL53L0X[0].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

  // 高速モード
  VL53L0X[0].setMeasurementTimingBudget(20000);

  // start
  VL53L0X[0].startContinuous(10);

  // 2台目
  digitalWrite(xshut_right, HIGH);
  delay(1);
  VL53L0X[1].setTimeout(500);
  if (!VL53L0X[1].init())
  {
    while (1){
      Serial.println("Failed to detect and initialize vl_left!");
      delay(1000);
    }
  }

  // 射程を広げる
  VL53L0X[1].setSignalRateLimit(0.1);
  VL53L0X[1].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  VL53L0X[1].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

  // 高速モード
  VL53L0X[1].setMeasurementTimingBudget(20000);

  // start
  VL53L0X[1].startContinuous(10);
  
  // ------------------------------------------------------------
  Serial.println("start");
}

// ================================================================================

void loop() {
    uint16_t vl_left_mm = VL53L0X[0].readRangeSingleMillimeters();
    if (VL53L0X[0].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    uint16_t vl_right_mm = VL53L0X[1].readRangeSingleMillimeters();
    if (VL53L0X[1].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    // Serial.print("left; ");
    // Serial.print(vl_left_mm);
    // Serial.print(", right; ");
    // Serial.print(vl_right_mm);
    // Serial.println();

    // 横に壁があるかを判定
    if (vl_left_mm <= 200 && vl_right_mm <= 200){
      isWall = 3;
    } else if (vl_left_mm <= 200){
      isWall = 1;
    } else if (vl_right_mm <= 200){
      isWall = 2;
    } else {
      isWall = 0;
    }

    vl_distance_mm[0] = highByte(vl_left_mm);
    vl_distance_mm[1] = lowByte(vl_left_mm);
    vl_distance_mm[2] = highByte(vl_right_mm);
    vl_distance_mm[3] = lowByte(vl_right_mm);    
}

void vlxReset()
{
  Serial.println("Resetting Sensor");
 /* Stop the Sensor reading and even stop the i2c Transmission */
  VL53L0X[0].stopContinuous();
  VL53L0X[1].stopContinuous();
  Wire.endTransmission();

  digitalWrite(xshut_left, LOW);
  digitalWrite(xshut_right, LOW);
  delay(1);

  digitalWrite(xshut_left, HIGH);
  delay(1);
  VL53L0X[0].setTimeout(500);
  if (!VL53L0X[0].init())
  {
    while (1){
      Serial.println("Failed to detect and initialize vl_left!");
      delay(1000);
    }
  }
  VL53L0X[0].setAddress(vl_left_address);
    // 射程を広げる
  VL53L0X[0].setSignalRateLimit(0.1);
  VL53L0X[0].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  VL53L0X[0].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

  // 高速モード
  VL53L0X[0].setMeasurementTimingBudget(20000);
  VL53L0X[0].startContinuous(10);

  digitalWrite(xshut_right, HIGH);
  delay(1);
  VL53L0X[1].setTimeout(500);
  if (!VL53L0X[1].init())
  {
    while (1){
      Serial.println("Failed to detect and initialize vl_left!");
      delay(1000);
    }
  }

  // 射程を広げる
  VL53L0X[1].setSignalRateLimit(0.1);
  VL53L0X[1].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  VL53L0X[1].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

  // 高速モード
  VL53L0X[1].setMeasurementTimingBudget(20000);
  VL53L0X[1].startContinuous(10);
}
