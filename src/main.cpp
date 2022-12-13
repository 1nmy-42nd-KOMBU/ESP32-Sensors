#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <VL53L1X.h>
#include "HardwareSerial.h"
// this sample code provided by www.programmingboss.com
#define RXp2 16
#define TXp2 17

#define BNO055_SAMPLERATE_DELAY_MS (10)
#define Wire1_SDA (33)
#define Wire1_SCL (32)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire1);
VL53L1X sensor;
TaskHandle_t thp[3];

// data of each thread------------
// loop(tof)
int8_t tof_mm[2];
// Core0GyS(Gyro)
int8_t Gyro_degree[2];
int8_t bump = 0;
// Core1USL(ultrasonic-left)
int8_t zipped_ultrasonic_left_cm;
// Core0USR(ultraconic-right)
int8_t zipped_ultrasonic_right_cm;

int16_t xyz_degree_tmp[3];

void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXp2, TXp2);
  while(!Serial2); //wait untill it opens
  Wire1.begin(Wire1_SDA, Wire1_SCL);
  Wire.begin();
  Wire.setClock(100000); // use 100 kHz I2C

  pinMode(5, OUTPUT);
  pinMode(18, OUTPUT);

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  sensor.setDistanceMode(VL53L1X::Long); // long-mode
  sensor.setMeasurementTimingBudget(50000);
  sensor.startContinuous(10);

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
  xTaskCreatePinnedToCore(Core0GyS, "CoreGyS", 4096, NULL, 3, &thp[0], 0); 
  xTaskCreatePinnedToCore(Core1US, "Core1US", 4096, NULL, 3, &thp[1], 1);
  xTaskCreatePinnedToCore(Core1tof, "Core1tof", 4096, NULL, 3, &thp[2], 1);
}

// main-loop------------------------------------------------------------------------------
void loop()
{
  if (Serial2.available() > 0){
    int8_t start_byte_from_Arduino = Serial2.read();
    if (start_byte_from_Arduino == 32){
      char list[9] = {0,0,0,0,0,0,0,0,0};
      list[0] = 64; // start byte
      list[1] = tof_mm[0];
      list[2] = tof_mm[1];
      Serial.print("tof;"); // tof------------------------------
      if (tof_mm[1] < 0){
        Serial.println(tof_mm[0]*256+tof_mm[1]+256);
      } else {Serial.println(tof_mm[0]*256+tof_mm[1]);}
    
      Serial.print("Gyro;"); // Gyro----------------------------
      list[3] = Gyro_degree[0];
      list[4] = Gyro_degree[1];
      list[5] = bump;
      int16_t x_tmp = Gyro_degree[0]*256+Gyro_degree[1];
      if (x_tmp <= -1 && x_tmp >= -128){
        Serial.print(x_tmp+256);
      } else {Serial.print(x_tmp);}
      Serial.print(",");
      Serial.println(bump);
    
      Serial.print("ultrasonic left;"); // ultrasonic-----------
      list[6] = zipped_ultrasonic_left_cm;
      list[7] = zipped_ultrasonic_right_cm;
      Serial.println((zipped_ultrasonic_left_cm + 127)*2);
      Serial.print("ultrasonic right;");
      Serial.println((zipped_ultrasonic_right_cm + 127)*2);
      Serial2.write(list,9);
    }
  }
}

// tof------------------------------------------------------------------------------------
void Core1tof(void *args) {//Core1で実行するプログラム
  while (1) {//ここで無限ループを作っておく
    int16_t tof_mm_tmp =sensor.read();
    if (sensor.timeoutOccurred()) { // タイムアウトの監視 
      Serial.print(" TIMEOUT");
      vlxReset(19);
    } else {
      tof_mm[0] = byte(highByte(tof_mm_tmp));
      tof_mm[1] = byte(lowByte(tof_mm_tmp));
    }
  }
}

// restart tof - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void vlxReset(uint8_t resetPin) {
  Serial.println("Resetting Sensor");
 /* Stop the Sensor reading and even stop the i2c Transmission */
  sensor.stopContinuous();
  Wire.endTransmission();

/* Reset the sensor over here, you can change the delay */
  digitalWrite(resetPin, LOW);
  vTaskDelay(600 / portTICK_PERIOD_MS);  // Change to delay(600); if not using rtos
  digitalWrite(resetPin, HIGH);
  vTaskDelay(600 / portTICK_PERIOD_MS);

  if (!sensor.init()) {
    ESP_LOGE(TAG, "Failed To Detect Sensor.. Restarting!!");
    ESP.restart();
  }
  sensor.setTimeout(500);

  sensor.startContinuous(10);
}

// Gyro-------------------------------------------------------------------------------------
void Core0GyS(void *args) {//サブCPU(Core0)で実行するプログラム
  while (1) {//ここで無限ループを作っておく
    /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);

    /* The processing sketch expects data as roll, pitch, heading */
    xyz_degree_tmp[0] = (int16_t)event.orientation.x;
    xyz_degree_tmp[1] = (int16_t)event.orientation.y;
    xyz_degree_tmp[2] = (int16_t)event.orientation.z;

    Gyro_degree[0] = highByte(xyz_degree_tmp[0]);
    Gyro_degree[1] = lowByte(xyz_degree_tmp[0]);

    // 上りなら1、下りなら2、0なら3を返す、何もなければ0
    if (abs(xyz_degree_tmp[1]) + abs(xyz_degree_tmp[2]) > 7)
    {
      if (xyz_degree_tmp[1] < 0){
        bump = 1;
      } else if (xyz_degree_tmp[1] > 0){
        bump = 2;
      } else {bump = 3;}
    }else{
      bump=0;
    }

    delay(BNO055_SAMPLERATE_DELAY_MS);
  }
}

// ultrasonic-------------------------------------------------------------------------------
void Core1US(void *args) {// Core1で実行するプログラム
  const int pingPin_left = 18; // set for 18 pin
  unsigned long duration_left;
  int16_t cm_left;
  const int pingPin_right = 5; // set for 5 pin
  unsigned long duration_right;
  int16_t cm_right;
  while (1) {//ここで無限ループを作っておく
    // 18-pin-------------------------------------------------------------------------------
    //ピンをOUTPUTに設定（パルス送信のため）
    pinMode(pingPin_left, OUTPUT);
    //LOWパルスを送信
    digitalWrite(pingPin_left, LOW);
    delayMicroseconds(2);  
    //HIGHパルスを送信
    digitalWrite(pingPin_left, HIGH);  
    //5uSパルスを送信してPingSensorを起動
    delayMicroseconds(5); 
    digitalWrite(pingPin_left, LOW); 

    //入力パルスを読み取るためにデジタルピンをINPUTに変更（シグナルピンを入力に切り替え）
    pinMode(pingPin_left, INPUT);   

    //入力パルスの長さを測定
    duration_left = pulseIn(pingPin_left, HIGH);  

    //パルスの長さを半分に分割
    duration_left=duration_left/2;  
    //cmに変換
    cm_left = int16_t(duration_left/29); 

    zipped_ultrasonic_left_cm = cm_left/2-127;
    
    delay(5);
    // 5-pin--------------------------------------------------------------------------------
    //ピンをOUTPUTに設定（パルス送信のため）
    pinMode(pingPin_right, OUTPUT);
    //LOWパルスを送信
    digitalWrite(pingPin_right, LOW);
    delayMicroseconds(2);  
    //HIGHパルスを送信
    digitalWrite(pingPin_right, HIGH);  
    //5uSパルスを送信してPingSensorを起動
    delayMicroseconds(5); 
    digitalWrite(pingPin_right, LOW); 

    //入力パルスを読み取るためにデジタルピンをINPUTに変更（シグナルピンを入力に切り替え）
    pinMode(pingPin_right, INPUT);

    //入力パルスの長さを測定
    duration_right = pulseIn(pingPin_right, HIGH);  

    //パルスの長さを半分に分割
    duration_right=duration_right/2;  
    //cmに変換
    cm_right = int16_t(duration_right/29); 

    zipped_ultrasonic_right_cm = cm_right/2-127;
    delay(5);
  }
}
