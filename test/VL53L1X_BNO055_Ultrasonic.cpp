#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <VL53L1X.h>

VL53L1X sensor;
#define BNO055_SAMPLERATE_DELAY_MS (10)
#define Wire1_SDA (33)
#define Wire1_SCL (32)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire1);
TaskHandle_t thp[4];

// data of each thread------------
// loop(tof)
int tof_cm;
// Core0GyS(Gyro)
int Gyro_degree[3];
// Core1USL(ultrasonic-left)
int ultrasonic_left_cm;
// Core0USR(ultraconic-right)
int ultrasonic_right_cm;

bool canUpdate = true;

void setup()
{
  Serial.begin(115200);
  Wire1.begin(Wire1_SDA, Wire1_SCL);
  Wire.begin();
  Wire.setClock(100000); // use 100 kHz I2C

  pinMode(pingPin, OUTPUT);

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
  xTaskCreatePinnedToCore(Core1USR, "Core1USR", 4096, NULL, 3, &thp[1], 1);
  xTaskCreatePinnedToCore(Core1USL, "Core1USL", 4096, NULL, 3, &thp[2], 1);
  xTaskCreatePinnedToCore(Core1tof, "Core1tof", 4096, NULL, 3, &thp[3], 1);
}

// main-loop------------------------------------------------------------------------------
void loop()
{
  delay(100);
  canUpdate = false;
  delayMicroseconds(10);
  Serial.println(tof_cm);
  for (int i = 0; i < 3; ++i) {
    Serial.println(Gyro_degree[i]);
  }
  Serial.println(ultrasonic_left_cm);
  canUpdate = true;
}

// tof------------------------------------------------------------------------------------
void Core1tof(void *args) {//Core1で実行するプログラム
  while (1) {//ここで無限ループを作っておく
    int tof_cm_tmp =sensor.read();
    if (sensor.timeoutOccurred()) { // タイムアウトの監視 
      Serial.print(" TIMEOUT");
      vlxReset(19);
    }
    if (canUpdate){
      tof_cm = tof_cm_tmp;
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
    int xyz_degree[] = {(int)event.orientation.x,(int)event.orientation.y,(int)event.orientation.z};

    if (canUpdate){
      for (int i = 0; i < 3; ++i) {
        Gyro_degree[i] = xyz_degree[i];
      }
    }

    delay(BNO055_SAMPLERATE_DELAY_MS);
  }
}

// ultrasonic-Left--------------------------------------------------------------------------
void Core1USL(void *args) {// Core1で実行するプログラム
  const int pingPin = 18;
  unsigned long duration;
  int cm;
  while (1) {//ここで無限ループを作っておく
    //ピンをOUTPUTに設定（パルス送信のため）
    pinMode(pingPin, OUTPUT);
    //LOWパルスを送信
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);  
    //HIGHパルスを送信
    digitalWrite(pingPin, HIGH);  
    //5uSパルスを送信してPingSensorを起動
    delayMicroseconds(5); 
    digitalWrite(pingPin, LOW); 
    
    //入力パルスを読み取るためにデジタルピンをINPUTに変更（シグナルピンを入力に切り替え）
    pinMode(pingPin, INPUT);   
    
    //入力パルスの長さを測定
    duration = pulseIn(pingPin, HIGH);  
  
    //パルスの長さを半分に分割
    duration=duration/2;  
    //cmに変換
    cm = int(duration/29); 

    if (canUpdate){
      ultrasonic_left_cm = cm;
    }
    delay(25);
  }
}

// ultrasonic-Right--------------------------------------------------------------------------
void Core1USR(void *args) {// Core1で実行するプログラム
  const int pingPin = 5;
  unsigned long duration;
  int cm;
  while (1) {//ここで無限ループを作っておく
    //ピンをOUTPUTに設定（パルス送信のため）
    pinMode(pingPin, OUTPUT);
    //LOWパルスを送信
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);  
    //HIGHパルスを送信
    digitalWrite(pingPin, HIGH);  
    //5uSパルスを送信してPingSensorを起動
    delayMicroseconds(5); 
    digitalWrite(pingPin, LOW); 
    
    //入力パルスを読み取るためにデジタルピンをINPUTに変更（シグナルピンを入力に切り替え）
    pinMode(pingPin, INPUT);   
    
    //入力パルスの長さを測定
    duration = pulseIn(pingPin, HIGH);  
  
    //パルスの長さを半分に分割
    duration=duration/2;  
    //cmに変換
    cm = int(duration/29); 

    if (canUpdate){
      ultrasonic_right_cm = cm;
    }
    delay(25);
  }
}