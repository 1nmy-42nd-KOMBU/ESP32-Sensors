#include <Wire.h>
#include <VL53L1X.h>
#include "HardwareSerial.h"
// this sample code provided by www.programmingboss.com
#define RXp2 16
#define TXp2 17

#define Wire1_SDA (33)
#define Wire1_SCL (32)

VL53L1X sensor;
TaskHandle_t thp[1];

// data of each thread------------
// loop(tof)
int8_t tof_mm = 65;

void setup()
{
  Serial.begin(115200);
  Serial2.begin(57600, SERIAL_8N1, RXp2, TXp2);
  while(!Serial2); //wait untill it opens
  Wire1.begin(Wire1_SDA, Wire1_SCL);
  sensor.setBus(&Wire1);
  Wire1.setClock(100000); // use 100 kHz I2C

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  sensor.setDistanceMode(VL53L1X::Short); // long-mode
  sensor.setMeasurementTimingBudget(30000);
  sensor.startContinuous(1);

  /* Use external crystal for better accuracy */
  xTaskCreatePinnedToCore(Core1tof, "Core1tof", 4096, NULL, 3, &thp[0], 1);
}

// main-loop------------------------------------------------------------------------------
void loop()
{
  if (Serial2.available() > 0){
    int8_t start_byte_from_Arduino = Serial2.read();
    while(Serial2.available())Serial2.read(); // 受信バッファをからにする
    if (start_byte_from_Arduino == 32){
      char list = 0;
      list = tof_mm; // start bytem;
      Serial.print("tof;"); // tof------------------------------
      Serial.println(tof_mm);
      Serial2.write(list);
      tof_mm = 65;
    }
  }
}

// tof------------------------------------------------------------------------------------
void Core1tof(void *args) {//Core1で実行するプログラム
  while (1) {//ここで無限ループを作っておく
    int16_t tof_mm_tmp =sensor.read();
    if (sensor.timeoutOccurred()) { // タイムアウトの監視 
      Serial.print(" TIMEOUT");
      vlxReset(25);
    } else {
      if (tof_mm_tmp > 450 && (tof_mm == 65 || tof_mm == 62)){
        tof_mm = 65;
      } else if (tof_mm_tmp > 100 && (tof_mm == 65 || tof_mm == 62 || tof_mm == 64)){
        tof_mm = 64;
      } else if (tof_mm_tmp != 0){
        tof_mm = 63;
      }
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
  delay(600);
  digitalWrite(resetPin, HIGH);
  delay(600);

  if (!sensor.init()) {
    ESP_LOGE(TAG, "Failed To Detect Sensor.. Restarting!!");
    ESP.restart();
  }
  sensor.setTimeout(500);

  sensor.startContinuous(10);
}