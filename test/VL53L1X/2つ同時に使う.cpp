#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

VL53L1X vl_left;
VL53L1X vl_right;

const int xshut_left = 23;
const int xshut_right = 5;

byte vl_left_address = 0x10; // 左のVLのアドレスは0x10

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // まず全てのGPIOをLOW
  pinMode(xshut_left, OUTPUT);
  pinMode(xshut_right, OUTPUT);
  digitalWrite(xshut_left, LOW);
  digitalWrite(xshut_right, LOW);
  delay(1);

  digitalWrite(xshut_left, HIGH); // 左センサを有効状態に戻す
  delay(1);
  vl_left.setTimeout(500); //[msec]
  if (!vl_left.init())
  {
    while (1){
      Serial.println("Failed to detect and initialize vl_left!");
      delay(1000);
    }
  }
  vl_left.setAddress(vl_left_address); //2台目センサーのアドレス変更
  // -------------------------------------------------------------------------------

  digitalWrite(xshut_right, HIGH); //右のVLセンサを有効状態に戻す
  delay(1);
  vl_right.setTimeout(500); //[msec]
  if (!vl_right.init())
  {
    while (1){
      Serial.println("Failed to detect and initialize vl_right!");
      delay(1000);
    }
  }

  vl_right.setDistanceMode(VL53L1X::Long);
  vl_left.setDistanceMode(VL53L1X::Long);
  vl_right.setMeasurementTimingBudget(50000); //測定タイミングバジェット(1回の距離測定)に許容される時間[micros]
  vl_left.setMeasurementTimingBudget(50000); 
  vl_right.startContinuous(50);
  vl_left.startContinuous(50);
}

void vlxReset()
{
  Serial.println("Resetting Sensor");
 /* Stop the Sensor reading and even stop the i2c Transmission */
  vl_left.stopContinuous();
  vl_right.stopContinuous();
  Wire.endTransmission();

/* Reset the sensor over here, you can change the delay */
  digitalWrite(xshut_left, LOW);
  digitalWrite(xshut_right, LOW);
  delay(600);
  digitalWrite(xshut_left, HIGH); // 左センサを有効状態に戻す
  delay(1);
  vl_left.setTimeout(500); //[msec]
  if (!vl_left.init())
  {
    while (1){
      Serial.println("Failed to detect and initialize vl_left!");
      delay(1000);
    }
  }
  vl_left.setAddress(vl_left_address); //2台目センサーのアドレス変更
  // -------------------------------------------------------------------------------

  digitalWrite(xshut_right, HIGH); //右のVLセンサを有効状態に戻す
  delay(1);
  vl_right.setTimeout(500); //[msec]
  if (!vl_right.init())
  {
    while (1){
      Serial.println("Failed to detect and initialize vl_right!");
      delay(1000);
    }
  }

  vl_right.setDistanceMode(VL53L1X::Long);
  vl_left.setDistanceMode(VL53L1X::Long);
  vl_right.setMeasurementTimingBudget(50000); //測定タイミングバジェット(1回の距離測定)に許容される時間[micros]
  vl_left.setMeasurementTimingBudget(50000); 
  vl_right.startContinuous(50);
  vl_left.startContinuous(50);
}

void loop() {
  Serial.print("LEFT: ");
  Serial.print(vl_left.read());
  if (vl_left.timeoutOccurred()) { 
    Serial.print(" TIMEOUT LEFT");
    vlxReset()
  }

  Serial.print(", RIGHT: ");
  Serial.print(vl_right.read());
  if (vl_right.timeoutOccurred()) {
    Serial.print(" TIMEOUT RIGHT");
    vlxReset()
  }
  Serial.println();
  delay(500);
}