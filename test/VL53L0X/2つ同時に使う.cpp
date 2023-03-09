#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// VL53L0X[0]を左 VL53L0X[1]を右とする
VL53L0X VL53L0X[2];

const int xshut_left = 23;
const int xshut_right = 5;

byte vl_left_address = 0x10; // 左のVLのアドレスは0x10

void setup(){
  Serial.begin(115200);
  Wire.begin();

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

void loop()
{
  Serial.print("left; ");
  Serial.print(VL53L0X[0].readRangeSingleMillimeters());
  if (VL53L0X[0].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.print(", right; ");
  Serial.print(VL53L0X[1].readRangeSingleMillimeters());
  if (VL53L0X[1].timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();
}