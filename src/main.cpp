#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// VL53L0X[0]を左 VL53L0X[1]を右とする
VL53L0X VL53L0X[2];

const int xshut_left = 23;
const int xshut_right = 5;

byte vl_left_address = 0x10; // 左のVLのアドレスは0x10

void setup(){
  // まず全てのGPIOをLOW
  pinMode(xshut_left, OUTPUT);
  pinMode(xshut_right, OUTPUT);
  digitalWrite(xshut_left, LOW);
  digitalWrite(xshut_right, LOW);
  delay(1);

  digitalWrite(xshut_left, HIGH);
  delay(1);
  if (!VL53L0X[0].init())
  {
    while (1){
      Serial.println("Failed to detect and initialize vl_left!");
      delay(1000);
    }
  }
  VL53L0X[0].setTimeout(500);
  VL53L0X[0].startContinuous(10);
  VL53L0X[0].setAddress(vl_left_address);

  digitalWrite(xshut_right, HIGH);
  delay(1);
  if (!VL53L0X[1].init())
  {
    while (1){
      Serial.println("Failed to detect and initialize vl_left!");
      delay(1000);
    }
  }
  VL53L0X[1].setTimeout(500);
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
  if (!VL53L0X[0].init())
  {
    while (1){
      Serial.println("Failed to detect and initialize vl_left!");
      delay(1000);
    }
  }
  VL53L0X[0].setTimeout(500);
  VL53L0X[0].startContinuous(10);
  VL53L0X[0].setAddress(vl_left_address);

  digitalWrite(xshut_right, HIGH);
  delay(1);
  if (!VL53L0X[1].init())
  {
    while (1){
      Serial.println("Failed to detect and initialize vl_left!");
      delay(1000);
    }
  }
  VL53L0X[1].setTimeout(500);
  VL53L0X[1].startContinuous(10);
}