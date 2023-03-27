#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;
#define Wire1_SDA (33)
#define Wire1_SCL (32)
TaskHandle_t thp[1];
uint16_t tof = 0;
#define SLAVE_ADDRESS 0x04

int8_t datafromEV3 = 5;

void setup()
{
  Serial.begin(115200);
  Wire1.begin(Wire1_SDA, Wire1_SCL);
  Wire1.setClock(100000); // use 100 kHz I2C
  Wire.begin();
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  sensor.setBus(&Wire1);
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  sensor.setDistanceMode(VL53L1X::Long); // long-mode
  sensor.setMeasurementTimingBudget(50000);
  sensor.startContinuous(10);

  delay(1000);
  xTaskCreatePinnedToCore(Core0a, "Core0a", 4096, NULL, 3, &thp[0], 0); 
}

void loop(){}

// マスターからのデータ受信イベント
void receiveEvent(int howMany) {
  while (Wire.available() > 0) {
    datafromEV3 = Wire.read();
    Serial.print(datafromEV3);
  }
  Serial.println();
}

// マスターからのデータ要求イベント
void requestEvent() {
  if (datafromEV3 == 5){}
  else {
      byte sendtoEV3[2] = {highByte(tof),lowByte(tof)};
    Wire.write(sendtoEV3,2);
  }
}

void Core0a(void *args) {//サブCPU(Core0)で実行するプログラム
  while (1) {
    // tof-------------------------------------------
    tof = sensor.read();
    // Serial.println(tof);
    if (sensor.timeoutOccurred()) { 
      Serial.println(" TIMEOUT");
      vlxReset(19);
    }
  
    // Serial.println();
  }
}

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
