#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <VL53L1X.h>

VL53L1X sensor;
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define Wire1_SDA (33)
#define Wire1_SCL (32)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire1);
TaskHandle_t thp[1];

void setup()
{
  Serial.begin(115200);
  Wire1.begin(Wire1_SDA, Wire1_SCL);
  Wire.begin();
  Wire.setClock(100000); // use 400 kHz I2C

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  sensor.setDistanceMode(VL53L1X::Long);
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
  xTaskCreatePinnedToCore(Core0a, "Core0a", 4096, NULL, 3, &thp[0], 0); 
}

void loop()
{
  Serial.print(sensor.read());
  if (sensor.timeoutOccurred()) { 
    Serial.print(" TIMEOUT");
    vlxReset(19);
  }

  Serial.println();
}

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

void Core0a(void *args) {//サブCPU(Core0)で実行するプログラム
  while (1) {//ここで無限ループを作っておく
    /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);

    /* The processing sketch expects data as roll, pitch, heading */
    Serial.print(F("Orientation: "));
    Serial.print((float)event.orientation.x);
    Serial.print(F(" "));
    Serial.print((float)event.orientation.y);
    Serial.print(F(" "));
    Serial.print((float)event.orientation.z);
    Serial.println(F(""));

    // /* Also send calibration data for each sensor. */
    // uint8_t sys, gyro, accel, mag = 0;
    // bno.getCalibration(&sys, &gyro, &accel, &mag);
    // Serial.print(F("Calibration: "));
    // Serial.print(sys, DEC);
    // Serial.print(F(" "));
    // Serial.print(gyro, DEC);
    // Serial.print(F(" "));
    // Serial.print(accel, DEC);
    // Serial.print(F(" "));
    // Serial.println(mag, DEC);

    delay(BNO055_SAMPLERATE_DELAY_MS);
  }
}