#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define Wire1_SDA (33)
#define Wire1_SCL (32)
TaskHandle_t thp[1];

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire1);

void Core0a(void *args) {//サブCPU(Core0)で実行するプログラム
  while (1) {//ここで無限ループを作っておく
    /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);

    // float start_angle = event.orientation.x;
    // float target_angle = start_angle + 180;

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

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Wire1.begin(Wire1_SDA, Wire1_SCL);
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
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

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void){}
