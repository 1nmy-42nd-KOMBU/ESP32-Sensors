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

    int16_t start_angle = event.orientation.x;
    int16_t target_angle_range_min = start_angle + 170; // 目標角の範囲の最小値
    int16_t target_angle_range_max = start_angle + 180; // 目標角の範囲の最大値
    
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

    Serial.print(F("turn: "));

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
