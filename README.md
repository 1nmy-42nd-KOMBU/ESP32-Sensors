# ESP32-Arduino

## 足跡  
EV3とESP32を直接I2Cで接続しようとする  
↓  
失敗(ESP32のRequestEventがそもそも呼ばれん)  
↓  
Arduinoを介してUARTで通信しようとする  
↓  
失敗(ArduinoのSoftwareSieialが安定しない)  

## 予定
* SPI通信なら安定するかも
* PybricksでUART通信してみる
