#include <Arduino.h>

const int MicSw1 = 32;
const int MicSw2 = 33;

void setup()
{
    pinMode( MicSw1, INPUT_PULLUP );
    pinMode( MicSw2, INPUT_PULLUP );
    Serial.begin(9600);
}

void loop() {
    // スイッチの状態を表示
    Serial.print( digitalRead( MicSw2 ) );
    Serial.println( digitalRead( MicSw1 ) );
    delay(500);
}