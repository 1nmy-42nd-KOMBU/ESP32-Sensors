#include <CheapStepper.h>

CheapStepper stepper(12,14,27,26); // IN1,IN2,IN3,IN4　①
boolean clockwise = true;

void setup() {
    stepper.setRpm(-22); //②
}

void loop() {
    // モーターの全Steps = 4096
    clockwise = true; //③
    stepper.moveTo (clockwise, 0); //④
    delay(1000);
    stepper.moveDegrees (clockwise, 360); //⑤
    delay(1000);
    stepper.moveDegrees (clockwise, 90); //⑥
    delay(1000);
    stepper.moveDegrees (clockwise, 90);
    delay(1000);
    stepper.moveDegrees (clockwise, 90);
    delay(1000);
    stepper.moveDegrees (clockwise, 90);
    delay(3000);
}
