/*
  iROB_motor.ino
  Direct duty-cycle example for all four motors.
*/

#include "Inc/iROB_Motor.h"

iROB_Motor iROB(100, 68, 27);

void setup() {
  Serial.begin(115200);

  // Stop all motors once during startup.
  iROB.Motor_DutyCycle_LF(0);
  iROB.Motor_DutyCycle_LB(0);
  iROB.Motor_DutyCycle_RF(0);
  iROB.Motor_DutyCycle_RB(0);
}

void loop() {
  // Test all motors in reverse at full duty cycle. Change to 0 to stop.
  iROB.Motor_DutyCycle_LF(4095);
  iROB.Motor_DutyCycle_LB(-4095);
  iROB.Motor_DutyCycle_RF(2095);
  iROB.Motor_DutyCycle_RB(-2095);
  delay(10);
}
