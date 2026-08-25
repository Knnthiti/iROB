/*
  iROB_encoder.ino
  Encoder-count example for the left-front (LF) wheel.
*/

#include "Inc/iROB_Motor.h"

iROB_Motor iROB(100, 68, 27);

// Alias for the LF wheel so the example can call iROB.getCount(LF).
const motor_Wheel LF = _LF;

void setup() {
  Serial.begin(115200);
}

void loop() {
  int16_t countLF = iROB.getCount(LF);

  Serial.print("Count_LF: ");
  Serial.println(countLF);

  delay(100);
}
