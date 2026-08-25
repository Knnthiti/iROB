/*
  iROB_encoder.ino
  Encoder-count example for the left-front (LF) wheel.
*/

#include "Inc/iROB_Motor.h"

iROB_Motor iROB(100, 68, 27);

void setup() {
  Serial.begin(115200);
}

void loop() {
  iROB.Motor_DutyCycle_LF(4095);
  iROB.Motor_DutyCycle_LB(4095);
  iROB.Motor_DutyCycle_RF(4095);
  iROB.Motor_DutyCycle_RB(4095);

  Serial.print("Count_LF: ");
  Serial.print(iROB.getCount(_LF));
  Serial.print("| Count_LB: ");
  Serial.print(iROB.getCount(_LB));
  Serial.print("| Count_RF: ");
  Serial.print(iROB.getCount(_RF));
  Serial.print("| Count_RB: ");
  Serial.println(iROB.getCount(_RB));

  delay(10);
}
