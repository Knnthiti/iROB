/*
  iROB_PID.ino
  Closed-loop speed PID example for the left-front (LF) motor.
*/

#include "Inc/iROB_Motor.h"
#include "Inc/Inverse_Kinematics.h"

iROB_Motor iROB(100, 68, 27);
Kinematic _Kinematic(0.23f, 0.23f, 0.06f);

const float TARGET_VX = 0.0f;  // Forward velocity command in m/s
const float TARGET_VY = 0.0f;  // Strafe velocity command in m/s
const float TARGET_WZ = 0.0f;  // Rotation command in rad/s

unsigned long pastTime = 0;
float pidLF = 0;

void setup() {
  Serial.begin(115200);

  // PID gains and RPM limit for LF wheel.
  iROB.Setup_PID_Wheel(1.5f, 0.01f, 0.1f, 100, 300, _LF);
}

void loop() {
  // The RPM calculation assumes a 100 Hz update rate, so run this every 10 ms.
  if ((millis() - pastTime) > 10) {
    pastTime = millis();

    _Kinematic.Inverse_Kinematic(TARGET_VX, TARGET_VY, TARGET_WZ);

    // Convert LF wheel rad/s target to RPM and drive LF with PID feedback.
    pidLF = iROB.Motor_Speed_LF(iROB.getRad_s_to_RPM(_Kinematic.Wheel.w_LF), iROB.getRPM(_LF));

    Serial.print("PID_LF: ");
    Serial.print(pidLF);
    Serial.print(" | RPM_LF: ");
    Serial.println(iROB.Motor_feedback._RPM[_LF]);
  }
}
