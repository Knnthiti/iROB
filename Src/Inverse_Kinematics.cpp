#include "../Inc/Inverse_Kinematics.h"

void Kinematic::Inverse_Kinematic(float Vx, float Vy, float wz) {
  // Mecanum inverse kinematics:
  // Vx is forward/back, Vy is left/right strafe, and wz is yaw rate.
  Wheel.w_LF = (Vx - Vy - (Lx_ + Ly_) * wz) / Radius_wheel_;
  Wheel.w_RF = -(Vx + Vy + (Lx_ + Ly_) * wz) / Radius_wheel_;
  Wheel.w_LB = (Vx + Vy - (Lx_ + Ly_) * wz) / Radius_wheel_;
  Wheel.w_RB = -(Vx - Vy + (Lx_ + Ly_) * wz) / Radius_wheel_;
}

void Kinematic::Inverse_Kinematic_Lock_Direction(float Vx, float Vy, float wz, float head_ing) {
  // Rotate the translation vector by current heading so movement remains
  // locked to the field/world direction instead of robot direction.
  _r = (float)(sqrt((Vx * Vx) + (Vy * Vy)));
  _Rad_s = (float)(atan2(Vy, Vx)) - head_ing;

  __Vx = (_r * cos(_Rad_s));
  __Vy = (_r * sin(_Rad_s));

  Wheel.w_LF = (__Vx - __Vy - (Lx_ + Ly_) * wz) / Radius_wheel_;
  Wheel.w_RF = -(__Vx + __Vy + (Lx_ + Ly_) * wz) / Radius_wheel_;
  Wheel.w_LB = (__Vx + __Vy - (Lx_ + Ly_) * wz) / Radius_wheel_;
  Wheel.w_RB = -(__Vx - __Vy + (Lx_ + Ly_) * wz) / Radius_wheel_;
}

float Kinematic::get_w_LF() {
  return Wheel.w_LF;
}

float Kinematic::get_w_LB() {
  return Wheel.w_LB;
}

float Kinematic::get_w_RF() {
  return Wheel.w_RF;
}

float Kinematic::get_w_RB() {
  return Wheel.w_RB;
}
