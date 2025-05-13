#ifndef INVERSE_KINEMATICS_H_
#define INVERSE_KINEMATICS_H_

#include "stdint.h"
#include "stdlib.h"
#include "math.h"


//https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html
//inverse kinematic equations mecanum wheel

class Kinematic {
public:
  float Lx_;
  float Ly_;
  float Radius_wheel_;

  float _r = 0;
  float _Rad_s = 0;
  float __Vx = 0;
  float __Vy = 0;

  typedef struct {
    float w_LF;
    float w_LB;
    float w_RF;
    float w_RB;
  }W_rad;

  W_rad Wheel;

  Kinematic() {
    //
  }

  Kinematic(float Lx, float Ly, float Radius_wheel) {
    Lx_ = Lx;
    Ly_ = Ly;
    Radius_wheel_ = Radius_wheel;
  }
  void Inverse_Kinematic(float Vx, float Vy, float wz);
  void Inverse_Kinematic_Lock_Direction(float Vx, float Vy, float wz, float head_ing);

  float get_w_LF();
  float get_w_LB();
  float get_w_RF();
  float get_w_RB();
};
#endif