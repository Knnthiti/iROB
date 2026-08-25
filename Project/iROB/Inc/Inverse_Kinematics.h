#ifndef INVERSE_KINEMATICS_H_
#define INVERSE_KINEMATICS_H_

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

// Inverse kinematics for a four-wheel mecanum robot.
// Reference: https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html
class Kinematic {
public:
  float Lx_;            // Half robot length from center to wheel, in meters
  float Ly_;            // Half robot width from center to wheel, in meters
  float Radius_wheel_;  // Wheel radius, in meters

  float _r = 0;       // Joystick translation magnitude
  float _Rad_s = 0;   // Translation direction after heading compensation
  float __Vx = 0;     // Robot-frame X velocity after heading lock
  float __Vy = 0;     // Robot-frame Y velocity after heading lock

  typedef struct {
    float w_LF;  // Left front wheel angular velocity, rad/s
    float w_LB;  // Left back wheel angular velocity, rad/s
    float w_RF;  // Right front wheel angular velocity, rad/s
    float w_RB;  // Right back wheel angular velocity, rad/s
  } W_rad;

  W_rad Wheel;

  Kinematic() {
    // Empty constructor for sketches that assign geometry later.
  }

  Kinematic(float Lx, float Ly, float Radius_wheel) {
    Lx_ = Lx;
    Ly_ = Ly;
    Radius_wheel_ = Radius_wheel;
  }

  // Convert robot velocity command (Vx, Vy, wz) to wheel angular velocity.
  void Inverse_Kinematic(float Vx, float Vy, float wz);

  // Same kinematic conversion, but rotates Vx/Vy by heading for field lock.
  void Inverse_Kinematic_Lock_Direction(float Vx, float Vy, float wz, float head_ing);

  // Wheel-speed getters in rad/s.
  float get_w_LF();
  float get_w_LB();
  float get_w_RF();
  float get_w_RB();
};

#endif
