#include "../Inc/iROB_Motor.h"

static void iROB_ledcWrite(uint8_t channel, uint32_t duty) {
  // ESP32 Arduino core 2.x writes by channel with ledcWrite().
  // ESP32 Arduino core 3.x keeps channel writes in ledcWriteChannel().
#if IROB_ESP32_CORE_V3_OR_NEWER
  ledcWriteChannel(channel, duty);
#else
  ledcWrite(channel, duty);
#endif
}

int16_t iROB_Motor::getCount(motor_Wheel _Wheel) {
  // Read the encoder that matches the requested wheel index.
  if (_Wheel == _LF) {
    Motor_feedback._Count[_Wheel] = (int16_t)enc_m1.getCount();
  } else if (_Wheel == _LB) {
    Motor_feedback._Count[_Wheel] = (int16_t)enc_m2.getCount();
  } else if (_Wheel == _RF) {
    Motor_feedback._Count[_Wheel] = (int16_t)enc_m3.getCount();
  } else if (_Wheel == _RB) {
    Motor_feedback._Count[_Wheel] = (int16_t)enc_m4.getCount();
  }

  return Motor_feedback._Count[_Wheel];
}

float iROB_Motor::getRPM(motor_Wheel _Wheel) {
  Present_Count[_Wheel] = (int16_t)getCount(_Wheel);

  // Convert encoder-count delta per control tick to wheel RPM:
  // count/tick -> count/second -> motor rev/min -> wheel rev/min.
  Motor_feedback._RPM[_Wheel] = (float)((int16_t)(Present_Count[_Wheel] - Past_Count[_Wheel]) * _Setup._freq);
  Motor_feedback._RPM[_Wheel] = (float)(((Motor_feedback._RPM[_Wheel] * 60.0f) / _Setup._CPR) / _Setup._Gear_Ratio);

  Past_Count[_Wheel] = Present_Count[_Wheel];

  return Motor_feedback._RPM[_Wheel];
}

void iROB_Motor::Motor_DutyCycle_LF(int16_t DutyCycle_LF) {
  // Limit to the 12-bit LEDC range, then drive IN1/IN2 for direction.
  if (DutyCycle_LF > 4095) {
    DutyCycle_LF = 4095;
  } else if (DutyCycle_LF < -4095) {
    DutyCycle_LF = -4095;
  }

  iROB_ledcWrite(0, (DutyCycle_LF > 0) ? DutyCycle_LF : 0);
  iROB_ledcWrite(1, (DutyCycle_LF >= 0) ? 0 : -DutyCycle_LF);
}

void iROB_Motor::Motor_DutyCycle_LB(int16_t DutyCycle_LB) {
  // Limit to the 12-bit LEDC range, then drive IN1/IN2 for direction.
  if (DutyCycle_LB > 4095) {
    DutyCycle_LB = 4095;
  } else if (DutyCycle_LB < -4095) {
    DutyCycle_LB = -4095;
  }

  iROB_ledcWrite(2, (DutyCycle_LB > 0) ? DutyCycle_LB : 0);
  iROB_ledcWrite(3, (DutyCycle_LB >= 0) ? 0 : -DutyCycle_LB);
}

void iROB_Motor::Motor_DutyCycle_RF(int16_t DutyCycle_RF) {
  // Limit to the 12-bit LEDC range, then drive IN1/IN2 for direction.
  if (DutyCycle_RF > 4095) {
    DutyCycle_RF = 4095;
  } else if (DutyCycle_RF < -4095) {
    DutyCycle_RF = -4095;
  }

  iROB_ledcWrite(4, (DutyCycle_RF > 0) ? DutyCycle_RF : 0);
  iROB_ledcWrite(5, (DutyCycle_RF >= 0) ? 0 : -DutyCycle_RF);
}

void iROB_Motor::Motor_DutyCycle_RB(int16_t DutyCycle_RB) {
  // Limit to the 12-bit LEDC range, then drive IN1/IN2 for direction.
  if (DutyCycle_RB > 4095) {
    DutyCycle_RB = 4095;
  } else if (DutyCycle_RB < -4095) {
    DutyCycle_RB = -4095;
  }

  iROB_ledcWrite(6, (DutyCycle_RB > 0) ? DutyCycle_RB : 0);
  iROB_ledcWrite(7, (DutyCycle_RB >= 0) ? 0 : -DutyCycle_RB);
}

void iROB_Motor::Setup_PID_Wheel(float Kp, float Ki, float Kd, float _min_speed, float _max_speed, motor_Wheel _Wheel) {
  // Store the PID constants for one wheel. The max value also limits output.
  Kp_Wheel[_Wheel] = Kp;
  Ki_Wheel[_Wheel] = Ki;
  Kd_Wheel[_Wheel] = Kd;

  min_speed[_Wheel] = _min_speed;
  max_speed[_Wheel] = _max_speed;
}

float iROB_Motor::PID_Speed(float _Setpoint, float RPM, motor_Wheel _Wheel) {
  Error_Speed[_Wheel] = _Setpoint - RPM;

  Proportional[_Wheel] = Error_Speed[_Wheel];
  Integnator[_Wheel] += Error_Speed[_Wheel];

  // Reset the integral term when the command is zero so the wheel stops cleanly.
  if (_Setpoint == 0) {
    Integnator[_Wheel] = 0;
  } else if (Integnator[_Wheel] > 25000) {
    Integnator[_Wheel] = 25000;
  } else if (Integnator[_Wheel] < -25000) {
    Integnator[_Wheel] = -25000;
  }

  Derivative[_Wheel] = Error_Speed[_Wheel] - Past_Error[_Wheel];
  Past_Error[_Wheel] = Error_Speed[_Wheel];

  Motor_feedback._PID[_Wheel] = (float)((Proportional[_Wheel] * Kp_Wheel[_Wheel]) + (Integnator[_Wheel] * Ki_Wheel[_Wheel]) + (Derivative[_Wheel] * Kd_Wheel[_Wheel]));

  // Clamp PID output before converting it to PWM duty cycle.
  if (Motor_feedback._PID[_Wheel] > max_speed[_Wheel]) {
    Motor_feedback._PID[_Wheel] = max_speed[_Wheel];
  } else if (Motor_feedback._PID[_Wheel] < -max_speed[_Wheel]) {
    Motor_feedback._PID[_Wheel] = -max_speed[_Wheel];
  }

  return Motor_feedback._PID[_Wheel];
}

float iROB_Motor::Motor_Speed_LF(int16_t RPM_INPUT, float RPM_LF) {
  // Limit target RPM, run PID, then scale the PID result to 12-bit PWM.
  if (RPM_INPUT > max_speed[0]) {
    RPM_INPUT = max_speed[0];
  } else if (RPM_INPUT < -max_speed[0]) {
    RPM_INPUT = -max_speed[0];
  }

  PID_Speed(RPM_INPUT, RPM_LF, _LF);

  Duty_Cycle[0] = (int16_t)((Motor_feedback._PID[0] / ((float)max_speed[0])) * 4095);
  Motor_DutyCycle_LF(Duty_Cycle[0]);

  return Motor_feedback._PID[0];
}

float iROB_Motor::Motor_Speed_LB(int16_t RPM_INPUT, float RPM_LB) {
  // Limit target RPM, run PID, then scale the PID result to 12-bit PWM.
  if (RPM_INPUT > max_speed[1]) {
    RPM_INPUT = max_speed[1];
  } else if (RPM_INPUT < -max_speed[1]) {
    RPM_INPUT = -max_speed[1];
  }

  PID_Speed(RPM_INPUT, RPM_LB, _LB);

  Duty_Cycle[1] = (int16_t)((Motor_feedback._PID[1] / ((float)max_speed[1])) * 4095);
  Motor_DutyCycle_LB(Duty_Cycle[1]);

  return Motor_feedback._PID[1];
}

float iROB_Motor::Motor_Speed_RF(int16_t RPM_INPUT, float RPM_RF) {
  // Limit target RPM, run PID, then scale the PID result to 12-bit PWM.
  if (RPM_INPUT > max_speed[2]) {
    RPM_INPUT = max_speed[2];
  } else if (RPM_INPUT < -max_speed[2]) {
    RPM_INPUT = -max_speed[2];
  }

  PID_Speed(RPM_INPUT, RPM_RF, _RF);

  Duty_Cycle[2] = (int16_t)((Motor_feedback._PID[2] / ((float)max_speed[2])) * 4095);
  Motor_DutyCycle_RF(Duty_Cycle[2]);

  return Motor_feedback._PID[2];
}

float iROB_Motor::Motor_Speed_RB(int16_t RPM_INPUT, float RPM_RB) {
  // Limit target RPM, run PID, then scale the PID result to 12-bit PWM.
  if (RPM_INPUT > max_speed[3]) {
    RPM_INPUT = max_speed[3];
  } else if (RPM_INPUT < -max_speed[3]) {
    RPM_INPUT = -max_speed[3];
  }

  PID_Speed(RPM_INPUT, RPM_RB, _RB);

  Duty_Cycle[3] = (int16_t)((Motor_feedback._PID[3] / ((float)max_speed[3])) * 4095);
  Motor_DutyCycle_RB(Duty_Cycle[3]);

  return Motor_feedback._PID[3];
}

float iROB_Motor::_map(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  return toLow + (toHigh - toLow) * ((value - fromLow) / (fromHigh - fromLow));
}

float iROB_Motor::getRPM_to_Rad_s(float RPM) {
  // 1 RPM = 2*pi/60 rad/s.
  float Rad_s = RPM * 0.10472f;
  return Rad_s;
}

float iROB_Motor::getRad_s_to_RPM(float Rad_s) {
  // 1 rad/s = 60/(2*pi) RPM.
  float RPM__ = Rad_s * 9.549297f;
  return RPM__;
}

// Ramp helpers convert between encoder count and wheel angle.
float _max_Count;

float Count_to_degree(int32_t Count) {
  return (Count * (360.0f / _max_Count));
}

float Degree_to_Count(float Degree) {
  return (Degree * (_max_Count / 360.0f));
}

void iROB_Motor::Setup_Ramp_Count(float Kp_Count, float Ki_Count, float Kd_Count, float max_Count, float DutyCycle_MAX) {
  _Kp_degree = Kp_Count;
  _Ki_degree = Ki_Count;
  _Kd_degree = Kd_Count;
  _max_Count = max_Count;
  _DutyCycle_MAX = DutyCycle_MAX;
}

int16_t iROB_Motor::Ramp_Count(float Set_degree, float degree) {
  // PID position ramp in degree space, returned as a motor duty cycle.
  Error_degree = Set_degree - degree;
  Proportiona_degree = Error_degree;
  Integnator_degree += Error_degree;
  Derivative_degree = Error_degree - Past_Error_degree;

  Past_Error_degree = Error_degree;

  _V_degree = (Proportiona_degree * _Kp_degree) + (Integnator_degree * _Ki_degree) + (Derivative_degree * _Kd_degree);

  _DutyCycle = Degree_to_Count(_V_degree) * (_DutyCycle_MAX / _max_Count);
  return _DutyCycle;
}
