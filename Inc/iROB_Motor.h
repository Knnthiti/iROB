#ifndef IROB_MOTOR_H_
#define IROB_MOTOR_H_

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <esp32-hal-ledc.h>

#if defined(__has_include)
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#endif

#if defined(ESP_ARDUINO_VERSION) && defined(ESP_ARDUINO_VERSION_VAL) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
#define IROB_ESP32_CORE_V3_OR_NEWER 1
#else
#define IROB_ESP32_CORE_V3_OR_NEWER 0
#endif

// Wheel index used by every motor feedback/PID array.
enum motor_Wheel {
  _LF = 0,  // Left front wheel
  _LB = 1,  // Left back wheel
  _RF = 2,  // Right front wheel
  _RB = 3   // Right back wheel
};

// Shared ramp conversion value. Set it with Setup_Ramp_Count() before using
// Count_to_degree() or Degree_to_Count().
extern float _max_Count;
float Count_to_degree(int32_t Count);
float Degree_to_Count(float Degree);

class iROB_Motor {
public:
  // ESP32 pin map for the motor driver inputs and quadrature encoders.
  enum Pin_On_brad {
    ENC_LF_B = 36,
    ENC_LF_A = 39,
    ENC_LB_B = 34,
    ENC_LB_A = 35,
    M_LF_IN1 = 32,
    M_LF_IN2 = 33,
    M_LB_IN1 = 25,
    M_LB_IN2 = 26,
    ENC_RB_A = 27,
    ENC_RB_B = 13,
    M_RF_IN2 = 21,
    M_RF_IN1 = 19,
    ENC_RF_A = 18,
    ENC_RF_B = 17,
    M_RB_IN2 = 16,
    M_RB_IN1 = 4
  } iROB_Pin;

  // Encoder objects are mapped in the same order as motor_Wheel.
  ESP32Encoder enc_m1;  // LF
  ESP32Encoder enc_m2;  // LB
  ESP32Encoder enc_m3;  // RF
  ESP32Encoder enc_m4;  // RB

  typedef struct {
    int16_t _Count[4];  // Encoder count: LF, LB, RF, RB
    float _RPM[4];      // Wheel speed feedback in RPM: LF, LB, RF, RB
    float _PID[4];      // Latest PID output: LF, LB, RF, RB
  } _Motor;
  _Motor Motor_feedback;

  typedef struct {
    uint16_t _freq;        // Control/feedback update rate in Hz
    uint16_t _CPR;         // Encoder count per motor revolution
    uint16_t _Gear_Ratio;  // Gear ratio between motor and wheel
  } Setup_Motor;
  Setup_Motor _Setup = {
    ._freq = 100,
    ._CPR = 68,
    ._Gear_Ratio = 27
  };

  iROB_Motor() {
    // Empty constructor for sketches that configure pins later.
  }

  iROB_Motor(uint16_t freq, uint16_t CPR, uint16_t Gear_Ratio) {
    _Setup._freq = freq;
    _Setup._CPR = CPR;
    _Setup._Gear_Ratio = Gear_Ratio;

    // Configure encoder pins before attaching each full-quadrature encoder.
    pinMode(ENC_LF_A, INPUT);
    pinMode(ENC_LF_B, INPUT);

    pinMode(ENC_LB_A, INPUT);
    pinMode(ENC_LB_B, INPUT);

    pinMode(ENC_RF_A, INPUT);
    pinMode(ENC_RF_B, INPUT);

    pinMode(ENC_RB_A, INPUT);
    pinMode(ENC_RB_B, INPUT);

    enc_m1.attachFullQuad(ENC_LF_A, ENC_LF_B);
    enc_m2.attachFullQuad(ENC_LB_A, ENC_LB_B);
    enc_m3.attachFullQuad(ENC_RF_A, ENC_RF_B);
    enc_m4.attachFullQuad(ENC_RB_A, ENC_RB_B);

    // Configure motor driver direction/PWM pins.
    pinMode(M_LF_IN1, OUTPUT);
    pinMode(M_LF_IN2, OUTPUT);

    pinMode(M_LB_IN1, OUTPUT);
    pinMode(M_LB_IN2, OUTPUT);

    pinMode(M_RF_IN1, OUTPUT);
    pinMode(M_RF_IN2, OUTPUT);

    pinMode(M_RB_IN1, OUTPUT);
    pinMode(M_RB_IN2, OUTPUT);

    // ESP32 LEDC channels use 12-bit PWM, so duty cycle range is -4095..4095.
#if IROB_ESP32_CORE_V3_OR_NEWER
    ledcAttachChannel(M_LF_IN1, 12000, 12, 0);
    ledcAttachChannel(M_LF_IN2, 12000, 12, 1);
    ledcAttachChannel(M_LB_IN1, 12000, 12, 2);
    ledcAttachChannel(M_LB_IN2, 12000, 12, 3);
    ledcAttachChannel(M_RF_IN1, 12000, 12, 4);
    ledcAttachChannel(M_RF_IN2, 12000, 12, 5);
    ledcAttachChannel(M_RB_IN1, 12000, 12, 6);
    ledcAttachChannel(M_RB_IN2, 12000, 12, 7);
#else
    ledcSetup(0, 12000, 12);
    ledcSetup(1, 12000, 12);
    ledcSetup(2, 12000, 12);
    ledcSetup(3, 12000, 12);
    ledcSetup(4, 12000, 12);
    ledcSetup(5, 12000, 12);
    ledcSetup(6, 12000, 12);
    ledcSetup(7, 12000, 12);

    ledcAttachPin(M_LF_IN1, 0);
    ledcAttachPin(M_LF_IN2, 1);
    ledcAttachPin(M_LB_IN1, 2);
    ledcAttachPin(M_LB_IN2, 3);
    ledcAttachPin(M_RF_IN1, 4);
    ledcAttachPin(M_RF_IN2, 5);
    ledcAttachPin(M_RB_IN1, 6);
    ledcAttachPin(M_RB_IN2, 7);
#endif
  }

  // Read raw encoder count for one wheel.
  int16_t getCount(motor_Wheel _Wheel);

  int16_t Present_Count[4] = { 0, 0, 0, 0 };  // Current encoder count
  int16_t Past_Count[4] = { 0, 0, 0, 0 };     // Previous encoder count

  // Calculate wheel RPM from encoder delta and configured update frequency.
  float getRPM(motor_Wheel _Wheel);

  // Direct duty-cycle control. Positive and negative values set wheel direction.
  void Motor_DutyCycle_LF(int16_t DutyCycle_LF);
  void Motor_DutyCycle_LB(int16_t DutyCycle_LB);
  void Motor_DutyCycle_RF(int16_t DutyCycle_RF);
  void Motor_DutyCycle_RB(int16_t DutyCycle_RB);

  float Kp_Wheel[4] = { 0, 0, 0, 0 };  // PID Kp for LF, LB, RF, RB
  float Ki_Wheel[4] = { 0, 0, 0, 0 };  // PID Ki for LF, LB, RF, RB
  float Kd_Wheel[4] = { 0, 0, 0, 0 };  // PID Kd for LF, LB, RF, RB

  float min_speed[4] = { 0, 0, 0, 0 };  // Reserved minimum speed value
  float max_speed[4] = { 0, 0, 0, 0 };  // PID output/RPM limit

  // Set PID constants and output limit for one wheel.
  void Setup_PID_Wheel(float Kp, float Ki, float Kd, float _min_speed, float _max_speed, motor_Wheel _Wheel);

  float Error_Speed[4] = { 0, 0, 0, 0 };  // PID error

  float Proportional[4] = { 0, 0, 0, 0 };  // P term
  float Integnator[4] = { 0, 0, 0, 0 };    // I term accumulator
  float Derivative[4] = { 0, 0, 0, 0 };    // D term

  float Past_Error[4] = { 0, 0, 0, 0 };  // Previous PID error

  // Calculate PID output in RPM units before converting to duty cycle.
  float PID_Speed(float _Setpoint, float RPM, motor_Wheel _Wheel);

  int16_t Duty_Cycle[4] = { 0, 0, 0, 0 };  // PWM duty sent to each motor

  // Closed-loop speed control for each wheel.
  float Motor_Speed_LF(int16_t RPM_INPUT, float RPM_LF);
  float Motor_Speed_LB(int16_t RPM_INPUT, float RPM_LB);
  float Motor_Speed_RF(int16_t RPM_INPUT, float RPM_RF);
  float Motor_Speed_RB(int16_t RPM_INPUT, float RPM_RB);

  // Floating-point map helper, useful for joystick conversion.
  float _map(float value, float fromLow, float fromHigh, float toLow, float toHigh);

  // Unit converters used between kinematics (rad/s) and motor PID (RPM).
  float getRPM_to_Rad_s(float RPM);
  float getRad_s_to_RPM(float Rad_s);

  // Ramp/PID position helper values.
  float _V_degree = 0.0f;
  int16_t _DutyCycle = 0;

  float _Kp_degree;
  float _Ki_degree;
  float _Kd_degree;

  float _DutyCycle_MAX;

  float Error_degree = 0.0f;
  float Proportiona_degree = 0.0f;
  float Integnator_degree = 0.0f;
  float Derivative_degree = 0.0f;
  float Past_Error_degree = 0.0f;

  // Configure and run a simple position ramp based on encoder counts.
  void Setup_Ramp_Count(float Kp_Count, float Ki_Count, float Kd_Count, float max_Count, float DutyCycle_MAX);
  int16_t Ramp_Count(float Set_degree, float degree);
};

#endif
