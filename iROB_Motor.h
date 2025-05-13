#include <ESP32Encoder.h>
#include <esp32-hal-ledc.h>
#include <Arduino.h>

class iROB_Motor {
public:
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

  enum motor_Wheel {
    _LF = 0,
    _LB = 1,
    _RF = 2,
    _RB = 3
  } _Wheel;

  ESP32Encoder enc_m1;
  ESP32Encoder enc_m2;
  ESP32Encoder enc_m3;
  ESP32Encoder enc_m4;

  typedef struct {
    int16_t _Count[4];  //LF ,LB ,RF ,RB
    float _RPM[4];      //LF ,LB ,RF ,RB
    float _PID[4];      //LF ,LB ,RF ,RB
  } _Motor;
  _Motor Motor_feedback;

  typedef struct {
    uint16_t _freq;
    uint16_t _CPR;
    uint16_t _Gear_Ratio;
  } Setup_Motor;
  Setup_Motor _Setup = {
    ._freq = 100,
    ._CPR = 68,
    ._Gear_Ratio = 27
  };

  iROB_Motor() {
    //
  }

  iROB_Motor(uint16_t freq, uint16_t CPR, uint16_t Gear_Ratio) {
    _Setup._freq = freq;
    _Setup._CPR = CPR;
    _Setup._Gear_Ratio = Gear_Ratio;

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

    pinMode(M_LF_IN1, OUTPUT);
    pinMode(M_LF_IN2, OUTPUT);

    pinMode(M_LB_IN1, OUTPUT);
    pinMode(M_LB_IN2, OUTPUT);

    pinMode(M_RF_IN1, OUTPUT);
    pinMode(M_RF_IN2, OUTPUT);

    pinMode(M_RB_IN1, OUTPUT);
    pinMode(M_RB_IN2, OUTPUT);

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
  }

  int16_t getCount(motor_Wheel _Wheel);

  int16_t Present_Count[4] = { 0, 0, 0, 0 };  //LF ,LB ,RF ,RB
  int16_t Past_Count[4] = { 0, 0, 0, 0 };     //LF ,LB ,RF ,RB
  float getRPM(motor_Wheel _Wheel);

  void Motor_DutyCycle_LF(int16_t DutyCycle_LF);
  void Motor_DutyCycle_LB(int16_t DutyCycle_LB);
  void Motor_DutyCycle_RF(int16_t DutyCycle_RF);
  void Motor_DutyCycle_RB(int16_t DutyCycle_RB);

  float Kp_Wheel[4] = { 0, 0, 0, 0 };  //LF ,LB ,RF ,RB
  float Ki_Wheel[4] = { 0, 0, 0, 0 };  //LF ,LB ,RF ,RB
  float Kd_Wheel[4] = { 0, 0, 0, 0 };  //LF ,LB ,RF ,RB

  float min_speed[4] = { 0, 0, 0, 0 };  //LF ,LB ,RF ,RB
  float max_speed[4] = { 0, 0, 0, 0 };  //LF ,LB ,RF ,RB
  void Setup_PID_Wheel(float Kp, float Ki, float Kd, float _min_speed, float _max_speed, motor_Wheel _Wheel);

  float Error_Speed[4] = { 0, 0, 0, 0 };  //LF ,LB ,RF ,RB

  float Proportional[4] = { 0, 0, 0, 0 };  //LF ,LB ,RF ,RB
  float Integnator[4] = { 0, 0, 0, 0 };    //LF ,LB ,RF ,RB
  float Derivative[4] = { 0, 0, 0, 0 };    //LF ,LB ,RF ,RB

  float Past_Error[4] = { 0, 0, 0, 0 };  //LF ,LB ,RF ,RB
  float PID_Speed(float _Setpoint, float RPM, motor_Wheel _Wheel);


  int16_t Duty_Cycle[4] = { 0, 0, 0, 0 };  //LF ,LB ,RF ,RB
  float Motor_Speed_LF(int16_t RPM_INPUT, float RPM_LF);
  float Motor_Speed_LB(int16_t RPM_INPUT, float RPM_LB);
  float Motor_Speed_RF(int16_t RPM_INPUT, float RPM_RF);
  float Motor_Speed_RB(int16_t RPM_INPUT, float RPM_RB);

  float _map(float value, float fromLow, float fromHigh, float toLow, float toHigh);

  float getRPM_to_Rad_s(float RPM);
  float getRad_s_to_RPM(float Rad_s);
};