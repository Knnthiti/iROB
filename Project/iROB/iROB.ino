/*
  iROB.ino - Run All
  Receives joystick data by ESP-NOW, converts it to mecanum wheel speeds,
  then drives all four motors with PID speed control.
*/

#include <string.h>

#if defined(__has_include)
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#endif

#include "Inc/iROB_Motor.h"
#include "Inc/Inverse_Kinematics.h"
#include "Inc/espnow_ROBOT.h"

iROB_Motor iROB(100, 68, 27);
Kinematic _Kinematic(0.23f, 0.23f, 0.06f);
ESPNOW_ROBOT ROBOT;

float Vx = 0;
float Vy = 0;
float Vz = 0;
float Rad = -0.7853f;
float PID[4] = { 0, 0, 0, 0 };

unsigned long Past_time = 0;
volatile bool newData = false;
volatile unsigned long Uart_data_time = 0;

typedef struct __attribute__((packed)) {
  uint8_t Header[2];

  union {
    uint8_t moveBtnByte;
    struct {
      uint8_t move1 : 1;
      uint8_t move2 : 1;
      uint8_t move3 : 1;
      uint8_t move4 : 1;
      uint8_t res1 : 2;
      uint8_t set1 : 1;
      uint8_t set2 : 1;
    } moveBtnBit;
  };

  union {
    uint8_t attackBtnByte;
    struct {
      uint8_t attack1 : 1;
      uint8_t attack2 : 1;
      uint8_t attack3 : 1;
      uint8_t attack4 : 1;
      uint8_t res1 : 4;
    } attackBtnBit;
  };

  int8_t stickValue[4];  // joyL_X, joyL_Y, joyR_X, joyR_Y
} Receive_ESPNOW;

Receive_ESPNOW Data;

#if ESPNOW_ROBOT_ESP32_CORE_V3_OR_NEWER
void OnDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len) {
  (void)recvInfo;
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  (void)mac;
#endif
  // Copy only the bytes that fit the receive structure.
  const size_t copyLength = (len < (int)sizeof(Data)) ? len : sizeof(Data);
  memset(&Data, 0, sizeof(Data));
  memcpy(&Data, incomingData, copyLength);

  newData = true;
  Uart_data_time = millis();
}

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);

  // PID gains and RPM limits for each wheel.
  iROB.Setup_PID_Wheel(1.5f, 0.01f, 0.1f, 100, 300, _LF);
  iROB.Setup_PID_Wheel(1.5f, 0.01f, 0.1f, 100, 300, _LB);
  iROB.Setup_PID_Wheel(1.5f, 0.01f, 0.1f, 100, 300, _RF);
  iROB.Setup_PID_Wheel(1.5f, 0.01f, 0.1f, 100, 300, _RB);

  ROBOT.Setup_receive_ESPNOW();

  // Register callback function for incoming ESP-NOW packets.
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Failsafe: stop all motors if no fresh controller packet arrives.
  if ((newData == false) && (millis() - Uart_data_time > 500)) {
    digitalWrite(2, LOW);

    iROB.Motor_DutyCycle_LF(0);
    iROB.Motor_DutyCycle_LB(0);
    iROB.Motor_DutyCycle_RF(0);
    iROB.Motor_DutyCycle_RB(0);

    memset(&Data, 0, sizeof(Data));

    Vx = 0;
    Vy = 0;
    Vz = 0;
    Rad = 0;
  } else if (newData == true) {
    newData = false;
    digitalWrite(2, HIGH);
  }

  // The RPM calculation assumes a 100 Hz update rate, so run this every 10 ms.
  if ((millis() - Past_time) > 10) {
    Past_time = millis();

    // Convert joystick values (-100..100) to robot velocity commands.
    Vx = iROB._map(Data.stickValue[0], 100.0f, -100.0f, -2.5f, 2.5f);
    Vy = iROB._map(Data.stickValue[1], 100.0f, -100.0f, -2.5f, 2.5f);
    Vz = iROB._map(Data.stickValue[3], 100.0f, -100.0f, -3.0f, 3.0f);

    // Calculate wheel angular velocity, then apply heading lock.
    _Kinematic.Inverse_Kinematic(Vx, Vy, Vz);
    _Kinematic.Inverse_Kinematic_Lock_Direction(Vx, Vy, Vz, Rad);

    // Convert wheel rad/s targets to RPM and close the loop with encoder RPM.
    PID[0] = iROB.Motor_Speed_LF(iROB.getRad_s_to_RPM(_Kinematic.Wheel.w_LF), iROB.getRPM(_LF));
    PID[1] = iROB.Motor_Speed_LB(iROB.getRad_s_to_RPM(_Kinematic.Wheel.w_LB), iROB.getRPM(_LB));
    PID[2] = iROB.Motor_Speed_RF(iROB.getRad_s_to_RPM(_Kinematic.Wheel.w_RF), iROB.getRPM(_RF));
    PID[3] = iROB.Motor_Speed_RB(iROB.getRad_s_to_RPM(_Kinematic.Wheel.w_RB), iROB.getRPM(_RB));
  }
}
