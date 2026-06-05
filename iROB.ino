#include "iROB_Motor.h"
iROB_Motor iROB(100, 68, 27);
float Vx = 0;
float Vy = 0;
float Vz = 0;
float Rad = -0.7853;

float PID[4] = { 0 };

#include "Inverse_Kinematics.h"
Kinematic _Kinematic(0.23f, 0.23f, 0.06f);

long Past_time = 0;

#include <espnow_ROBOT.h>
ESPNOW_ROBOT ROBOT;

bool newData = 0;
long Uart_data_time = 0;

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

  int8_t stickValue[4];  //joyL_X,joyL_Y ,joyR_X,joyR_Y

} Receive_ESPNOW;

Receive_ESPNOW Data;

// #include "MPU6050_E12.h"

// #define I2C_SCL_Pin 22
// #define I2C_SDA_Pin 23
// #define freq_MPU6050 100

// MPU6050 _MPU6050(I2C_SDA_Pin, I2C_SCL_Pin, freq_MPU6050);

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&Data, incomingData, sizeof(Data));

  newData = 1;
  Uart_data_time = millis();
}


void setup() {
  iROB.Setup_PID_Wheel(1.5, 0.01, 0.1, 100, 300, _LF);
  iROB.Setup_PID_Wheel(1.5, 0.01, 0.1, 100, 300, _LB);
  iROB.Setup_PID_Wheel(1.5, 0.01, 0.1, 100, 300, _RF);
  iROB.Setup_PID_Wheel(1.5, 0.01, 0.1, 100, 300, _RB);

  Serial.begin(115200);
  pinMode(2, OUTPUT);

  // _MPU6050.MPU_init();
  // _MPU6050.gyro_calib();

  ROBOT.Setup_receive_ESPNOW();
  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  if ((newData == 0) && (millis() - Uart_data_time > 500)) {
    digitalWrite(2, 0);

    iROB.Motor_DutyCycle_LF(0);
    iROB.Motor_DutyCycle_LB(0);
    iROB.Motor_DutyCycle_RF(0);
    iROB.Motor_DutyCycle_RB(0);

    memset(&Data, 0, sizeof(Data));

    Vx = 0;
    Vy = 0;
    Vz = 0;
    Rad = 0;
  } else if (newData == 1) {
    newData = 0;
    digitalWrite(2, 1);
  }

  if ((millis() - Past_time) > 10) {
    Past_time = millis();

    Vx = iROB._map(Data.stickValue[0], 100.0f, -100.0f, -2.5f, 2.5f);
    Vy = iROB._map(Data.stickValue[1], 100.0f, -100.0f, -2.5f, 2.5f);
    Vz = iROB._map(Data.stickValue[3], 100.0f, -100.0f, -3.0f, 3.0f);

    // _MPU6050.Radian();

    // if(Data.attackBtnBit.attack4 == 1){
    //   _MPU6050.Angular.Rad_z = 0;
    // }

    _Kinematic.Inverse_Kinematic(Vx, Vy, Vz);
    _Kinematic.Inverse_Kinematic_Lock_Direction(Vx, Vy, Vz, Rad);

    PID[0] = iROB.Motor_Speed_LF(iROB.getRad_s_to_RPM(_Kinematic.Wheel.w_LF), iROB.getRPM(_LF));
    PID[1] = iROB.Motor_Speed_LB(iROB.getRad_s_to_RPM(_Kinematic.Wheel.w_LB), iROB.getRPM(_LB));
    PID[2] = iROB.Motor_Speed_RF(iROB.getRad_s_to_RPM(_Kinematic.Wheel.w_RF), iROB.getRPM(_RF));
    PID[3] = iROB.Motor_Speed_RB(iROB.getRad_s_to_RPM(_Kinematic.Wheel.w_RB), iROB.getRPM(_RB));

    // PID[0] = iROB.Motor_Speed_LF(250, iROB.getRPM(_LF));
    // PID[1] = iROB.Motor_Speed_LB(250, iROB.getRPM(_LB));
    // PID[2] = iROB.Motor_Speed_RF(250, iROB.getRPM(_RF));
    // PID[3] = iROB.Motor_Speed_RB(250, iROB.getRPM(_RB));
    // /////////////////////////////////////////Test motor////////////////////////////////////////////////
    // iROB.Motor_DutyCycle_LF(4000);
    // iROB.Motor_DutyCycle_LB(-4000);
    // iROB.Motor_DutyCycle_RF(2000);
    // iROB.Motor_DutyCycle_RB(-2000);

    // iROB.getRPM(iROB._LF);
    // iROB.getRPM(iROB._LB);
    // iROB.getRPM(iROB._RF);
    // iROB.getRPM(iROB._RB);

    /////////////////////////////////////////////////////////////////////////////////////////
    // Serial.print(PID[0]);
    // Serial.print(" , ");
    // Serial.print(PID[1]);
    // Serial.print(" , ");
    // Serial.print(PID[2]);
    // Serial.print(" , ");
    // Serial.print(PID[3]);
    // Serial.print("  |  ");

    // Serial.print(iROB.Motor_feedback._RPM[0]);
    // Serial.print(" , ");
    // Serial.print(iROB.Motor_feedback._RPM[1]);
    // Serial.print(" , ");
    // Serial.print(iROB.Motor_feedback._RPM[2]);
    // Serial.print(" , ");
    // Serial.println(iROB.Motor_feedback._RPM[3]);

    // Serial.println(iROB.getCount(iROB._LF));

    // iROB.Motor_DutyCycle_LF(iROB.Ramp_Count(90, Count_to_degree(iROB.getCount(_LF))));

    // Serial.println(iROB.Error_degree);

    // Serial.println(_MPU6050.Angular.Rad_z);

    // uart_write_bytes(UART_PORT, (uint8_t *)&Data, sizeof(Data));
  }
}
