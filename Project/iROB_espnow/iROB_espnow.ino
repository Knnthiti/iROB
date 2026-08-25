/*
  iROB_espnow.ino
  ESP-NOW receive example. Registers OnDataRecv and prints joystick data.
*/

#include <string.h>

#if defined(__has_include)
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#endif

#include "Inc/espnow_ROBOT.h"

ESPNOW_ROBOT ROBOT;

volatile bool newData = false;

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
}

void setup() {
  Serial.begin(115200);

  ROBOT.Setup_receive_ESPNOW();

  // Register callback function for incoming ESP-NOW packets.
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  if (newData == true) {
    newData = false;

    Serial.print("joyL_X: ");
    Serial.print(Data.stickValue[0]);
    Serial.print(" | joyL_Y: ");
    Serial.print(Data.stickValue[1]);
    Serial.print(" | joyR_X: ");
    Serial.print(Data.stickValue[2]);
    Serial.print(" | joyR_Y: ");
    Serial.println(Data.stickValue[3]);
  }
}
