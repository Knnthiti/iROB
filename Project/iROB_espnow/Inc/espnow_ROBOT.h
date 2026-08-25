#ifndef ESPNOW_ROBOT_H_
#define ESPNOW_ROBOT_H_

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <stdint.h>
#include <string.h>

#if defined(__has_include)
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#endif

#if defined(ESP_ARDUINO_VERSION) && defined(ESP_ARDUINO_VERSION_VAL) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
#define ESPNOW_ROBOT_ESP32_CORE_V3_OR_NEWER 1
#else
#define ESPNOW_ROBOT_ESP32_CORE_V3_OR_NEWER 0
#endif

// Uncomment this line when you want ESP-NOW debug messages on Serial.
// #define ESPNOW

extern uint8_t status_ESPNOW_Sent;

class ESPNOW_ROBOT {
public:
  esp_now_peer_info_t peerInfo = {};
  uint8_t broadcastAddress[6] = { 0, 0, 0, 0, 0, 0 };

  ESPNOW_ROBOT() {
    // Use this constructor when the sketch only receives ESP-NOW packets.
  }

  ESPNOW_ROBOT(const uint8_t broadAddress[]) {
    // Store the receiver MAC address used by Sendvalue_ESPNOW().
    memcpy(broadcastAddress, broadAddress, sizeof(broadcastAddress));
  }

  // Print this ESP32 MAC address. Use it when pairing two ESP-NOW boards.
  void MAC_Address_ESPNOW();

  // Initialize ESP-NOW for sending and register the peer address.
  void Setup_send_ESPNOW();

  // Send a raw byte payload to the configured peer address.
  void Sendvalue_ESPNOW(uint8_t *data, size_t len);

  // Initialize ESP-NOW receive mode. The sketch registers its receive callback.
  void Setup_receive_ESPNOW();
};

#endif
