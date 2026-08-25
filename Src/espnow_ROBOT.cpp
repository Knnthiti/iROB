#include "../Inc/espnow_ROBOT.h"

uint8_t status_ESPNOW_Sent = 0;

void ESPNOW_ROBOT::MAC_Address_ESPNOW() {
  // Put ESP32 into station mode before reading the station MAC address.
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);

  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // ESP-NOW send callback. The public flag is useful for simple retry logic.
#ifdef ESPNOW
  Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success " : "Delivery Fail ");
#endif

  if (status == ESP_NOW_SEND_SUCCESS) {
    status_ESPNOW_Sent = 1;
  } else {
    status_ESPNOW_Sent = 0;
  }
}

void ESPNOW_ROBOT::Setup_send_ESPNOW() {
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW before registering callbacks or peers.
  if (esp_now_init() != ESP_OK) {
#ifdef ESPNOW
    Serial.println("Error initializing ESP-NOW");
#endif
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  // Register the peer board that will receive Sendvalue_ESPNOW() payloads.
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, sizeof(broadcastAddress));
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
#ifdef ESPNOW
    Serial.println("Failed to add peer");
#endif
    return;
  }
}

void ESPNOW_ROBOT::Sendvalue_ESPNOW(uint8_t *data, size_t len) {
  // Send the caller-provided byte buffer to the configured peer.
  esp_err_t result = esp_now_send(broadcastAddress, data, len);

  if (result == ESP_OK) {
#ifdef ESPNOW
    Serial.println(" | Sending confirmed");
#endif
  } else {
#ifdef ESPNOW
    Serial.println(" | Sending error");
#endif
  }
}

void ESPNOW_ROBOT::Setup_receive_ESPNOW() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // The receive callback is registered in the sketch with
  // esp_now_register_recv_cb(OnDataRecv), because each sketch owns its payload.
  if (esp_now_init() != ESP_OK) {
#ifdef ESPNOW
    Serial.println("Error initializing ESP-NOW");
#endif
    return;
  }
}
