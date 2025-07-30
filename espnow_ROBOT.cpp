// #include"espnow_ROBOT.h"


// void ESPNOW_ROBOT ::MAC_Address_ESPNOW(){
//   // Setup Serial Monitor
//   Serial.begin(115200);
 
//   // Put ESP32 into Station mode
//   WiFi.mode(WIFI_MODE_STA);
 
//   // Print MAC Address to Serial monitor
//   Serial.print("MAC Address: ");
//   Serial.println(WiFi.macAddress());
// }

// uint8_t status_ESPNOW_Sent;

// void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
// #ifdef ESPNOW
//   Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success " : "Delivery Fail ");
// #endif
//  if(status == ESP_NOW_SEND_SUCCESS){
//   status_ESPNOW_Sent = 1;
//  }else{
//   status_ESPNOW_Sent = 0;
//  }
// }

// void ESPNOW_ROBOT ::Setup_send_ESPNOW() {

//   WiFi.mode(WIFI_STA);

//   // Initilize ESP-NOW
//   if (esp_now_init() != ESP_OK) {
// #ifdef ESPNOW
//     Serial.println("Error initializing ESP-NOW");
// #endif
//     return;
//   }

//   // Register the send callback
//   esp_now_register_send_cb(OnDataSent);

//   // Register peer
//   memcpy(peerInfo.peer_addr, broadcastAddress, sizeof(broadcastAddress));
//   peerInfo.channel = 0;
//   peerInfo.encrypt = false;

//   // Add peer
//   if (esp_now_add_peer(&peerInfo) != ESP_OK) {
// #ifdef ESPNOW
//     Serial.println("Failed to add peer");
// #endif
//     return;
//   }
// }

// void ESPNOW_ROBOT ::Sendvalue_ESPNOW(uint8_t* data, size_t len) {
//   esp_err_t result = esp_now_send(broadcastAddress, data, len);

//   if (result == ESP_OK) {
// #ifdef ESPNOW
//     Serial.println(" | Sending confirmed");
// #endif
//   } else {
// #ifdef ESPNOW
//     Serial.println(" | Sending error");
// #endif
//   }
//   // delay(10);
// }

// void ESPNOW_ROBOT ::Setup_receive_ESPNOW() {
//   // Set up Serial Monitor
//   Serial.begin(115200);
  
//   // Set ESP32 as a Wi-Fi Station
//   WiFi.mode(WIFI_STA);

//   // Initilize ESP-NOW
//   if (esp_now_init() != ESP_OK) {
// #ifdef ESPNOW
//     Serial.println("Error initializing ESP-NOW");
// #endif
//     return;
//   }
// }

// // void ESPNOW_ROBOT::Receivevalue_ESPNOW()
// // {
// //     esp_now_register_recv_cb(Esp_Now_Data_Receieve);
// // }

// // Receive_ESPNOW Data;
// // void Esp_Now_Data_Receieve(const uint8_t *Mac_Address, const uint8_t *Received_Data, int length)
// // {
// //     memcpy(&Data, Received_Data, sizeof(Data));
// // }