// ESP32 sketch for sending iROB feedback data to a ROS2 computer over WiFi.
// The UDP packet layout is kept compatible with the TX packet from app_ros_comm.ino.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// 1 = ESP32 creates its own WiFi access point.
// 0 = ESP32 joins an existing WiFi network as a station.
#define IROB_WIFI_AP_MODE 1

// WiFi credentials requested for the robot link.
static const char *IROB_WIFI_SSID = "ABU_robot2027";
static const char *IROB_WIFI_PASSWORD = "ABU_robot67";

// Access point network settings.
// In AP mode this is the ESP32 address, so the ROS2 computer should join this AP.
static const IPAddress IROB_AP_IP(192, 168, 1, 67);
static const IPAddress IROB_AP_GATEWAY(192, 168, 1, 67);
static const IPAddress IROB_AP_SUBNET(255, 255, 255, 0);

// Optional static IP settings for station mode.
// Keep this disabled unless the existing network needs a fixed ESP32 address.
#define IROB_WIFI_USE_STATIC_STA_IP 0
static const IPAddress IROB_STA_LOCAL_IP(192, 168, 1, 68);
static const IPAddress IROB_STA_GATEWAY(192, 168, 1, 1);
static const IPAddress IROB_STA_SUBNET(255, 255, 255, 0);

// UDP destination.
// 192.168.1.255 is the subnet broadcast address, so any ROS2 computer on this link can receive.
static const IPAddress ROS2_NODE_IP(192, 168, 1, 255);
static const uint16_t IROB_UDP_PORT = 6767;

// Send period in milliseconds. 10 ms gives a 100 Hz feedback stream.
static const uint32_t IROB_SEND_PERIOD_MS = 10;

// Exact 32-byte feedback packet sent from ESP32 to ROS2.
// __attribute__((packed)) prevents the compiler from adding padding bytes.
typedef struct __attribute__((packed)) {
  // Header bytes. Must be 'J' and 'B' for the ROS2 node to accept the packet.
  uint8_t ajbHeader[2];

  // Reply or status byte from MCU. 0x00 means normal in the current sketch.
  uint8_t cmdDataMCU;

  // Motor feedback values in the same order as app_ros_comm.ino: LF, LB, RB, RF.
  struct {
    int16_t motor1_fb;
    int16_t motor2_fb;
    int16_t motor3_fb;
    int16_t motor4_fb;
  } motorFeedBack;

  // Optional mouse/optical-flow velocity fields kept for protocol compatibility.
  struct {
    int8_t mouse_x_vel;
    int8_t mouse_y_vel;
  } mouseVel;

  // Raw gyroscope values from the IMU.
  int16_t gyro_x_raw;
  int16_t gyro_y_raw;
  int16_t gyro_z_raw;

  // Raw magnetometer values.
  int16_t mag_x_raw;
  int16_t mag_y_raw;
  int16_t mag_z_raw;

  // Raw accelerometer values from the IMU.
  int16_t acc_x_raw;
  int16_t acc_y_raw;
  int16_t acc_z_raw;

  // XOR checksum over the first 31 bytes.
  uint8_t cks;
} irob_ros_packet_t;

// Compile-time guard: if the struct changes size, the build will stop here.
static_assert(sizeof(irob_ros_packet_t) == 32, "iROB ROS packet must be 32 bytes");

// UDP socket object and reusable transmit packet.
WiFiUDP udp;
irob_ros_packet_t txPacket;

// Timestamp used to keep the send loop at IROB_SEND_PERIOD_MS.
uint32_t lastSendMs = 0;

// Calculate the simple XOR checksum used by this WiFi protocol.
uint8_t checksumXor(const irob_ros_packet_t &packet)
{
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&packet);
  uint8_t checksum = 0;

  // The final byte is the checksum field itself, so it is not included.
  for (size_t i = 0; i < sizeof(irob_ros_packet_t) - 1; ++i) {
    checksum ^= bytes[i];
  }

  return checksum;
}

// Fill all outgoing fields before each UDP send.
// Replace the zero assignments here with real encoder and IMU values from the robot.
void fillRobotData(irob_ros_packet_t &packet)
{
  // Packet marker expected by the ROS2 receiver.
  packet.ajbHeader[0] = 'J';
  packet.ajbHeader[1] = 'B';

  // Current MCU status or command response.
  packet.cmdDataMCU = 0x00;

  // TODO: connect these fields to real motor RPM/encoder feedback.
  packet.motorFeedBack.motor1_fb = 0;  // LF
  packet.motorFeedBack.motor2_fb = 0;  // LB
  packet.motorFeedBack.motor3_fb = 0;  // RB
  packet.motorFeedBack.motor4_fb = 0;  // RF

  // TODO: connect these fields if mouse/optical-flow data is available.
  packet.mouseVel.mouse_x_vel = 0;
  packet.mouseVel.mouse_y_vel = 0;

  // TODO: connect these fields to raw gyro readings.
  packet.gyro_x_raw = 0;
  packet.gyro_y_raw = 0;
  packet.gyro_z_raw = 0;

  // TODO: connect these fields to raw magnetometer readings if the robot has one.
  packet.mag_x_raw = 0;
  packet.mag_y_raw = 0;
  packet.mag_z_raw = 0;

  // TODO: connect these fields to raw accelerometer readings.
  packet.acc_x_raw = 0;
  packet.acc_y_raw = 0;
  packet.acc_z_raw = 0;

  // Checksum must be calculated after every data field has been filled.
  packet.cks = checksumXor(packet);
}

// Start WiFi and bind the local UDP port.
void startWiFi()
{
  // Clear any previous WiFi state before switching AP/STA mode.
  WiFi.disconnect(true, true);
  delay(100);

#if IROB_WIFI_AP_MODE
  // AP mode: the ESP32 creates the robot network itself.
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IROB_AP_IP, IROB_AP_GATEWAY, IROB_AP_SUBNET);
  WiFi.softAP(IROB_WIFI_SSID, IROB_WIFI_PASSWORD);

  Serial.print("iROB AP SSID: ");
  Serial.println(IROB_WIFI_SSID);
  Serial.print("iROB AP IP: ");
  Serial.println(WiFi.softAPIP());
#else
  // Station mode: the ESP32 connects to an existing access point/router.
  WiFi.mode(WIFI_STA);
#if IROB_WIFI_USE_STATIC_STA_IP
  // Optional static station address. Disabled by default to avoid IP conflicts.
  WiFi.config(IROB_STA_LOCAL_IP, IROB_STA_GATEWAY, IROB_STA_SUBNET);
#endif
  WiFi.begin(IROB_WIFI_SSID, IROB_WIFI_PASSWORD);

  // Block here until the ESP32 is connected before sending UDP packets.
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
#endif

  // Open a UDP socket. This also allows replies or diagnostics on the same port later.
  udp.begin(IROB_UDP_PORT);

  Serial.print("Sending UDP packets to ");
  Serial.print(ROS2_NODE_IP);
  Serial.print(":");
  Serial.println(IROB_UDP_PORT);
}

// Serialize and send one feedback packet to the ROS2 bridge node.
void sendPacketToRos()
{
  // Refresh all packet fields just before sending.
  fillRobotData(txPacket);

  // Send the packed 32-byte binary packet by UDP.
  udp.beginPacket(ROS2_NODE_IP, IROB_UDP_PORT);
  udp.write(reinterpret_cast<const uint8_t *>(&txPacket), sizeof(txPacket));
  udp.endPacket();
}

void setup()
{
  // Serial is only used for debug/status prints.
  Serial.begin(115200);

  // Configure WiFi once at startup.
  startWiFi();
}

void loop()
{
#if !IROB_WIFI_AP_MODE
  // Reconnect automatically when station mode loses WiFi.
  if (WiFi.status() != WL_CONNECTED) {
    startWiFi();
  }
#endif

  // Non-blocking 100 Hz send scheduler.
  const uint32_t now = millis();
  if ((uint32_t)(now - lastSendMs) >= IROB_SEND_PERIOD_MS) {
    lastSendMs = now;
    sendPacketToRos();
  }
}
