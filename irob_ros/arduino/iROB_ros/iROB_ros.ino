// ESP32 STA sketch for sending iROB feedback data to a ROS2 computer over WiFi.
// The UDP packet layout is kept compatible with the TX packet from app_ros_comm.ino.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "Inc/iROB_Motor.h"
#include "Inc/MPU6050_E12.h"

// The matching .cpp files are kept in this sketch folder's src/ directory.

// Credentials for the existing WiFi network that the ESP32 will join.
static const char *IROB_WIFI_SSID = "ABU_robot2027";
static const char *IROB_WIFI_PASSWORD = "ABU_robot67";

// Optional static IP settings for STA mode.
// 0 = ESP32 gets its IP from DHCP.
// 1 = ESP32 uses IROB_STA_LOCAL_IP below.
#define IROB_WIFI_USE_STATIC_STA_IP 0
static const IPAddress IROB_STA_LOCAL_IP(192, 168, 1, 68);
static const IPAddress IROB_STA_GATEWAY(192, 168, 1, 1);
static const IPAddress IROB_STA_SUBNET(255, 255, 255, 0);

// UDP destination.
// 192.168.1.255 broadcasts to every device on the 192.168.1.x WiFi network.
static const IPAddress ROS2_NODE_IP(192, 168, 1, 255);
static const uint16_t IROB_UDP_PORT = 6767;

// Send period in milliseconds. 10 ms gives a 100 Hz feedback stream.
static const uint32_t IROB_SEND_PERIOD_MS = 10;

// Stop motor targets if no ROS2 command packet arrives within this time.
static const uint32_t IROB_COMMAND_TIMEOUT_MS = 500;

// ROS2 -> ESP32 command packet size. This matches app_ros_comm.ino RX size.
static const size_t IROB_COMMAND_PACKET_SIZE = 13;

// Command register values compatible with the older serial protocol.
static const uint8_t IROB_REG_NULL = 0x00;
static const uint8_t IROB_REG_ESTOP = 0xAA;
static const uint8_t IROB_STATUS_OK = 0x00;
static const uint8_t IROB_STATUS_ESTOP = 0x55;

// Motor feedback is calculated at the same 100 Hz rate as the UDP feedback stream.
static const uint16_t IROB_MOTOR_FREQ_HZ = 100;
static const uint16_t IROB_ENCODER_CPR = 68;
static const uint16_t IROB_GEAR_RATIO = 27;

// PID settings copied from the main iROB sketch.
static const float IROB_PID_KP = 1.5f;
static const float IROB_PID_KI = 0.01f;
static const float IROB_PID_KD = 0.1f;
static const float IROB_MIN_SPEED_RPM = 100.0f;
static const float IROB_MAX_SPEED_RPM = 300.0f;

// MPU6050 I2C pins. Change these two constants if your board uses different wiring.
static const uint8_t IROB_MPU6050_SDA_PIN = 22;
static const uint8_t IROB_MPU6050_SCL_PIN = 23;
static const uint16_t IROB_MPU6050_FREQ_HZ = 100;

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

// Robot hardware objects from the existing project libraries.
iROB_Motor iROB(IROB_MOTOR_FREQ_HZ, IROB_ENCODER_CPR, IROB_GEAR_RATIO);
MPU6050 mpu;

// Timestamp used to keep the send loop at IROB_SEND_PERIOD_MS.
uint32_t lastSendMs = 0;

// Latest motor targets received from ROS2, stored in wheel names for readability.
int16_t targetRpmLF = 0;
int16_t targetRpmLB = 0;
int16_t targetRpmRF = 0;
int16_t targetRpmRB = 0;

// Latest measured RPM values. These are sent back in irob_ros_packet_t.
int16_t feedbackRpmLF = 0;
int16_t feedbackRpmLB = 0;
int16_t feedbackRpmRF = 0;
int16_t feedbackRpmRB = 0;

// MCU status byte that is reported back to ROS2 in cmdDataMCU.
uint8_t mcuStatus = IROB_STATUS_OK;

// Timestamp of the latest valid command packet from ROS2.
uint32_t lastCommandMs = 0;

// Read one little-endian int16 field from a received UDP command packet.
int16_t readInt16LE(const uint8_t *bytes)
{
  const uint16_t value = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
  return (int16_t)value;
}

// Clamp float RPM feedback to the int16 field used by the original packet.
int16_t floatToInt16(float value)
{
  if (value > 32767.0f) {
    return 32767;
  }

  if (value < -32768.0f) {
    return -32768;
  }

  return (int16_t)value;
}

// Set every motor target to zero.
void clearMotorTargets()
{
  targetRpmLF = 0;
  targetRpmLB = 0;
  targetRpmRF = 0;
  targetRpmRB = 0;
}

// Immediately stop motor PWM outputs.
void stopMotorOutputs()
{
  iROB.Motor_DutyCycle_LF(0);
  iROB.Motor_DutyCycle_LB(0);
  iROB.Motor_DutyCycle_RF(0);
  iROB.Motor_DutyCycle_RB(0);
}

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
  packet.cmdDataMCU = mcuStatus;

  // Motor feedback uses the app_ros_comm.ino packet order: LF, LB, RB, RF.
  packet.motorFeedBack.motor1_fb = feedbackRpmLF;
  packet.motorFeedBack.motor2_fb = feedbackRpmLB;
  packet.motorFeedBack.motor3_fb = feedbackRpmRB;
  packet.motorFeedBack.motor4_fb = feedbackRpmRF;

  // TODO: connect these fields if mouse/optical-flow data is available.
  packet.mouseVel.mouse_x_vel = 0;
  packet.mouseVel.mouse_y_vel = 0;

  // Raw gyro readings from MPU6050_E12.h.
  packet.gyro_x_raw = mpu.mpuData.gx;
  packet.gyro_y_raw = mpu.mpuData.gy;
  packet.gyro_z_raw = mpu.mpuData.gz;

  // MPU6050 has gyro + accelerometer only, so magnetometer fields stay zero.
  packet.mag_x_raw = 0;
  packet.mag_y_raw = 0;
  packet.mag_z_raw = 0;

  // Raw accelerometer readings from MPU6050_E12.h.
  packet.acc_x_raw = mpu.mpuData.Ax;
  packet.acc_y_raw = mpu.mpuData.Ay;
  packet.acc_z_raw = mpu.mpuData.Az;

  // Checksum must be calculated after every data field has been filled.
  packet.cks = checksumXor(packet);
}

// Process one valid 13-byte command packet from ROS2.
void applyCommandPacket(const uint8_t *packet)
{
  if (packet[0] != 'R' || packet[1] != 'B') {
    return;
  }

  const uint8_t reg = packet[2];
  lastCommandMs = millis();

  if (reg == IROB_REG_ESTOP) {
    mcuStatus = IROB_STATUS_ESTOP;
    clearMotorTargets();
    stopMotorOutputs();
    return;
  }

  mcuStatus = IROB_STATUS_OK;

  // Command packet order is LF, LB, RB, RF.
  targetRpmLF = readInt16LE(&packet[4]);
  targetRpmLB = readInt16LE(&packet[6]);
  targetRpmRB = readInt16LE(&packet[8]);
  targetRpmRF = readInt16LE(&packet[10]);
}

// Read all pending UDP command packets from ROS2.
void receiveCommandFromRos()
{
  int packetSize = udp.parsePacket();

  while (packetSize > 0) {
    uint8_t packet[IROB_COMMAND_PACKET_SIZE] = { 0 };
    const int readLength = udp.read(packet, sizeof(packet));

    while (udp.available() > 0) {
      udp.read();
    }

    if (packetSize == (int)IROB_COMMAND_PACKET_SIZE &&
        readLength == (int)IROB_COMMAND_PACKET_SIZE) {
      applyCommandPacket(packet);
    }

    packetSize = udp.parsePacket();
  }
}

// Stop the robot if ROS2 control data is lost.
void applyCommandTimeout()
{
  if ((uint32_t)(millis() - lastCommandMs) > IROB_COMMAND_TIMEOUT_MS) {
    clearMotorTargets();
  }
}

// Read sensors, update motor PID, and cache feedback data for the next packet.
void updateRobotHardware()
{
  feedbackRpmLF = floatToInt16(iROB.getRPM(_LF));
  feedbackRpmLB = floatToInt16(iROB.getRPM(_LB));
  feedbackRpmRF = floatToInt16(iROB.getRPM(_RF));
  feedbackRpmRB = floatToInt16(iROB.getRPM(_RB));

  iROB.Motor_Speed_LF(targetRpmLF, feedbackRpmLF);
  iROB.Motor_Speed_LB(targetRpmLB, feedbackRpmLB);
  iROB.Motor_Speed_RF(targetRpmRF, feedbackRpmRF);
  iROB.Motor_Speed_RB(targetRpmRB, feedbackRpmRB);

  mpu.MPU_get_gyro();
  mpu.MPU_get_Accelerometer();
}

// Join WiFi in STA mode and bind the local UDP port.
void startWiFi()
{
  // Clear any previous WiFi state before joining the network.
  WiFi.disconnect(true, true);
  delay(100);

  // STA mode only: the ESP32 joins the existing WiFi network.
  WiFi.mode(WIFI_STA);

#if IROB_WIFI_USE_STATIC_STA_IP
  // Optional static ESP32 address. Disabled by default to avoid IP conflicts.
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

  // Open the UDP socket used to send robot feedback packets.
  udp.stop();
  udp.begin(IROB_UDP_PORT);

  Serial.print("Sending UDP packets to ");
  Serial.print(ROS2_NODE_IP);
  Serial.print(":");
  Serial.println(IROB_UDP_PORT);
}

// Initialize sensors and leave motor outputs stopped.
void setupRobotHardware()
{
  iROB.Setup_PID_Wheel(IROB_PID_KP, IROB_PID_KI, IROB_PID_KD, IROB_MIN_SPEED_RPM, IROB_MAX_SPEED_RPM, _LF);
  iROB.Setup_PID_Wheel(IROB_PID_KP, IROB_PID_KI, IROB_PID_KD, IROB_MIN_SPEED_RPM, IROB_MAX_SPEED_RPM, _LB);
  iROB.Setup_PID_Wheel(IROB_PID_KP, IROB_PID_KI, IROB_PID_KD, IROB_MIN_SPEED_RPM, IROB_MAX_SPEED_RPM, _RF);
  iROB.Setup_PID_Wheel(IROB_PID_KP, IROB_PID_KI, IROB_PID_KD, IROB_MIN_SPEED_RPM, IROB_MAX_SPEED_RPM, _RB);

  clearMotorTargets();
  stopMotorOutputs();

  Wire.begin(IROB_MPU6050_SDA_PIN, IROB_MPU6050_SCL_PIN);
  mpu.freq_MPU6050 = IROB_MPU6050_FREQ_HZ;
  mpu.MPU_init();
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

  // Prepare encoder/motor pins and MPU6050 before networking starts.
  setupRobotHardware();

  // Join WiFi once at startup.
  startWiFi();
}

void loop()
{
  // Reconnect automatically if the ESP32 drops off WiFi.
  if (WiFi.status() != WL_CONNECTED) {
    clearMotorTargets();
    stopMotorOutputs();
    startWiFi();
  }

  receiveCommandFromRos();

  // Non-blocking 100 Hz send scheduler.
  const uint32_t now = millis();
  if ((uint32_t)(now - lastSendMs) >= IROB_SEND_PERIOD_MS) {
    lastSendMs = now;
    applyCommandTimeout();
    updateRobotHardware();
    sendPacketToRos();
  }
}
