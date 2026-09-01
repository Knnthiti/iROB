// Sketch ฝั่ง ESP32 สำหรับเชื่อมกับ ROS2 ผ่าน WiFi แบบ STA mode
// หน้าที่หลัก:
// 1. รับคำสั่งจาก ROS2 node iROB_CMD ผ่าน UDP packet ขนาด 13 bytes
// 2. ควบคุม motor ด้วย iROB_Motor.h
// 3. อ่านค่า IMU จาก MPU6050_E12.h
// 4. ส่ง feedback กลับไปให้ ROS2 node iROB_ESP ผ่าน UDP packet ขนาด 32 bytes
// รูปแบบ packet ยังอ้างอิงจาก app_ros_comm.ino เพื่อให้ข้อมูลเหมือน protocol เดิม

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "Inc/iROB_Motor.h"
#include "Inc/MPU6050_E12.h"

// ไฟล์ .cpp ที่คู่กับ header ด้านบนถูกเก็บไว้ในโฟลเดอร์ src/ ของ sketch นี้

// SSID/password ของ WiFi ที่ ESP32 จะ join เข้าไป
static const char *IROB_WIFI_SSID = "ABU_robot2027";
static const char *IROB_WIFI_PASSWORD = "ABU_robot67";

// ตั้งค่า IP แบบ static สำหรับ STA mode
// 0 = ให้ router แจก IP ด้วย DHCP
// 1 = บังคับให้ ESP32 ใช้ IP ตาม IROB_STA_LOCAL_IP
#define IROB_WIFI_USE_STATIC_STA_IP 0
static const IPAddress IROB_STA_LOCAL_IP(192, 168, 1, 68);
static const IPAddress IROB_STA_GATEWAY(192, 168, 1, 1);
static const IPAddress IROB_STA_SUBNET(255, 255, 255, 0);

// ปลายทาง UDP สำหรับส่ง feedback ไปหาเครื่อง ROS2
// 192.168.1.255 คือ broadcast address ของวง 192.168.1.x
// วิธีนี้ทำให้ node iROB_ESP บนเครื่อง ROS2 รับ packet ได้โดยไม่ต้อง fix IP เครื่อง ROS2
static const IPAddress ROS2_NODE_IP(192, 168, 1, 255);
static const uint16_t IROB_UDP_PORT = 6767;

// คาบเวลาส่ง feedback ไป ROS2
// 10 ms = 100 Hz
static const uint32_t IROB_SEND_PERIOD_MS = 10;

// ถ้าไม่ได้รับคำสั่งจาก ROS2 นานเกินเวลานี้ จะตั้ง target motor เป็น 0 เพื่อหยุดหุ่น
static const uint32_t IROB_COMMAND_TIMEOUT_MS = 500;

// ขนาด packet คำสั่งจาก ROS2 -> ESP32
// ต้องตรงกับส่วน RX ของ app_ros_comm.ino
static const size_t IROB_COMMAND_PACKET_SIZE = 13;

// ค่า register/status ที่ใช้ร่วมกับ protocol เดิม
static const uint8_t IROB_REG_NULL = 0x00;
static const uint8_t IROB_REG_ESTOP = 0xAA;
static const uint8_t IROB_STATUS_OK = 0x00;
static const uint8_t IROB_STATUS_ESTOP = 0x55;

// ค่าพื้นฐานของ encoder/motor สำหรับคำนวณ feedback RPM
static const uint16_t IROB_MOTOR_FREQ_HZ = 100;
static const uint16_t IROB_ENCODER_CPR = 68;
static const uint16_t IROB_GEAR_RATIO = 27;

// ค่า PID สำหรับควบคุมความเร็ว motor
// คัดลอกจาก sketch หลักของ iROB
static const float IROB_PID_KP = 1.5f;
static const float IROB_PID_KI = 0.01f;
static const float IROB_PID_KD = 0.1f;
static const float IROB_MIN_SPEED_RPM = 100.0f;
static const float IROB_MAX_SPEED_RPM = 300.0f;

// ขา I2C ของ MPU6050
// ถ้าต่อสาย SDA/SCL คนละขา ให้แก้ค่าตรงนี้
static const uint8_t IROB_MPU6050_SDA_PIN = 22;
static const uint8_t IROB_MPU6050_SCL_PIN = 23;
static const uint16_t IROB_MPU6050_FREQ_HZ = 100;

// โครงสร้าง feedback packet ขนาด 32 bytes ที่ ESP32 ส่งกลับไปให้ ROS2
// __attribute__((packed)) ป้องกัน compiler แทรก padding byte เพิ่ม
typedef struct __attribute__((packed)) {
  // header ต้องเป็น 'J' และ 'B' เพื่อให้ ROS2 node ยอมรับ packet
  uint8_t ajbHeader[2];

  // สถานะจาก MCU เช่น ปกติ หรือ emergency stop
  uint8_t cmdDataMCU;

  // feedback motor เรียงเหมือน app_ros_comm.ino: LF, LB, RB, RF
  struct {
    int16_t motor1_fb;
    int16_t motor2_fb;
    int16_t motor3_fb;
    int16_t motor4_fb;
  } motorFeedBack;

  // ช่อง mouse/optical-flow เก็บไว้เพื่อให้ protocol ยังเหมือนเดิม
  // ตอนนี้ยังไม่ได้ต่อ sensor จึงส่งค่า 0
  struct {
    int8_t mouse_x_vel;
    int8_t mouse_y_vel;
  } mouseVel;

  // ค่า gyro raw จาก MPU6050
  int16_t gyro_x_raw;
  int16_t gyro_y_raw;
  int16_t gyro_z_raw;

  // ค่า magnetometer raw
  // MPU6050 ไม่มี magnetometer จึงใส่ 0 ไว้ใน fillRobotData()
  int16_t mag_x_raw;
  int16_t mag_y_raw;
  int16_t mag_z_raw;

  // ค่า accelerometer raw จาก MPU6050
  int16_t acc_x_raw;
  int16_t acc_y_raw;
  int16_t acc_z_raw;

  // checksum แบบ XOR จาก byte 0 ถึง byte 30
  uint8_t cks;
} irob_ros_packet_t;

// ตรวจตั้งแต่ตอน compile ว่า packet ยังมีขนาด 32 bytes จริง
// ถ้าเผลอแก้ struct แล้วขนาดเปลี่ยน build จะหยุดทันที
static_assert(sizeof(irob_ros_packet_t) == 32, "iROB ROS packet must be 32 bytes");

// object สำหรับ UDP และ packet ที่ใช้ซ้ำทุกครั้งก่อนส่ง feedback
WiFiUDP udp;
irob_ros_packet_t txPacket;

// object ของ library เดิมในโปรเจกต์ สำหรับควบคุม motor และอ่าน IMU
iROB_Motor iROB(IROB_MOTOR_FREQ_HZ, IROB_ENCODER_CPR, IROB_GEAR_RATIO);
MPU6050 mpu;

// timestamp สำหรับควบคุมรอบส่ง feedback ให้ได้ตาม IROB_SEND_PERIOD_MS
uint32_t lastSendMs = 0;

// target RPM ล่าสุดที่รับมาจาก ROS2
// ตั้งชื่อเป็นตำแหน่งล้อเพื่ออ่าน logic ง่าย
int16_t targetRpmLF = 0;
int16_t targetRpmLB = 0;
int16_t targetRpmRF = 0;
int16_t targetRpmRB = 0;

// RPM feedback ล่าสุดที่อ่านจาก encoder
// ค่านี้จะถูกใส่ลง irob_ros_packet_t แล้วส่งกลับ ROS2
int16_t feedbackRpmLF = 0;
int16_t feedbackRpmLB = 0;
int16_t feedbackRpmRF = 0;
int16_t feedbackRpmRB = 0;

// สถานะ MCU ที่จะรายงานกลับ ROS2 ผ่าน field cmdDataMCU
uint8_t mcuStatus = IROB_STATUS_OK;

// เวลาที่ได้รับ command packet ถูกต้องล่าสุดจาก ROS2
uint32_t lastCommandMs = 0;

// อ่าน int16 แบบ little-endian จาก command packet ที่รับมาทาง UDP
int16_t readInt16LE(const uint8_t *bytes)
{
  const uint16_t value = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
  return (int16_t)value;
}

// จำกัดค่า RPM แบบ float ให้อยู่ในช่วง int16 ก่อนใส่ลง packet เดิม
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

// ตั้ง target motor ทุกล้อเป็น 0
void clearMotorTargets()
{
  targetRpmLF = 0;
  targetRpmLB = 0;
  targetRpmRF = 0;
  targetRpmRB = 0;
}

// สั่ง PWM ทุกล้อเป็น 0 ทันที
// ใช้ตอน WiFi หลุดหรือ emergency stop
void stopMotorOutputs()
{
  iROB.Motor_DutyCycle_LF(0);
  iROB.Motor_DutyCycle_LB(0);
  iROB.Motor_DutyCycle_RF(0);
  iROB.Motor_DutyCycle_RB(0);
}

// คำนวณ checksum แบบ XOR สำหรับ feedback packet
uint8_t checksumXor(const irob_ros_packet_t &packet)
{
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&packet);
  uint8_t checksum = 0;

  // byte สุดท้ายคือ checksum เอง จึงไม่เอามารวมคำนวณ
  for (size_t i = 0; i < sizeof(irob_ros_packet_t) - 1; ++i) {
    checksum ^= bytes[i];
  }

  return checksum;
}

// เติมข้อมูลทั้งหมดก่อนส่ง feedback ไป ROS2
// ข้อมูล motor มาจาก encoder/PID และข้อมูล IMU มาจาก MPU6050_E12.h
void fillRobotData(irob_ros_packet_t &packet)
{
  // marker ที่ ROS2 receiver ใช้ตรวจว่าเป็น packet ของ iROB
  packet.ajbHeader[0] = 'J';
  packet.ajbHeader[1] = 'B';

  // สถานะปัจจุบันของ MCU
  packet.cmdDataMCU = mcuStatus;

  // motor feedback ใช้ลำดับจาก app_ros_comm.ino: LF, LB, RB, RF
  packet.motorFeedBack.motor1_fb = feedbackRpmLF;
  packet.motorFeedBack.motor2_fb = feedbackRpmLB;
  packet.motorFeedBack.motor3_fb = feedbackRpmRB;
  packet.motorFeedBack.motor4_fb = feedbackRpmRF;

  // ถ้ามี mouse/optical-flow sensor ในอนาคต ให้เปลี่ยน 0 เป็นค่าจริงตรงนี้
  packet.mouseVel.mouse_x_vel = 0;
  packet.mouseVel.mouse_y_vel = 0;

  // ค่า gyro raw จาก MPU6050_E12.h
  packet.gyro_x_raw = mpu.mpuData.gx;
  packet.gyro_y_raw = mpu.mpuData.gy;
  packet.gyro_z_raw = mpu.mpuData.gz;

  // MPU6050 มีเฉพาะ gyro + accelerometer จึงส่ง magnetometer เป็น 0
  packet.mag_x_raw = 0;
  packet.mag_y_raw = 0;
  packet.mag_z_raw = 0;

  // ค่า accelerometer raw จาก MPU6050_E12.h
  packet.acc_x_raw = mpu.mpuData.Ax;
  packet.acc_y_raw = mpu.mpuData.Ay;
  packet.acc_z_raw = mpu.mpuData.Az;

  // ต้องคำนวณ checksum หลังจากเติมข้อมูลทุก field แล้ว
  packet.cks = checksumXor(packet);
}

// ประมวลผล command packet ขนาด 13 bytes จาก ROS2
// packet นี้ถูกสร้างจาก message /iROB_command โดย node iROB_CMD
void applyCommandPacket(const uint8_t *packet)
{
  // header ต้องเป็น 'R', 'B' ถ้าไม่ตรงให้ทิ้ง packet
  if (packet[0] != 'R' || packet[1] != 'B') {
    return;
  }

  const uint8_t reg = packet[2];
  lastCommandMs = millis();

  // ถ้า ROS2 ส่ง emergency stop มา ให้หยุด motor ทันที
  if (reg == IROB_REG_ESTOP) {
    mcuStatus = IROB_STATUS_ESTOP;
    clearMotorTargets();
    stopMotorOutputs();
    return;
  }

  mcuStatus = IROB_STATUS_OK;

  // ลำดับ command packet คือ LF, LB, RB, RF
  targetRpmLF = readInt16LE(&packet[4]);
  targetRpmLB = readInt16LE(&packet[6]);
  targetRpmRB = readInt16LE(&packet[8]);
  targetRpmRF = readInt16LE(&packet[10]);
}

// อ่าน command packet ที่ค้างอยู่ใน UDP socket ทั้งหมด
// ถ้ามี packet หลายก้อน จะอ่านวนจนหมดเพื่อใช้คำสั่งล่าสุด
void receiveCommandFromRos()
{
  int packetSize = udp.parsePacket();

  while (packetSize > 0) {
    uint8_t packet[IROB_COMMAND_PACKET_SIZE] = { 0 };
    const int readLength = udp.read(packet, sizeof(packet));

    // ถ้า packet ยาวเกิน buffer ให้ทิ้ง byte ที่เหลือ
    while (udp.available() > 0) {
      udp.read();
    }

    // รับเฉพาะ packet ที่ขนาดถูกต้องเท่านั้น
    if (packetSize == (int)IROB_COMMAND_PACKET_SIZE &&
        readLength == (int)IROB_COMMAND_PACKET_SIZE) {
      applyCommandPacket(packet);
    }

    packetSize = udp.parsePacket();
  }
}

// หยุดหุ่นถ้าข้อมูลควบคุมจาก ROS2 ขาดหาย
void applyCommandTimeout()
{
  if ((uint32_t)(millis() - lastCommandMs) > IROB_COMMAND_TIMEOUT_MS) {
    clearMotorTargets();
  }
}

// อ่าน sensor, อัปเดต PID motor, และเก็บ feedback สำหรับ packet รอบถัดไป
void updateRobotHardware()
{
  // อ่าน RPM feedback จาก encoder
  feedbackRpmLF = floatToInt16(iROB.getRPM(_LF));
  feedbackRpmLB = floatToInt16(iROB.getRPM(_LB));
  feedbackRpmRF = floatToInt16(iROB.getRPM(_RF));
  feedbackRpmRB = floatToInt16(iROB.getRPM(_RB));

  // สั่ง PID speed control ตาม target ที่มาจาก ROS2
  iROB.Motor_Speed_LF(targetRpmLF, feedbackRpmLF);
  iROB.Motor_Speed_LB(targetRpmLB, feedbackRpmLB);
  iROB.Motor_Speed_RF(targetRpmRF, feedbackRpmRF);
  iROB.Motor_Speed_RB(targetRpmRB, feedbackRpmRB);

  // อ่าน IMU เพื่อใช้ส่ง feedback กลับ ROS2
  mpu.MPU_get_gyro();
  mpu.MPU_get_Accelerometer();
}

// เชื่อมต่อ WiFi แบบ STA mode และเปิด UDP port สำหรับรับ/ส่งข้อมูลกับ ROS2
void startWiFi()
{
  // ล้างสถานะ WiFi เดิมก่อนเริ่มเชื่อมต่อใหม่
  WiFi.disconnect(true, true);
  delay(100);

  // ใช้ STA mode เท่านั้น: ESP32 join เข้า WiFi ที่มีอยู่แล้ว
  WiFi.mode(WIFI_STA);

#if IROB_WIFI_USE_STATIC_STA_IP
  // ใช้ IP static ถ้าเปิด IROB_WIFI_USE_STATIC_STA_IP เป็น 1
  WiFi.config(IROB_STA_LOCAL_IP, IROB_STA_GATEWAY, IROB_STA_SUBNET);
#endif

  WiFi.begin(IROB_WIFI_SSID, IROB_WIFI_PASSWORD);

  // รอจนกว่า ESP32 จะต่อ WiFi สำเร็จ ก่อนเริ่มรับ/ส่ง UDP
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // เปิด UDP socket ที่ port เดียวกับ ROS2 bridge
  udp.stop();
  udp.begin(IROB_UDP_PORT);

  Serial.print("Sending UDP packets to ");
  Serial.print(ROS2_NODE_IP);
  Serial.print(":");
  Serial.println(IROB_UDP_PORT);
}

// เตรียม motor PID, หยุด motor, และ init MPU6050
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

// สร้าง feedback packet แล้วส่งไปยัง ROS2 bridge node
void sendPacketToRos()
{
  // อัปเดตข้อมูลทุก field ก่อนส่ง
  fillRobotData(txPacket);

  // ส่ง binary packet ขนาด 32 bytes ผ่าน UDP
  udp.beginPacket(ROS2_NODE_IP, IROB_UDP_PORT);
  udp.write(reinterpret_cast<const uint8_t *>(&txPacket), sizeof(txPacket));
  udp.endPacket();
}

void setup()
{
  // Serial ใช้แสดงสถานะ/debug เช่น IP ที่ ESP32 ได้รับ
  Serial.begin(115200);

  // เตรียม motor/encoder/IMU ก่อนเริ่ม network
  setupRobotHardware();

  // join WiFi ตอนเริ่มระบบ
  startWiFi();
}

void loop()
{
  // ถ้า WiFi หลุด ให้หยุด motor แล้วเชื่อมต่อใหม่
  if (WiFi.status() != WL_CONNECTED) {
    clearMotorTargets();
    stopMotorOutputs();
    startWiFi();
  }

  // รับคำสั่งควบคุมจาก ROS2
  receiveCommandFromRos();

  // scheduler แบบไม่ block สำหรับรอบควบคุม/feedback 100 Hz
  const uint32_t now = millis();
  if ((uint32_t)(now - lastSendMs) >= IROB_SEND_PERIOD_MS) {
    lastSendMs = now;
    applyCommandTimeout();
    updateRobotHardware();
    sendPacketToRos();
  }
}
