# irob_ros

แพ็กเกจ ROS2 Humble สำหรับรับ packet จาก ESP32 ผ่าน WiFi/UDP แล้ว publish ข้อมูลไปที่ topic `iROB_controller`

## Protocol

ESP32 จะส่ง UDP packet ทุก `10 ms` หรือประมาณ `100 Hz` ผ่าน port `6767` ขนาด packet คือ `32 bytes` และจัดวางข้อมูลให้ตรงกับส่วน TX ของ `app_ros_comm.ino`

| Bytes | Field | Type | ความหมาย |
| --- | --- | --- | --- |
| 0-1 | `JB` header | `uint8[2]` | header ของ packet |
| 2 | `cmd_data_mcu` | `uint8` | status/command reply จาก ESP32 |
| 3-10 | `motor1_fb..motor4_fb` | `int16[4]` | feedback ความเร็วมอเตอร์ |
| 11-12 | `mouse_x_vel`, `mouse_y_vel` | `int8[2]` | ความเร็วจาก mouse/optical flow ถ้ามี |
| 13-18 | `gyro_x_raw..gyro_z_raw` | `int16[3]` | ค่า gyro raw |
| 19-24 | `mag_x_raw..mag_z_raw` | `int16[3]` | ค่า magnetometer raw |
| 25-30 | `acc_x_raw..acc_z_raw` | `int16[3]` | ค่า accelerometer raw |
| 31 | `cks` | `uint8` | XOR checksum |

ลำดับล้อใน packet คือ `LF`, `LB`, `RB`, `RF` ตาม protocol เดิม

## ESP32

เปิดไฟล์ `arduino/iROB_ros/iROB_ros.ino` ด้วย Arduino IDE แล้ว upload ลง ESP32

ค่า WiFi เริ่มต้น:

- ESP32 join WiFi SSID `ABU_robot2027`
- Password คือ `ABU_robot67`
- ESP32 รับ IP จาก router/DHCP เป็นค่าเริ่มต้น
- ESP32 broadcast UDP packet ไปที่ `192.168.1.255:6767`

ให้ ESP32 และคอมพิวเตอร์ที่รัน ROS2 ต่อ WiFi วงเดียวกันคือ `ABU_robot2027`

หลัง upload แล้วเปิด Serial Monitor ที่ `115200` เพื่อดู IP ของ ESP32:

```text
ESP32 IP: 192.168.1.xxx
```

ถ้าต้องการ fix IP ของ ESP32 ให้แก้ใน sketch:

```cpp
#define IROB_WIFI_USE_STATIC_STA_IP 1
static const IPAddress IROB_STA_LOCAL_IP(192, 168, 1, 68);
```

ถ้า broadcast ใช้ไม่ได้ ให้แก้ `ROS2_NODE_IP` ใน sketch เป็น IP ของคอม ROS2 โดยตรง

## การใช้งาน ROS2 Project

แพ็กเกจนี้เป็น ROS2 Humble package ปกติ ให้วางไว้ในโฟลเดอร์ `src` ของ ROS2 workspace

ตัวอย่างสร้าง workspace:

```bash
mkdir -p ~/irob_ws/src
cp -r irob_ros ~/irob_ws/src/
cd ~/irob_ws
```

build package:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select irob_ros
source install/setup.bash
```

run node รับ UDP จาก ESP32:

```bash
ros2 run irob_ros iROB_ESP
```

node นี้จะ listen UDP ที่ port `6767` แล้ว publish ข้อมูลที่ decode แล้วไปที่:

```text
/iROB_controller
```

ชนิดของ topic คือ:

```text
irob_ros/msg/IROBController
```

ตรวจสอบว่า topic ถูกสร้างหรือยัง:

```bash
ros2 topic list
ros2 topic info /iROB_controller
```

ดูข้อมูลจริงจาก ESP32:

```bash
ros2 topic echo /iROB_controller
```

ดูโครงสร้าง message:

```bash
ros2 interface show irob_ros/msg/IROBController
```

ถ้าต้องการใช้ UDP port อื่น ให้ตั้ง port ฝั่ง ESP32 และ ROS2 node ให้ตรงกัน:

```bash
IROB_UDP_PORT=6768 ros2 run irob_ros iROB_ESP
```

## ลำดับการ Run

1. ต่อคอมพิวเตอร์ ROS2 เข้ากับ WiFi `ABU_robot2027`
2. Upload และเริ่มรัน `arduino/iROB_ros/iROB_ros.ino` บน ESP32
3. เปิด Serial Monitor ที่ `115200` แล้วดูว่า ESP32 ได้ IP แล้ว
4. เปิด terminal แล้ว run ROS2 node:

```bash
cd ~/irob_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run irob_ros iROB_ESP
```

5. เปิดอีก terminal เพื่อดูข้อมูลจาก topic:

```bash
cd ~/irob_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /iROB_controller
```

## Troubleshooting

ถ้า `/iROB_controller` ไม่มีข้อมูลหรือไม่ update:

- เช็คว่า ESP32 และคอมพิวเตอร์ ROS2 อยู่ WiFi วงเดียวกัน
- เช็ค Serial Monitor ของ ESP32 ต้องเห็น `Sending UDP packets to 192.168.1.255:6767`
- เช็ค terminal ฝั่ง ROS2 ต้องเห็น `iROB_ESP listening on UDP 0.0.0.0:6767`
- เช็ค firewall ว่าอนุญาต incoming UDP port `6767`
- เช็คว่า `IROB_UDP_PORT` ใน ESP32 ตรงกับ port ของ ROS2 node
- ถ้า network ไม่รับ broadcast ให้ตั้ง `ROS2_NODE_IP` เป็น IP ของคอม ROS2 โดยตรง

ถ้า `ros2 run irob_ros iROB_ESP` หา executable ไม่เจอ ให้ build และ source workspace ใหม่:

```bash
cd ~/irob_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select irob_ros
source install/setup.bash
```
