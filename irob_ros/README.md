# irob_ros

แพ็กเกจ ROS2 Humble สำหรับรับ feedback จาก ESP32 ผ่าน WiFi/UDP แล้ว publish ไปที่ topic `iROB_controller` และส่ง command จาก ROS2 กลับไปควบคุม ESP32 ผ่าน topic `iROB_command`

## Protocol

### ESP32 -> ROS2 Feedback

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

### ROS2 -> ESP32 Command

ROS2 node `iROB_CMD` จะ subscribe topic `iROB_command` แล้วส่ง UDP command packet ไปหา ESP32 ขนาด `13 bytes` ตามส่วน RX ของ `app_ros_comm.ino`

| Bytes | Field | Type | ความหมาย |
| --- | --- | --- | --- |
| 0-1 | `RB` header | `uint8[2]` | header ของ command packet |
| 2 | `reg` | `uint8` | command/register เช่น `0` ปกติ หรือ `170` สำหรับ emergency stop |
| 3 | `ctk` | `uint8` | field สำรองตาม protocol เดิม |
| 4-11 | `motor1_ctrl..motor4_ctrl` | `int16[4]` | target RPM ของมอเตอร์ |
| 12 | `cmd_data_pc` | `uint8` | command data จากฝั่ง ROS2/PC |

ลำดับมอเตอร์ใน command packet คือ `LF`, `LB`, `RB`, `RF`

ค่า `reg` ที่ใช้ตอนนี้:

- `0` หรือ `REG_NULL` คือ command ปกติ
- `170` หรือ `0xAA` คือ emergency stop

ESP32 จะหยุด target RPM เป็น `0` อัตโนมัติถ้าไม่ได้รับ command จาก ROS2 เกิน `500 ms`

## ESP32

เปิดไฟล์ `arduino/iROB_ros/iROB_ros.ino` ด้วย Arduino IDE แล้ว upload ลง ESP32

โฟลเดอร์ `arduino/iROB_ros/` ต้องมีไฟล์เหล่านี้อยู่ด้วย เพื่อให้ Arduino IDE compile ได้:

```text
arduino/iROB_ros/
├── iROB_ros.ino
├── Inc/
│   ├── iROB_Motor.h
│   └── MPU6050_E12.h
└── src/
    ├── iROB_Motor.cpp
    └── MPU6050_E12.cpp
```

ค่า WiFi เริ่มต้น:

- ESP32 join WiFi SSID `ABU_robot2027`
- Password คือ `ABU_robot67`
- ESP32 รับ IP จาก router/DHCP เป็นค่าเริ่มต้น
- ESP32 broadcast UDP packet ไปที่ `192.168.1.255:6767`
- ESP32 listen UDP command จาก ROS2 ที่ port `6767`

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

run node สำหรับส่ง command ไปควบคุม ESP32:

```bash
IROB_ESP32_IP=192.168.1.100 ros2 run irob_ros iROB_CMD
```

ถ้า IP ของ ESP32 ไม่ใช่ `192.168.1.100` ให้เปลี่ยนค่า `IROB_ESP32_IP` ให้ตรงกับ IP ที่ Serial Monitor แสดง

ส่งคำสั่ง target RPM ไปยัง ESP32:

```bash
ros2 topic pub -r 20 /iROB_command irob_ros/msg/IROBCommand "{reg: 0, ctk: 0, motor1_ctrl: 100, motor2_ctrl: 100, motor3_ctrl: 100, motor4_ctrl: 100, cmd_data_pc: 0}"
```

หยุดมอเตอร์แบบปกติ:

```bash
ros2 topic pub -1 /iROB_command irob_ros/msg/IROBCommand "{reg: 0, ctk: 0, motor1_ctrl: 0, motor2_ctrl: 0, motor3_ctrl: 0, motor4_ctrl: 0, cmd_data_pc: 0}"
```

ส่ง emergency stop:

```bash
ros2 topic pub -1 /iROB_command irob_ros/msg/IROBCommand "{reg: 170, ctk: 0, motor1_ctrl: 0, motor2_ctrl: 0, motor3_ctrl: 0, motor4_ctrl: 0, cmd_data_pc: 0}"
```

ดูโครงสร้าง message:

```bash
ros2 interface show irob_ros/msg/IROBController
ros2 interface show irob_ros/msg/IROBCommand
```

ถ้าต้องการใช้ UDP port อื่น ให้ตั้ง port ฝั่ง ESP32 และ ROS2 node ให้ตรงกัน:

```bash
IROB_UDP_PORT=6768 ros2 run irob_ros iROB_ESP
IROB_UDP_PORT=6768 IROB_ESP32_IP=192.168.1.100 ros2 run irob_ros iROB_CMD
```

## ลำดับการ Run

1. ต่อคอมพิวเตอร์ ROS2 เข้ากับ WiFi `ABU_robot2027`
2. Upload และเริ่มรัน `arduino/iROB_ros/iROB_ros.ino` บน ESP32
3. เปิด Serial Monitor ที่ `115200` แล้วดูว่า ESP32 ได้ IP แล้ว
4. เปิด terminal แรก แล้ว run feedback node:

```bash
cd ~/irob_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run irob_ros iROB_ESP
```

5. เปิด terminal ที่สอง แล้ว run command node:

```bash
cd ~/irob_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
IROB_ESP32_IP=192.168.1.100 ros2 run irob_ros iROB_CMD
```

6. เปิด terminal ที่สาม เพื่อดูข้อมูลจาก feedback topic:

```bash
cd ~/irob_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /iROB_controller
```

7. ส่งคำสั่งควบคุมมอเตอร์จาก terminal ใหม่:

```bash
cd ~/irob_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic pub -r 20 /iROB_command irob_ros/msg/IROBCommand "{reg: 0, ctk: 0, motor1_ctrl: 100, motor2_ctrl: 100, motor3_ctrl: 100, motor4_ctrl: 100, cmd_data_pc: 0}"
```

## Troubleshooting

ถ้า `/iROB_controller` ไม่มีข้อมูลหรือไม่ update:

- เช็คว่า ESP32 และคอมพิวเตอร์ ROS2 อยู่ WiFi วงเดียวกัน
- เช็ค Serial Monitor ของ ESP32 ต้องเห็น `Sending UDP packets to 192.168.1.255:6767`
- เช็ค terminal ฝั่ง ROS2 ต้องเห็น `iROB_ESP listening on UDP 0.0.0.0:6767`
- เช็ค firewall ว่าอนุญาต incoming UDP port `6767`
- เช็คว่า `IROB_UDP_PORT` ใน ESP32 ตรงกับ port ของ ROS2 node
- ถ้า network ไม่รับ broadcast ให้ตั้ง `ROS2_NODE_IP` เป็น IP ของคอม ROS2 โดยตรง

ถ้า ESP32 ไม่ขยับตาม command:

- เช็คว่า run node `iROB_CMD` อยู่
- เช็คว่า `IROB_ESP32_IP` ตรงกับ IP ที่ ESP32 แสดงใน Serial Monitor
- เช็คว่า publish `/iROB_command` แบบต่อเนื่อง เช่นใช้ `-r 20`
- เช็คว่า firewall อนุญาต outgoing UDP ไปยัง IP ของ ESP32 ที่ port `6767`
- ถ้าขยับแค่ครึ่งวินาทีแล้วหยุด แปลว่า command timeout ทำงาน ให้ publish command ต่อเนื่อง

ถ้า `ros2 run irob_ros iROB_ESP` หา executable ไม่เจอ ให้ build และ source workspace ใหม่:

```bash
cd ~/irob_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select irob_ros
source install/setup.bash
```

## ควบคุมด้วย Keyboard

ถ้าต้องการควบคุม ESP32 ด้วย keyboard ให้ใช้ package `irob_keyboard_control` ร่วมกับ package นี้

build ทั้งสอง package:

```bash
cd ~/irob_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select irob_ros irob_keyboard_control
source install/setup.bash
```

run launch file:

```bash
ros2 launch irob_keyboard_control keyboard_control.launch.py esp32_ip:=192.168.1.100
```

เปิด `rqt_graph` พร้อม launch:

```bash
ros2 launch irob_keyboard_control keyboard_control.launch.py esp32_ip:=192.168.1.100 start_rqt_graph:=true
```
