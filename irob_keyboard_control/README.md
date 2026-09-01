# irob_keyboard_control

แพ็กเกจ ROS2 Humble สำหรับควบคุม ESP32 iROB ด้วย keyboard

แพ็กเกจนี้ทำงานร่วมกับ `irob_ros`:

- `irob_keyboard` อ่านปุ่ม keyboard แล้ว publish topic `/iROB_command`
- `iROB_CMD` รับ `/iROB_command` แล้วส่ง UDP command packet ไป ESP32
- `iROB_ESP` รับ feedback packet จาก ESP32 แล้ว publish `/iROB_controller`

## Build

วางทั้งสอง package ใน ROS2 workspace:

```bash
mkdir -p ~/irob_ws/src
cp -r irob_ros ~/irob_ws/src/
cp -r irob_keyboard_control ~/irob_ws/src/
cd ~/irob_ws
```

build:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select irob_ros irob_keyboard_control
source install/setup.bash
```

## Run ด้วย Launch File

เปิด ESP32 ก่อน แล้วดู IP จาก Serial Monitor เช่น:

```text
ESP32 IP: 192.168.1.100
```

run launch:

```bash
ros2 launch irob_keyboard_control keyboard_control.launch.py esp32_ip:=192.168.1.100
```

หมายเหตุ: keyboard node ต้องรันใน terminal ที่รับปุ่มได้จริง ถ้ารันผ่าน IDE แล้วกดปุ่มไม่เข้า ให้เปิด terminal ใหม่แล้วใช้ `ros2 run irob_keyboard_control irob_keyboard`

ถ้าต้องการเปิด `rqt_graph` พร้อมกัน:

```bash
ros2 launch irob_keyboard_control keyboard_control.launch.py esp32_ip:=192.168.1.100 start_rqt_graph:=true
```

ปรับความเร็วเริ่มต้น:

```bash
ros2 launch irob_keyboard_control keyboard_control.launch.py esp32_ip:=192.168.1.100 speed_rpm:=150 turn_rpm:=120
```

## ปุ่มควบคุม

| Key | คำสั่ง |
| --- | --- |
| `W` | เดินหน้า |
| `S` | ถอยหลัง |
| `A` | สไลด์ซ้าย |
| `D` | สไลด์ขวา |
| `Q` | หมุนซ้าย |
| `E` | หมุนขวา |
| `Space` หรือ `X` | หยุด |
| `Z` | emergency stop |
| `+` หรือ `=` | เพิ่ม speed/turn RPM |
| `-` หรือ `_` | ลด speed/turn RPM |
| `Ctrl+C` | ออกจากโปรแกรม และส่ง stop |

## Topic

keyboard node publish:

```text
/iROB_command
```

message type:

```text
irob_ros/msg/IROBCommand
```

feedback จาก ESP32 อยู่ที่:

```text
/iROB_controller
```

## rqt_graph

เอกสาร graph ที่คาดว่าจะเห็นอยู่ที่:

```text
docs/rqt_graph_design.md
docs/rqt_graph.dot
```

ภาพรวม graph:

```text
/iROB_keyboard -> /iROB_command -> /iROB_command_bridge -> ESP32
ESP32 -> /iROB_feedback_bridge -> /iROB_controller
```

เปิดดู graph จริง:

```bash
rqt_graph
```

หรือ:

```bash
ros2 launch irob_keyboard_control keyboard_control.launch.py esp32_ip:=192.168.1.100 start_rqt_graph:=true
```

## หมายเหตุ

ถ้า ESP32 ขยับแล้วหยุดเองหลังประมาณ `500 ms` แปลว่า command timeout ทำงาน ให้เช็คว่า `irob_keyboard` และ `iROB_CMD` ยังรันอยู่ และ keyboard node publish `/iROB_command` ต่อเนื่องอยู่
