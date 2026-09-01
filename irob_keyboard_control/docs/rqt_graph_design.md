# rqt_graph Design

ภาพรวม graph ที่ควรเห็นเมื่อรัน launch file:

```text
┌────────────────────┐      /iROB_command       ┌────────────────────┐
│ /iROB_keyboard     │ ───────────────────────> │ /iROB_command_bridge│
│ irob_keyboard      │                          │ iROB_CMD            │
└────────────────────┘                          └─────────┬──────────┘
                                                           │ UDP 13 bytes
                                                           v
                                                    ┌──────────────┐
                                                    │ ESP32 iROB   │
                                                    │ iROB_ros.ino │
                                                    └──────┬───────┘
                                                           │ UDP 32 bytes
                                                           v
┌────────────────────┐      /iROB_controller      ┌────────────────────┐
│ /rqt_graph         │ <────────────────────────── │ /iROB_feedback_bridge│
│ monitor only       │                            │ iROB_ESP            │
└────────────────────┘                            └────────────────────┘
```

## Node และ Topic

| Node | หน้าที่ |
| --- | --- |
| `/iROB_keyboard` | อ่านปุ่ม keyboard แล้ว publish topic `/iROB_command` |
| `/iROB_command_bridge` | subscribe `/iROB_command` แล้วส่ง UDP command packet 13 bytes ไป ESP32 |
| `ESP32 iROB` | รับ command, ควบคุม motor, ส่ง feedback UDP 32 bytes กลับ ROS2 |
| `/iROB_feedback_bridge` | รับ UDP feedback จาก ESP32 แล้ว publish topic `/iROB_controller` |
| `/rqt_graph` | แสดง graph เพื่อ debug การเชื่อม node/topic |

| Topic | Type | ทิศทาง |
| --- | --- | --- |
| `/iROB_command` | `irob_ros/msg/IROBCommand` | ROS2 keyboard -> command bridge |
| `/iROB_controller` | `irob_ros/msg/IROBController` | feedback bridge -> ROS2 tools |

## เปิด rqt_graph

รันพร้อม launch:

```bash
ros2 launch irob_keyboard_control keyboard_control.launch.py start_rqt_graph:=true
```

หรือเปิดอีก terminal:

```bash
rqt_graph
```

ใน `rqt_graph` ให้กด refresh หลังจากทุก node เริ่มทำงานแล้ว
