# irob_ros

ROS2 Humble package for receiving iROB ESP32 WiFi packets and publishing them to `iROB_controller`.

## Protocol

The ESP32 sends one UDP packet every 10 ms on port `6767`. The packet is 32 bytes and matches the TX layout from `app_ros_comm.ino`:

| Bytes | Field | Type |
| --- | --- | --- |
| 0-1 | `JB` header | `uint8[2]` |
| 2 | `cmd_data_mcu` | `uint8` |
| 3-10 | `motor1_fb..motor4_fb` | `int16[4]` |
| 11-12 | `mouse_x_vel`, `mouse_y_vel` | `int8[2]` |
| 13-18 | `gyro_x_raw..gyro_z_raw` | `int16[3]` |
| 19-24 | `mag_x_raw..mag_z_raw` | `int16[3]` |
| 25-30 | `acc_x_raw..acc_z_raw` | `int16[3]` |
| 31 | `cks` | XOR checksum |

Wheel order follows the original packet: LF, LB, RB, RF.

## ESP32

Open `arduino/iROB_ros/iROB_ros.ino` in Arduino IDE and upload it to the ESP32.

Default WiFi behavior:

- ESP32 creates AP SSID `ABU_robot2027`
- Password is `ABU_robot67`
- ESP32 AP IP is `192.168.1.67`
- ESP32 broadcasts UDP packets to `192.168.1.255:6767`

Connect the ROS2 computer to `ABU_robot2027`, then run the ROS2 node.

If `192.168.1.67` is the ROS2 computer IP instead of the ESP32 IP, change `IROB_WIFI_AP_MODE` to `0` in the sketch and set `ROS2_NODE_IP` to `192.168.1.67`. In STA mode the sketch uses DHCP by default, so it will not take the same IP as the ROS2 computer.

## ROS2 Humble

Copy or keep this package in a ROS2 workspace, then build:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select irob_ros
source install/setup.bash
```

Run the UDP bridge:

```bash
ros2 run irob_ros iROB_ESP
```

Check the topic:

```bash
ros2 topic echo /iROB_controller
```

Use another UDP port if needed:

```bash
IROB_UDP_PORT=6768 ros2 run irob_ros iROB_ESP
```
