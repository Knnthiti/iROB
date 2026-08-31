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

- ESP32 joins WiFi SSID `ABU_robot2027`
- Password is `ABU_robot67`
- ESP32 gets its IP from the router/DHCP by default
- ESP32 broadcasts UDP packets to `192.168.1.255:6767`

Connect both the ESP32 and ROS2 computer to `ABU_robot2027`, then run the ROS2 node.

The ESP32 prints its assigned IP in Serial Monitor:

```text
ESP32 IP: 192.168.1.xxx
```

To force a fixed ESP32 IP, set `IROB_WIFI_USE_STATIC_STA_IP` to `1` in the sketch and edit `IROB_STA_LOCAL_IP`.

## ROS2 Project Usage

This package is a normal ROS2 Humble package. Put it inside the `src` folder of a ROS2 workspace:

```bash
mkdir -p ~/irob_ws/src
cp -r irob_ros ~/irob_ws/src/
cd ~/irob_ws
```

Build the package:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select irob_ros
source install/setup.bash
```

Run the UDP bridge node:

```bash
ros2 run irob_ros iROB_ESP
```

The node listens for ESP32 UDP packets on port `6767` and publishes decoded data to:

```text
/iROB_controller
```

The topic type is:

```text
irob_ros/msg/IROBController
```

Check that the topic exists:

```bash
ros2 topic list
ros2 topic info /iROB_controller
```

Echo live robot data:

```bash
ros2 topic echo /iROB_controller
```

Show the message definition:

```bash
ros2 interface show irob_ros/msg/IROBController
```

Use another UDP port if needed. The ESP32 sketch and ROS2 node must use the same port:

```bash
IROB_UDP_PORT=6768 ros2 run irob_ros iROB_ESP
```

## ROS2 Run Order

1. Connect the ROS2 computer to WiFi `ABU_robot2027`.
2. Upload and start `arduino/iROB_ros/iROB_ros.ino` on the ESP32.
3. Open Serial Monitor at `115200` and confirm the ESP32 prints its IP.
4. Source and run the ROS2 node:

```bash
cd ~/irob_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run irob_ros iROB_ESP
```

5. In another terminal, check the topic:

```bash
cd ~/irob_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /iROB_controller
```

## Troubleshooting

If `/iROB_controller` does not update:

- Confirm the ESP32 and ROS2 computer are on the same WiFi network.
- Confirm the ESP32 Serial Monitor shows `Sending UDP packets to 192.168.1.255:6767`.
- Confirm the ROS2 node prints `iROB_ESP listening on UDP 0.0.0.0:6767`.
- Check that firewall rules allow incoming UDP traffic on port `6767`.
- Make sure the ESP32 `IROB_UDP_PORT` value matches the ROS2 node port.
- If broadcast does not work on the network, set `ROS2_NODE_IP` in the ESP32 sketch to the ROS2 computer IP.

If `ros2 run irob_ros iROB_ESP` cannot find the executable:

```bash
cd ~/irob_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select irob_ros
source install/setup.bash
```
