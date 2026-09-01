# ROS2 -> ESP32 UDP command packet.
# Layout matches the 13-byte RX section of app_ros_comm.ino.

uint8 HEADER_R=82
uint8 HEADER_B=66
uint8 REG_NULL=0
uint8 REG_ESTOP=170

uint8 reg
uint8 ctk

# Original command order: LF, LB, RB, RF.
int16 motor1_ctrl
int16 motor2_ctrl
int16 motor3_ctrl
int16 motor4_ctrl

uint8 cmd_data_pc
