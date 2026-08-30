# ESP32 -> ROS2 UDP packet. Layout matches the 32-byte TX section of app_ros_comm.ino.

uint8 HEADER_J=74
uint8 HEADER_B=66

uint8 header0
uint8 header1

uint8 cmd_data_mcu

# Original wheel order: LF, LB, RB, RF.
int16 motor1_fb
int16 motor2_fb
int16 motor3_fb
int16 motor4_fb

int8 mouse_x_vel
int8 mouse_y_vel

int16 gyro_x_raw
int16 gyro_y_raw
int16 gyro_z_raw

int16 mag_x_raw
int16 mag_y_raw
int16 mag_z_raw

int16 acc_x_raw
int16 acc_y_raw
int16 acc_z_raw

uint8 cks
