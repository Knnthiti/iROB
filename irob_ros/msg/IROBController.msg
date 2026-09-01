# ข้อมูล feedback จาก ESP32 ไปยัง ROS2
# layout นี้ตรงกับ feedback packet ขนาด 32 bytes ของฝั่งส่งใน app_ros_comm.ino

# ค่าคงที่ของ header feedback packet
uint8 HEADER_J=74
uint8 HEADER_B=66

# header จริงที่รับมาจาก ESP32 ควรเป็น 'J' และ 'B'
uint8 header0
uint8 header1

# สถานะจาก MCU เช่น ปกติ หรือ emergency stop
uint8 cmd_data_mcu

# feedback motor เรียงตาม protocol เดิม: LF, LB, RB, RF
int16 motor1_fb
int16 motor2_fb
int16 motor3_fb
int16 motor4_fb

# ความเร็วจาก mouse/optical-flow ถ้ามี sensor
int8 mouse_x_vel
int8 mouse_y_vel

# ค่า gyro raw จาก IMU
int16 gyro_x_raw
int16 gyro_y_raw
int16 gyro_z_raw

# ค่า magnetometer raw ถ้า hardware มี sensor
int16 mag_x_raw
int16 mag_y_raw
int16 mag_z_raw

# ค่า accelerometer raw จาก IMU
int16 acc_x_raw
int16 acc_y_raw
int16 acc_z_raw

# checksum ที่มากับ packet
uint8 cks
