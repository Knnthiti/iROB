# ข้อมูลคำสั่งจาก ROS2 ไปยัง ESP32
# layout นี้ตรงกับ command packet ขนาด 13 bytes ของฝั่งรับใน app_ros_comm.ino

# ค่าคงที่ของ header และ register ที่ใช้ใน protocol
uint8 HEADER_R=82
uint8 HEADER_B=66
uint8 REG_NULL=0
uint8 REG_ESTOP=170

# register คำสั่ง เช่น 0 = ปกติ, 170 = emergency stop
uint8 reg

# control token สำรองไว้ให้ protocol เดิม
uint8 ctk

# คำสั่ง motor เรียงตาม protocol เดิม: LF, LB, RB, RF
int16 motor1_ctrl
int16 motor2_ctrl
int16 motor3_ctrl
int16 motor4_ctrl

# command byte จาก PC/ROS2 สำรองไว้ให้ protocol เดิม
uint8 cmd_data_pc
