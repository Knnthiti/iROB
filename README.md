# iROB ESP32 Robot

โปรเจกต์นี้เป็นโค้ดควบคุมหุ่นยนต์ล้อ mecanum 4 ล้อบน ESP32 โดยแยก library ไว้ใน `Inc/` และ `Src/` และมี sketch ตัวอย่างอยู่ใน `Project/`

## โครงสร้างโปรเจกต์

- `Inc/iROB_Motor.h` และ `Src/iROB_Motor.cpp` - ควบคุม motor driver, encoder feedback, RPM, duty cycle, PID speed และ ramp count
- `Inc/Inverse_Kinematics.h` และ `Src/Inverse_Kinematics.cpp` - แปลงคำสั่งความเร็ว `Vx`, `Vy`, `wz` เป็นความเร็วล้อ mecanum หน่วย rad/s
- `Inc/espnow_ROBOT.h` และ `Src/espnow_ROBOT.cpp` - ตั้งค่า Wi-Fi station mode, init ESP-NOW, ส่งข้อมูล และรับ callback จาก sketch
- `Inc/MPU6050_E12.h` และ `Src/MPU6050_E12.cpp` - อ่าน gyro/accelerometer จาก MPU6050 และ integrate เป็นองศา/radian
- `Project/` - sketch สำหรับทดสอบแยกส่วนและ sketch รวมทั้งหมด โดยแต่ละ project มี `Inc/` สำหรับ header และ `src/` สำหรับ source ของตัวเอง

## Sketch ใน Project

- `Project/iROB_motor/iROB_motor.ino` - ตัวอย่างสั่ง motor แบบ duty cycle โดยเริ่มจาก `iROB.Motor_DutyCycle_LF(0);` และมีตัวอย่างทดสอบทั้ง 4 ล้อใน `loop()`
- `Project/iROB_PID/iROB_PID.ino` - ตัวอย่าง PID speed motor LF โดยใช้ `iROB.Motor_Speed_LF(iROB.getRad_s_to_RPM(_Kinematic.Wheel.w_LF), iROB.getRPM(_LF));`
- `Project/iROB_espnow/iROB_espnow.ino` - ตัวอย่างรับข้อมูล ESP-NOW โดยใช้ `esp_now_register_recv_cb(OnDataRecv);`
- `Project/iROB/iROB.ino` - Run All: รับ joystick ผ่าน ESP-NOW, คำนวณ inverse kinematics, สั่ง PID ทั้ง 4 ล้อ และหยุดมอเตอร์เมื่อสัญญาณขาด

## Library ที่ต้องใช้

- ESP32 board package สำหรับ Arduino IDE (รองรับ Arduino-ESP32 core 2.x และ 3.x ในส่วน LEDC/PWM และ ESP-NOW callback)
- `ESP32Encoder`
- Library มาตรฐานจาก ESP32 Arduino core: `WiFi`, `esp_now`, `Wire`, `ledc`

## วิธีใช้งาน

1. เปิด Arduino IDE
2. เลือกบอร์ด ESP32 ที่ใช้งานจริง
3. เปิด sketch ที่ต้องการจาก `Project/<ชื่อโปรเจกต์>/<ชื่อโปรเจกต์>.ino`
4. ติดตั้ง library `ESP32Encoder` ถ้ายังไม่มี
5. Upload ลง ESP32
6. เปิด Serial Monitor ที่ `115200`

หมายเหตุ: sketch ใน `Project/` มีโฟลเดอร์ `Inc/` และ `src/` ของตัวเอง เพื่อให้ Arduino IDE copy ไป build ใน temp folder แล้วยังหา header/source เจอ ไม่ต้องใช้ path แบบ `../../Inc` หรือ `../../Src`

## ข้อมูลควบคุมหลัก

- Duty cycle ใช้ช่วง `-4095` ถึง `4095` เพราะ PWM เป็น 12-bit
- Feedback loop ตั้งไว้ที่ `100 Hz` ดังนั้นส่วนที่อ่าน RPM/PID ควรรันทุก `10 ms`
- Wheel order ใน array คือ `LF`, `LB`, `RF`, `RB`
- ESP-NOW receive callback จะ copy payload เข้า struct `Receive_ESPNOW`
- ถ้า `Project/iROB/iROB.ino` ไม่ได้รับ packet เกิน `500 ms` ระบบจะสั่ง duty cycle ทุกล้อเป็น `0`

## การปรับค่า

- ปรับ PID ได้ที่ `Setup_PID_Wheel(Kp, Ki, Kd, min_speed, max_speed, wheel)`
- ปรับขนาดหุ่นและรัศมีล้อได้ที่ `Kinematic _Kinematic(0.23f, 0.23f, 0.06f);`
- ปรับช่วง joystick mapping ได้ที่ `_map(..., -2.5f, 2.5f)` และ `_map(..., -3.0f, 3.0f)`
- ถ้าต้องการ debug ESP-NOW ให้เปิด `#define ESPNOW` ใน `Inc/espnow_ROBOT.h`
