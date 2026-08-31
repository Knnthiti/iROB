#include "../Inc/MPU6050_E12.h"

void MPU6050::MPU_i2c_writeReg8(uint8_t reg, uint8_t data8) {
  // Send a register address followed by one configuration byte.
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data8);
  Wire.endTransmission();
}

void MPU6050::MPU_init() {
  // Register reference:
  // https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf

  // Wake the sensor from sleep mode.
  MPU_i2c_writeReg8(REG_PWRMGMT_1, 0x00);

  // Set sample-rate divider. 0 keeps the base gyro sample rate.
  MPU_i2c_writeReg8(REG_SMPRT_DIV, 0);

  // Digital low-pass filter: accel around 44 Hz, gyro around 42 Hz.
  MPU_i2c_writeReg8(REG_DLPFCONF, 0x03);

  // Accelerometer full-scale range: +/- 16 g.
  MPU_i2c_writeReg8(REG_ACCLCONF, 3 << 3);

  // Gyroscope full-scale range: +/- 2000 deg/s.
  MPU_i2c_writeReg8(REG_GYROCONF, 3 << 3);
}

void MPU6050::MPU_get_Accelerometer() {
  // Read six bytes starting at ACCEL_XOUT_H: X, Y, Z high/low pairs.
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);

  mpuData.Ax = Wire.read() << 8 | Wire.read();
  mpuData.Ay = Wire.read() << 8 | Wire.read();
  mpuData.Az = Wire.read() << 8 | Wire.read();
}

void MPU6050::MPU_get_gyro() {
  // Read six bytes starting at GYRO_XOUT_H: X, Y, Z high/low pairs.
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);

  mpuData.gx = Wire.read() << 8 | Wire.read();
  mpuData.gy = Wire.read() << 8 | Wire.read();
  mpuData.gz = Wire.read() << 8 | Wire.read();
}

void MPU6050::gyro_calib() {
  // Keep the robot still while this runs. It averages raw gyro bias.
  float Offset_gx = 0;
  float Offset_gy = 0;
  float Offset_gz = 0;

  for (uint16_t i = 0; i < 2000; i++) {
    MPU_get_gyro();
    Offset_gx += mpuData.gx;
    Offset_gy += mpuData.gy;
    Offset_gz += mpuData.gz;
    delay(1);
  }

  mpuData_Offset.gx = Offset_gx / 2000.0f;
  mpuData_Offset.gy = Offset_gy / 2000.0f;
  mpuData_Offset.gz = Offset_gz / 2000.0f;
}

void MPU6050::Degree() {
  MPU_get_gyro();

  // Convert raw gyro values to deg/s using the 2000 deg/s sensitivity.
  Angular.Deg_x += (float)((mpuData.gx - mpuData_Offset.gx) / (16.4f * freq_MPU6050));
  Angular.Deg_y += (float)((mpuData.gy - mpuData_Offset.gy) / (16.4f * freq_MPU6050));
  Angular.Deg_z += (float)((mpuData.gz - mpuData_Offset.gz) / (16.4f * freq_MPU6050));

  // Deadband tiny changes to reduce drift noise.
  if (abs(Angular.Deg_x - Part_Angular.Deg_x) < 0.002) {
    Angular.Deg_x = Part_Angular.Deg_x;
  } else {
    Part_Angular.Deg_x = Angular.Deg_x;
  }

  if (abs(Angular.Deg_y - Part_Angular.Deg_y) < 0.002) {
    Angular.Deg_y = Part_Angular.Deg_y;
  } else {
    Part_Angular.Deg_y = Angular.Deg_y;
  }

  if (abs(Angular.Deg_z - Part_Angular.Deg_z) < 0.002) {
    Angular.Deg_z = Part_Angular.Deg_z;
  } else {
    Part_Angular.Deg_z = Angular.Deg_z;
  }
}

void MPU6050::Radian() {
  MPU_get_gyro();

  // Convert integrated gyro angle to radians.
  Angular.Rad_x += (float)((mpuData.gx - mpuData_Offset.gx) / (16.4f * freq_MPU6050)) * Degree_to_Radian;
  Angular.Rad_y += (float)((mpuData.gy - mpuData_Offset.gy) / (16.4f * freq_MPU6050)) * Degree_to_Radian;
  Angular.Rad_z += (float)((mpuData.gz - mpuData_Offset.gz) / (16.4f * freq_MPU6050)) * Degree_to_Radian;

  // Deadband tiny changes to reduce drift noise.
  if (abs(Angular.Rad_x - Part_Angular.Rad_x) < 0.00005) {
    Angular.Rad_x = Part_Angular.Rad_x;
  } else {
    Part_Angular.Rad_x = Angular.Rad_x;
  }

  if (abs(Angular.Rad_y - Part_Angular.Rad_y) < 0.00005) {
    Angular.Rad_y = Part_Angular.Rad_y;
  } else {
    Part_Angular.Rad_y = Angular.Rad_y;
  }

  if (abs(Angular.Rad_z - Part_Angular.Rad_z) < 0.00005) {
    Angular.Rad_z = Part_Angular.Rad_z;
  } else {
    Part_Angular.Rad_z = Angular.Rad_z;
  }
}

void MPU6050::Degree(float _Degree[]) {
  MPU_get_gyro();

  // Same integration as Degree(), but write into a caller-owned array.
  _Degree[0] += (float)((mpuData.gx - mpuData_Offset.gx) / (16.4f * freq_MPU6050));
  _Degree[1] += (float)((mpuData.gy - mpuData_Offset.gy) / (16.4f * freq_MPU6050));
  _Degree[2] += (float)((mpuData.gz - mpuData_Offset.gz) / (16.4f * freq_MPU6050));

  if (abs(_Degree[0] - Part_Angular.Deg_x) < 0.002) {
    _Degree[0] = Part_Angular.Deg_x;
  } else {
    Part_Angular.Deg_x = _Degree[0];
  }

  if (abs(_Degree[1] - Part_Angular.Deg_y) < 0.002) {
    _Degree[1] = Part_Angular.Deg_y;
  } else {
    Part_Angular.Deg_y = _Degree[1];
  }

  if (abs(_Degree[2] - Part_Angular.Deg_z) < 0.002) {
    _Degree[2] = Part_Angular.Deg_z;
  } else {
    Part_Angular.Deg_z = _Degree[2];
  }
}

void MPU6050::Radian(float _Radian[]) {
  MPU_get_gyro();

  // Same integration as Radian(), but write into a caller-owned array.
  _Radian[0] += (float)((mpuData.gx - mpuData_Offset.gx) / (16.4f * freq_MPU6050)) * Degree_to_Radian;
  _Radian[1] += (float)((mpuData.gy - mpuData_Offset.gy) / (16.4f * freq_MPU6050)) * Degree_to_Radian;
  _Radian[2] += (float)((mpuData.gz - mpuData_Offset.gz) / (16.4f * freq_MPU6050)) * Degree_to_Radian;

  if (abs(_Radian[0] - Part_Angular.Rad_x) < 0.00005) {
    _Radian[0] = Part_Angular.Rad_x;
  } else {
    Part_Angular.Rad_x = _Radian[0];
  }

  if (abs(_Radian[1] - Part_Angular.Rad_y) < 0.00005) {
    _Radian[1] = Part_Angular.Rad_y;
  } else {
    Part_Angular.Rad_y = _Radian[1];
  }

  if (abs(_Radian[2] - Part_Angular.Rad_z) < 0.00005) {
    _Radian[2] = Part_Angular.Rad_z;
  } else {
    Part_Angular.Rad_z = _Radian[2];
  }
}
