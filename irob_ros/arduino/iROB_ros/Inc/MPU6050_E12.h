#ifndef MPU6050_E12_H_
#define MPU6050_E12_H_

#include <Arduino.h>
#include <Wire.h>

class MPU6050 {
protected:
  // MPU-6050 register addresses used by this lightweight gyro/accel driver.
  typedef enum {
    MPU_ADDR = 0x68,
    REG_PWRMGMT_1 = 0x6B,
    REG_PWRMGMT_2 = 0x6C,
    REG_SMPRT_DIV = 0x19,
    REG_DLPFCONF = 0x1A,
    REG_GYROCONF = 0x1B,
    REG_ACCLCONF = 0x1C,
    REG_ACCEL_XOUT_H = 0x3B,
    REG_GYRO_XOUT_H = 0x43
  } ADDR_Setup;

  ADDR_Setup Address;

public:
  uint16_t freq_MPU6050 = 100;  // Expected update rate in Hz

  // Degree-to-radian conversion used when integrating gyro data.
#define Degree_to_Radian 0.017453f

  MPU6050() {
    // Empty constructor for sketches that configure I2C later.
  }

  MPU6050(uint8_t I2C_SDA_Pin, uint8_t I2C_SCL_Pin, uint16_t freq) {
    Wire.begin(I2C_SDA_Pin, I2C_SCL_Pin);
    freq_MPU6050 = freq;
  }

  typedef struct {
    int16_t gx;
    int16_t gy;
    int16_t gz;

    int16_t Ax;
    int16_t Ay;
    int16_t Az;
  } Data_MPU6050;

  Data_MPU6050 mpuData;

  typedef struct {
    float gx;
    float gy;
    float gz;

    float Ax;
    float Ay;
    float Az;
  } calib_MPU6050;

  // Gyro/accelerometer offsets measured during calibration.
  calib_MPU6050 mpuData_Offset{
    .gx = 0,
    .gy = 0,
    .gz = 0,
    .Ax = 0,
    .Ay = 0,
    .Az = 0
  };

  typedef struct {
    float Deg_x;
    float Deg_y;
    float Deg_z;

    float Rad_x;
    float Rad_y;
    float Rad_z;
  } _Angular;

  _Angular Angular;

  // Last accepted angle values, used as a tiny deadband filter.
  _Angular Part_Angular{
    .Deg_x = 0,
    .Deg_y = 0,
    .Deg_z = 0,
    .Rad_x = 0,
    .Rad_y = 0,
    .Rad_z = 0
  };

  // Write one byte to an MPU-6050 register through I2C.
  void MPU_i2c_writeReg8(uint8_t reg, uint8_t data8);

  // Wake and configure the MPU-6050.
  void MPU_init();

  // Read raw gyro/accelerometer data from the sensor.
  void MPU_get_gyro();
  void MPU_get_Accelerometer();

  // Average gyro readings while robot is still to calculate offsets.
  void gyro_calib();

  // Integrate gyro readings to internal angle state.
  void Degree();
  void Radian();

  // Integrate gyro readings into user-provided arrays: [x, y, z].
  void Degree(float _Degree[]);
  void Radian(float _Radian[]);
};

#endif
