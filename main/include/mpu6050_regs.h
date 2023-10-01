#pragma once

// sample rate divider
#define MPU6050_SMPRT_DIV 0x19

// configuration (FSYNC,DLPF filter)
#define MPU6050_CONFIG 0x1A

// gyroscope configuration
#define MPU6050_GYRO_CONFIG 0x1B

// accelerometer configuration
#define MPU6050_ACCEL_CONFIG 0x1C

// fifo en
#define MPU6050_FIFO_EN 0x23

// INT Pin / Bypass Enable Configuration
#define MPU6050_INT_PIN_CFG 0x37

// Interrupt Enable
#define MPU6050_INT_ENABLE 0x38

// Interrupt Status
#define MPU6050_INT_STATUS 0x3A

// Accelerometer outputs:

// accelerometer x output high
#define MPU6050_ACCEL_XOUT_H 0x3B

// accelerometer x output low
#define MPU6050_ACCEL_XOUT_L 0x3C

// accelerometer y output high
#define MPU6050_ACCEL_YOUT_H 0x3D

// accelerometer y output low
#define MPU6050_ACCEL_YOUT_L 0x3E

// accelerometer z output high
#define MPU6050_ACCEL_ZOUT_H 0x3F

// accelerometer z output low
#define MPU6050_ACCEL_ZOUT_L 0x40

// temperature output high
#define MPU6050_TEMP_OUT_H 0x41

// temperature output low
#define MPU6050_TEMP_OUT_L 0x42


// Gyroscope outputs:

// gyroscope x output high
#define MPU6050_GYRO_XOUT_H 0x43

// gyroscope x output low
#define MPU6050_GYRO_XOUT_L 0x44

// gyroscope y output high
#define MPU6050_GYRO_YOUT_H 0x45

// gyroscope y output low
#define MPU6050_GYRO_YOUT_L 0x46

// gyroscope z output high
#define MPU6050_GYRO_ZOUT_H 0x47

// gyroscope z output low
#define MPU6050_GYRO_ZOUT_L 0x48

// user control 
#define MPU6050_USER_CTRL 0x6A

// power managment 1
#define MPU6050_PWR_MGMT_1 0x6B

// power managment 2
#define MPU6050_PWR_MGMT_2 0x6C

// FIFO count registers:
#define MPU6050_FIFO_COUNT_H 0x72
#define MPU6050_FIFO_COUNT_L 0x73

// FIFO Read Write
#define MPU6050_FIFO_R_W 0x74

// Who am I register
#define MPU6050_WHO_AM_I 0x75

// slaves stuff
#define MPU6050_RA_I2C_SLV0_ADDR    0x25
#define MPU6050_RA_I2C_SLV0_REG     0x26
#define MPU6050_RA_I2C_SLV0_CTRL    0x27
#define MPU6050_RA_I2C_SLV1_ADDR    0x28
#define MPU6050_RA_I2C_SLV1_REG     0x29
#define MPU6050_RA_I2C_SLV1_CTRL    0x2A
#define MPU6050_RA_I2C_SLV2_ADDR    0x2B
#define MPU6050_RA_I2C_SLV2_REG     0x2C
#define MPU6050_RA_I2C_SLV2_CTRL    0x2D
#define MPU6050_RA_I2C_SLV3_ADDR    0x2E
#define MPU6050_RA_I2C_SLV3_REG     0x2F
#define MPU6050_RA_I2C_SLV3_CTRL    0x30
#define MPU6050_RA_I2C_SLV4_ADDR    0x31
#define MPU6050_RA_I2C_SLV4_REG     0x32
#define MPU6050_RA_I2C_SLV4_DO      0x33
#define MPU6050_RA_I2C_SLV4_CTRL    0x34
#define MPU6050_RA_I2C_SLV4_DI      0x35
#define MPU6050_RA_I2C_MST_STATUS   0x36

// gryoscope offsets
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18

// accelerometer offsets
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B