#pragma once

extern "C"
{

#include "driver/gpio.h"
#include "i2c.h"

}

#include "config.hpp"
#include "Vec.hpp"

extern "C"
{

class MPU6050
{
    protected:

    // a number of I2C interface used
    i2c_port_t i2c_port;

    //void get_from_fifo();

    uint8_t i2c_addres;

    bool interrupt_status(uint8_t*_int) const;

    uint8_t who_am_i() const;

    public:

    struct calibr_data_t
    {
        Vec3Di gyroscope;
        Vec3Di accelerometer;
    };

    enum GyroscopeRange
    {
        G250=0,
        G500=1,
        G1000,
        G2000
    };

    enum AccelerometerRange
    {
        G2=0,
        G4=1,
        G8,
        G16
    };

    enum FSYNC_SIGNAL_SRC
    {
        DISABLED=0,
        TEMP_OUT_L=1,
        GYRO_XOUT_L,
        GYRO_YOUT_L,
        GYRO_ZOUT_L,
        ACCEL_XOUT_L,
        ACCEL_YOUT_L,
        ACCEL_ZOUT_L
    };

    enum DPLF_MODE
    {
        _240=0,
        _184=1,
        _94,
        _44,
        _21,
        _10,
        _5
    };

    enum CLOCK_SOURCE
    {
        INTERNAL_8MHZ=0,
        PLL_GYRO_X_REF=1,
        PLL_GYRO_Y_REF,
        PLL_GYRO_Z_REF,
        PLL_EXTERNAL_32k,
        PLL_EXTERNAL_19MHZ
    };

    enum FIFO_DATA_FLAGS
    {
        TEMPERATURE=1<<7,
        GRYO_X=1<<6,
        GYRO_Y=1<<5,
        GYRO_Z=1<<4,
        ACCEL=1<<3,
        ALL=0xF8,
        ALL_REG=0xFF

    };

    protected:

    calibr_data_t calibr_data;

    GyroscopeRange gyro_range;
    AccelerometerRange accel_range;

    public:

    float from_raw_gyro(const int16_t input) const;

    float from_raw_accel(const int16_t input) const;

    int16_t gyro_to_raw(const float input) const;

    int16_t accel_to_raw(const float input) const;

    Vec3Df from_raw_gyro(const Vec3Di& input) const;

    Vec3Df from_raw_accel(const Vec3Di& input) const;

    Vec3Di gyro_to_raw(const Vec3Df& input) const;

    Vec3Di accel_to_raw(const Vec3Df& input) const;

    MPU6050(i2c_port_t i2c_port = I2C_NUM_0,uint8_t i2c_address=MPU6050_ADDRESS);

    bool init(bool int_en=false);

    bool FIFOInit();

    bool FIFODeInit();

    void DoGyroCalibration(uint16_t n=100,uint16_t steps=5);

    void DoAccelCalibration(uint16_t n=100,uint16_t steps=5,Vec3Df gravity=Vec3Df(0,0,-1.0));

    void printOffsets() const;

    const calibr_data_t& getCalibrationData() const;

    void setCalibrationData(const calibr_data_t& calibr);

    bool setGyroscopeRange(GyroscopeRange range);

    bool setAccelerometerRange(AccelerometerRange range);

    const GyroscopeRange& getGyroscopeRange() const;

    const AccelerometerRange& getAccelerometerRange() const;

    // return interrupt status
    uint8_t IntStatus() const;

    bool reset() const;

    bool Sleep() const;

    bool Wakeup() const;

    bool FIFOEnable() const;

    bool FIFODisable() const;

    bool I2CMasterEnable() const;

    bool I2CMasterDisable() const;

    bool FIFOReset() const;

    bool I2CMasterReset() const;

    bool setSlaveAddress(uint8_t slave,uint8_t address) const;

    bool setDLPFMode(DPLF_MODE mode) const;

    bool setFrameSyncSource(FSYNC_SIGNAL_SRC src) const;

    bool setClockSource(CLOCK_SOURCE src) const; 

    bool setSampleRateDivider(uint8_t divider) const;

    bool getSampleRateDivider(uint8_t *divider) const;

    bool isFIFOOverflow() const;

    uint16_t FIFOCount() const;

    bool readFIFOBuffer(uint8_t *output,uint16_t size) const;

    bool readFIFOByte(uint8_t* byte) const;

    bool setFIFOInputFlag(uint8_t flags) const;

    bool resetFIFOInputFlag(uint8_t flags) const;

    int16_t readRawGyroscopeX() const;

    int16_t readRawGyroscopeY() const;

    int16_t readRawGyroscopeZ() const;

    int16_t readRawAccelerometerX() const;

    int16_t readRawAccelerometerY() const;

    int16_t readRawAccelerometerZ() const;

    float readGyroscopeX() const;

    float readGyroscopeY() const;

    float readGyroscopeZ() const;

    float readAccelerometerX() const;

    float readAccelerometerY() const;

    float readAccelerometerZ() const;

    Vec3Di readRawGyroscope() const;

    Vec3Di readRawAccelerometer() const;

    Vec3Df readGyroscope() const;

    Vec3Df readAccelerometer() const;

    void setGyroOffsets(const Vec3Di& offsets);

    void setAccelOffsets(const Vec3Di& offsets);

    void setGyroOffsetX(uint16_t offset);

    void setGyroOffsetY(uint16_t offset);

    void setGyroOffsetZ(uint16_t offset);

    void setAccelOffsetX(uint16_t offset);

    void setAccelOffsetY(uint16_t offset);

    void setAccelOffsetZ(uint16_t offset);

    const Vec3Di& getGyroOffsets() const;

    uint16_t getGyroOffsetX() const;

    uint16_t getGyroOffsetY() const;

    uint16_t getGyroOffsetZ() const;

    const Vec3Di& getAccelOffsets() const;

    uint16_t getAccelOffsetX() const;

    uint16_t getAccelOffsetY() const;

    uint16_t getAccelOffsetZ() const;

};

}