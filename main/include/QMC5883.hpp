#pragma once

/*
    To do:

    Read sensors reading,
    Status register flag check
    SET/RESET Period Register value setting
    Temperature reading?


*/

extern "C"
{
    #include "i2c.h"
}

#include "Magnetrometer.hpp"


#define QMC5883_DEFUALT_ADDRESS 0x0D

#define QMC5883_X_BOTTOM 0x00
#define QMC5883_X_TOP 0x01
#define QMC5883_Y_BOTTOM 0x02
#define QMC5883_Y_TOP 0x03
#define QMC5883_Z_BOTTOM 0x04
#define QMC5883_Z_TOP 0x05

#define QMC5883_STATUS_REGISTER 0x06

#define QMC5883_TEMPERATURE_BOTTOM 0x07
#define QMC5883_TEMPERATURE_TOP 0x08

#define QMC5883_CONFIG_1_REGISTER 0x09
#define QMC5883_CONFIG_2_REGISTER 0x0A

#define QMC5883_RESET_REGISTER 0x0C
#define QMC5883_CHIP_REG_ID 0x0D

#define QMC5883_CHIP_ID 0xFF


class QMC5883 : public Magnetrometer
{

    i2c_port_t i2c_num;

    uint8_t address;

    uint32_t LSB;

    uint8_t readStatus();

    bool setConfig1(uint8_t reg);

    uint8_t readConfig1();

    bool setConfig2(uint8_t reg);

    uint8_t readConfig2();

    bool setResetReg(uint8_t reg);

    uint8_t getResetReg();

    uint8_t readChipID();

    public:

    enum Mode
    {
        Standby=0x00,
        Continuous=0b01,
        MODE_NOK
    };

    enum OutputDataRate
    {
        _10Hz=0x00,
        _50Hz=0b01,
        _100Hz=0b10,
        _200Hz=0b11,
        DATA_RATE_NOK
    };

    enum FullScale
    {
        G2=0b00,
        G8=0b01,
        SCALE_NOK
    };

    enum OverSampleRatio
    {
        _512=0b00,
        _256=0x01,
        _128=0b10,
        _64=0b11,
        RATIO_NOK
    };

    QMC5883(i2c_port_t i2c_num,uint8_t address=QMC5883_DEFUALT_ADDRESS);

    bool init();

    bool checkChipID();

    bool setMode(QMC5883::Mode mode);

    QMC5883::Mode getMode();

    bool setOutputDataRate(QMC5883::OutputDataRate rate);

    QMC5883::OutputDataRate getOutputDataRate();

    bool setFullScale(QMC5883::FullScale scale);

    QMC5883::FullScale getFullScale();

    bool setOverSampleRatio(QMC5883::OverSampleRatio ratio);

    QMC5883::OverSampleRatio getOverSampleRatio();

    bool softReset();

    bool setRollOver(bool en);

    bool setIntEnable(bool en);

    bool checkDataReady();

    int16_t readXRaw();

    int16_t readYRaw();

    int16_t readZRaw();

    float readX();

    float readY();

    float readZ();
};