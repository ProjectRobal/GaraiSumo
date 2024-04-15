#pragma once

extern "C"
{
    #include "i2c.h"
}

#include "Magnetrometer.hpp"


#define HMC5883_DEFAULT_ADDRESS 0x1E

#define HMC5883_CONFIG_REGISTER_A 0x00
#define HMC5883_CONFIG_REGISTER_B 0x01
#define HMC5883_MODE_REGISTER 0x02
#define HMC5883_DATA_OUTPUT_X_TOP 0x03
#define HMC5883_DATA_OUTPUT_X_BOTTOM 0x04
#define HMC5883_DATA_OUTPUT_Z_TOP 0x05
#define HMC5883_DATA_OUTPUT_Z_BOTTOM 0x06
#define HMC5883_DATA_OUTPUT_Y_TOP 0x07
#define HMC5883_DATA_OUTPUT_Y_BOTTOM 0x08

#define HMC5883_STATUS_REGISTER 0x09
#define HMC5883_ID_REGISTER_A 0x0A
#define HMC5883_ID_REGISTER_B 0x0B
#define HMC5883_ID_REGISTER_C 0x0C


#define HMC5883_ID_A_VALUE 0b01001000
#define HMC5883_ID_B_VALUE 0b00110100
#define HMC5883_ID_C_VALUE 0b00110011


class HMC5883 : public Magnetrometer
{   

    uint8_t address;

    i2c_port_t i2c_num;


    uint8_t readConfigA();

    uint8_t readConfigB();

    bool setConfigA(uint8_t reg);

    bool setConfigB(uint8_t reg);

    bool setModeRegister(uint8_t reg);

    uint8_t readModeRegister();

    uint8_t readStatusRegister();

    uint8_t readIDA();

    uint8_t readIDB();

    uint8_t readIDC();

    uint16_t LSB;
    
    public:

    enum SampleAverage
    {
        _1=0b00,
        _2=0b01,
        _4=0b10,
        _8=0b11,
        AVG_NOK,
    };

    enum DataOutputRate
    {
        // 0.75 Hz
        _075=0b000,
        // 1.5 Hz
        _150=0b001,
        // 3 Hz
        _300=0b010,
        // 7.5 Hz
        _750=0b011,
        // 15 Hz
        _1500=0b100,
        // 30 Hz
        _3000=0b101,
        // 75 Hz
        _7500=0b110,
        RATE_NOK
    };

    enum MeasurmentMode
    {
        Normal=0b00,
        PositiveBias=0b01,
        NegativeBias=0b10,
        MEAS_NOK
    };

    enum Gain
    {
        LSB_1370=0b000,
        LSB_1090=0b001,
        LSB_820=0b010,
        LSB_660=0b011,
        LSB_440=0b100,
        LSB_390=0b101,
        LSB_330=0b110,
        LSB_230=0b111,
        GAIN_NOK
    };

    enum Mode
    {
        Continouse=0b00,
        Single=0b01,
        Idle=0b10,
        MODE_NOK
    };

    HMC5883(i2c_port_t i2c_num,uint8_t address=HMC5883_DEFAULT_ADDRESS);

    bool checkDevice();

    bool setSampleAveraging(HMC5883::SampleAverage average);

    HMC5883::SampleAverage readSampleAveraging();

    bool setDataOutputRate(HMC5883::DataOutputRate rate);

    HMC5883::DataOutputRate readDataOutputRate();

    bool setMeasurmentMode(HMC5883::MeasurmentMode mode);

    HMC5883::MeasurmentMode getMeasurmentMode();

    bool setGain(HMC5883::Gain gain);

    HMC5883::Gain readGain();

    bool setMode(HMC5883::Mode mode);

    HMC5883::Mode getMode();

    bool checkDataReady();

    int16_t readXRaw();

    int16_t readYRaw();

    int16_t readZRaw();

    float readX();

    float readY();

    float readZ();
    
    };


