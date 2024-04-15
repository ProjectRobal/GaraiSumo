
extern "C"
{
    #include "i2c.h"
}

#include "HMC5883.hpp"


uint8_t HMC5883::readConfigA()
{
    uint8_t reg=0x00;

    i2c_read8(this->i2c_num,this->address,HMC5883_CONFIG_REGISTER_A,&reg);

    return reg;   
}

uint8_t HMC5883::readConfigB()
{
    uint8_t reg=0x00;

    i2c_read8(this->i2c_num,this->address,HMC5883_CONFIG_REGISTER_B,&reg);

    return reg; 
}

bool HMC5883::setConfigA(uint8_t reg)
{
    return i2c_write8(this->i2c_num,this->address,HMC5883_CONFIG_REGISTER_A,reg);
}

bool HMC5883::setConfigB(uint8_t reg)
{
    return i2c_write8(this->i2c_num,this->address,HMC5883_CONFIG_REGISTER_B,reg);
}

bool HMC5883::setModeRegister(uint8_t reg)
{
    return i2c_write8(this->i2c_num,this->address,HMC5883_MODE_REGISTER,reg);
}

uint8_t HMC5883::readModeRegister()
{
    uint8_t reg=0x00;

    i2c_read8(this->i2c_num,this->address,HMC5883_MODE_REGISTER,&reg);

    return reg; 
}

uint8_t HMC5883::readStatusRegister()
{
    uint8_t reg=0x00;

    i2c_read8(this->i2c_num,this->address,HMC5883_STATUS_REGISTER,&reg);

    return reg; 
}

uint8_t HMC5883::readIDA()
{
    uint8_t reg=0x00;

    i2c_read8(this->i2c_num,this->address,HMC5883_ID_REGISTER_A,&reg);

    return reg; 
}

uint8_t HMC5883::readIDB()
{
    uint8_t reg=0x00;

    i2c_read8(this->i2c_num,this->address,HMC5883_ID_REGISTER_B,&reg);

    return reg; 
}

uint8_t HMC5883::readIDC()
{
    uint8_t reg=0x00;

    i2c_read8(this->i2c_num,this->address,HMC5883_ID_REGISTER_C,&reg);

    return reg; 
}

HMC5883::HMC5883(i2c_port_t i2c_num,uint8_t address)
{
    this->i2c_num=i2c_num;
    this->address=address;

    this->LSB=1090;
}

bool HMC5883::checkDevice()
{
    return ( this->readIDA() == HMC5883_ID_A_VALUE ) 
    && ( this->readIDB() == HMC5883_ID_B_VALUE )
    && ( this->readIDC() == HMC5883_ID_A_VALUE );
}


bool HMC5883::setSampleAveraging(HMC5883::SampleAverage average)
{
    uint8_t conf=this->readConfigA();

    conf=((conf&(~(0b11<<5)))|(average<<5));

    return this->setConfigA(conf);
}

HMC5883::SampleAverage HMC5883::readSampleAveraging()
{
    uint8_t conf=this->readConfigA();

    uint8_t samp= ( conf>>5 ) & 0b11;

    switch (samp)
    {
    case 0b00:
        return SampleAverage::_1;
    break;

    case 0b01:
        return SampleAverage::_2;
    break;

    case 0b10:
        return SampleAverage::_4;
    break;

    case 0b11:
        return SampleAverage::_8;
    break;
    default:
        return SampleAverage::AVG_NOK;
    }

    
}

bool HMC5883::setDataOutputRate(HMC5883::DataOutputRate rate)
{
    uint8_t conf=this->readConfigA();

    conf=((conf&(~(0b111<<2)))|(rate<<2));

    return this->setConfigA(conf);
}

HMC5883::DataOutputRate HMC5883::readDataOutputRate()
{
    uint8_t conf=this->readConfigA();

    uint8_t samp= ( conf>>2 ) & 0b111;

    switch (samp)
    {
    case 0b000:
        return DataOutputRate::_075;
    break;

    case 0b001:
        return DataOutputRate::_150;
    break;

    case 0b010:
        return DataOutputRate::_300;
    break;

    case 0b011:
        return DataOutputRate::_750;
    break;

    case 0b100:
        return DataOutputRate::_1500;
    break;

    case 0b101:
        return DataOutputRate::_3000;
    break;

    case 0b110:
        return DataOutputRate::_7500;
    break;
    default:
        return DataOutputRate::RATE_NOK;
    }

    
}

bool HMC5883::setMeasurmentMode(HMC5883::MeasurmentMode mode)
{
    uint8_t conf=this->readConfigA();

    conf=((conf&(~(0b11<<0)))|mode);

    return this->setConfigA(conf);
}

HMC5883::MeasurmentMode HMC5883::getMeasurmentMode()
{
    uint8_t conf=this->readConfigA();

    uint8_t samp= conf & 0b11;

    switch (samp)
    {
    case 0b00:
        return HMC5883::MeasurmentMode::Normal;
    break;

    case 0b01:
        return HMC5883::MeasurmentMode::PositiveBias;
    break;

    case 0b10:
        return HMC5883::MeasurmentMode::NegativeBias;
    break;
    default:
        return HMC5883::MeasurmentMode::MEAS_NOK;
    }

    
}

bool HMC5883::setGain(HMC5883::Gain gain)
{
    uint8_t conf=this->readConfigB();

    conf=((conf&(~(0b111<<5)))|(gain<<5));

    bool ret=this->setConfigB(conf);

    if(ret)
    {
        switch(gain)
        {
            case HMC5883::Gain::LSB_1370:
                this->LSB=1370;
            break;
            case HMC5883::Gain::LSB_1090:
                this->LSB=1090;
            break;
            case HMC5883::Gain::LSB_820:
                this->LSB=820;
            break;
            case HMC5883::Gain::LSB_660:
                this->LSB=660;
            break;
            case HMC5883::Gain::LSB_440:
                this->LSB=440;
            break;
            case HMC5883::Gain::LSB_390:
                this->LSB=390;
            break;
            case HMC5883::Gain::LSB_330:
                this->LSB=330;
            break;
            case HMC5883::Gain::LSB_230:
                this->LSB=230;
            break;
            default:
                this->LSB=1090;
        }
    }

    return ret;
}

HMC5883::Gain HMC5883::readGain()
{
    uint8_t conf=this->readConfigB();

    uint8_t samp= ( conf>>5 ) & 0b111;

    switch (samp)
    {
    case 0b000:
        return Gain::LSB_1370;
    break;

    case 0b001:
        return Gain::LSB_1090;
    break;

    case 0b010:
        return Gain::LSB_820;
    break;

    case 0b011:
        return Gain::LSB_660;
    break;

    case 0b100:
        return Gain::LSB_440;
    break;

    case 0b101:
        return Gain::LSB_390;
    break;

    case 0b110:
        return Gain::LSB_330;
    break;

    case 0b111:
        return Gain::LSB_230;
    break;
    default:
        return Gain::GAIN_NOK;
    }

}

bool HMC5883::setMode(HMC5883::Mode mode)
{
    uint8_t conf=this->readModeRegister();

    conf=((conf&(~(0b11<<0)))|mode);

    return this->setModeRegister(conf);
}

HMC5883::Mode HMC5883::getMode()
{
    uint8_t conf=this->readModeRegister();

    uint8_t mode = conf & 0b11;

    switch(mode)
    {
        case 0b00:
            return Mode::Continouse;
        break;
        case 0b01:
            return Mode::Single;
        break;
        case 0b10:
            return Mode::Idle;
        break;
        default:
            return Mode::MODE_NOK;
    }
}

bool HMC5883::checkDataReady()
{
    uint8_t status= this->readStatusRegister();

    return status & (1<<0);
}

int16_t HMC5883::readXRaw()
{
    uint8_t top=0x00;
    uint8_t bottom=0x00;

    i2c_read8(this->i2c_num,this->address,HMC5883_DATA_OUTPUT_X_TOP,&top);
    i2c_read8(this->i2c_num,this->address,HMC5883_DATA_OUTPUT_X_BOTTOM,&bottom);

    return top<<8 | bottom;

}

int16_t HMC5883::readYRaw()
{
    uint8_t top=0x00;
    uint8_t bottom=0x00;

    i2c_read8(this->i2c_num,this->address,HMC5883_DATA_OUTPUT_Y_TOP,&top);
    i2c_read8(this->i2c_num,this->address,HMC5883_DATA_OUTPUT_Y_BOTTOM,&bottom);

    return top<<8 | bottom;
}

int16_t HMC5883::readZRaw()
{
    uint8_t top=0x00;
    uint8_t bottom=0x00;

    i2c_read8(this->i2c_num,this->address,HMC5883_DATA_OUTPUT_Z_TOP,&top);
    i2c_read8(this->i2c_num,this->address,HMC5883_DATA_OUTPUT_Z_BOTTOM,&bottom);

    return top<<8 | bottom;
}

float HMC5883::readX()
{
    return this->readXRaw()/static_cast<float>(this->LSB);
}

float HMC5883::readY()
{
    return this->readYRaw()/static_cast<float>(this->LSB);
}

float HMC5883::readZ()
{
    return this->readZRaw()/static_cast<float>(this->LSB);
}