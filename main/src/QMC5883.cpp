
extern "C"
{
    #include "i2c.h"
}

#include "QMC5883.hpp"


uint8_t QMC5883::readConfig1()
{
    uint8_t reg;

    i2c_read8(this->i2c_num,this->address,QMC5883_CONFIG_1_REGISTER,&reg);

    return reg;
}

bool QMC5883::setConfig1(uint8_t reg)
{
    return i2c_write8(this->i2c_num,this->address,QMC5883_CONFIG_1_REGISTER,reg);
}


bool QMC5883::setConfig2(uint8_t reg)
{
    return i2c_write8(this->i2c_num,this->address,QMC5883_CONFIG_2_REGISTER,reg);
}

uint8_t QMC5883::readConfig2()
{
    uint8_t reg;

    i2c_read8(this->i2c_num,this->address,QMC5883_CONFIG_2_REGISTER,&reg);

    return reg;
}

bool QMC5883::setResetReg(uint8_t reg)
{
    return i2c_write8(this->i2c_num,this->address,QMC5883_RESET_REGISTER,reg);
}

uint8_t QMC5883::getResetReg()
{
    uint8_t reg;

    i2c_read8(this->i2c_num,this->address,QMC5883_RESET_REGISTER,&reg);

    return reg;
}

uint8_t QMC5883::readChipID()
{
    uint8_t reg;

    i2c_read8(this->i2c_num,this->address,QMC5883_CHIP_REG_ID,&reg);

    return reg;
}

uint8_t QMC5883::readStatus()
{
    uint8_t reg;

    i2c_read8(this->i2c_num,this->address,QMC5883_STATUS_REGISTER,&reg);

    return reg;
}

QMC5883::QMC5883(i2c_port_t i2c_num,uint8_t address)
{
    this->i2c_num=i2c_num;
    this->address=address;

    this->LSB=12000;
}

bool QMC5883::init()
{
    return this->setResetReg(0x01);
}

bool QMC5883::checkChipID()
{
    return this->readChipID() == QMC5883_CHIP_ID;
}

bool QMC5883::setMode(QMC5883::Mode mode)
{
    uint8_t reg=this->readConfig1();

    reg=(((reg&(~0b11))|mode));

    return this->setConfig1(reg);
}

QMC5883::Mode QMC5883::getMode()
{
    uint8_t reg=this->readConfig1();

    reg= reg & 0b11;

    switch (reg)
    {
        case 0b00:
            return QMC5883::Mode::Standby;
        break;
        case 0b01:
            return QMC5883::Mode::Continuous;
        break;
        
    }

    return QMC5883::Mode::MODE_NOK;
}

bool QMC5883::setOutputDataRate(QMC5883::OutputDataRate rate)
{
    uint8_t reg=this->readConfig1();

    reg=((reg&(~(0b11<<2)|(rate<<2))));

    return this->setConfig1(reg);
}

QMC5883::OutputDataRate QMC5883::getOutputDataRate()
{
    uint8_t reg=this->readConfig1();

    reg= ( reg>>2 ) & 0b11;

    switch (reg)
    {
        case 0b00:
            return QMC5883::OutputDataRate::_10Hz;
        break;
        case 0b01:
            return QMC5883::OutputDataRate::_50Hz;
        break;
        case 0b10:
            return QMC5883::OutputDataRate::_100Hz;
        break;
        case 0b11:
            return QMC5883::OutputDataRate::_200Hz;
        break;
        
    }

    return QMC5883::OutputDataRate::DATA_RATE_NOK;
}

bool QMC5883::setFullScale(QMC5883::FullScale scale)
{
    uint8_t reg=this->readConfig1();

    reg=((reg&((~(0b11<<4))|(scale<<4))));

    bool ret=this->setConfig1(reg);

    if(ret)
    {
        switch (scale)
        {
        case QMC5883::FullScale::G2:
            this->LSB=12000;
        break;
        
        case QMC5883::FullScale::G8:
            this->LSB=3000;
        break;
        default:
            this->LSB=3000;
        }
    }

    return ret;
}

QMC5883::FullScale QMC5883::getFullScale()
{
    uint8_t reg=this->readConfig1();

    reg= ( reg>>4 ) & 0b11;

    switch (reg)
    {
        case 0b00:
            return QMC5883::FullScale::G2;
        break;
        case 0b01:
            return QMC5883::FullScale::G8;
        break;
    }

    return QMC5883::FullScale::SCALE_NOK;
}

bool QMC5883::setOverSampleRatio(QMC5883::OverSampleRatio ratio)
{
    uint8_t reg=this->readConfig1();

    reg=((reg&((~(0b11<<6))|(ratio<<6))));

    return this->setConfig1(reg);
}

QMC5883::OverSampleRatio QMC5883::getOverSampleRatio()
{
    uint8_t reg=this->readConfig1();

    reg= ( reg>>6 ) & 0b11;

    switch (reg)
    {
        case 0b00:
            return QMC5883::OverSampleRatio::_512;
        break;
        case 0b01:
            return QMC5883::OverSampleRatio::_256;
        break;
        case 0b10:
            return QMC5883::OverSampleRatio::_128;
        break;
        case 0b11:
            return QMC5883::OverSampleRatio::_64;
        break;
        
    }

    return QMC5883::OverSampleRatio::RATIO_NOK;
}

bool QMC5883::softReset()
{
    uint8_t reg=this->readConfig2();

    reg|=(1<<7);

    return this->setConfig2(reg);
}

bool QMC5883::setRollOver(bool en)
{
    uint8_t reg=this->readConfig2();

    if(en)
    {
        reg|=(1<<6);
    }
    else
    {
        reg&=~(1<<6);
    }

    return this->setConfig2(reg);
}

bool QMC5883::setIntEnable(bool en)
{
    uint8_t reg=this->readConfig2();

    if(en)
    {
        reg|=1;
    }
    else
    {
        reg&=~1;
    }

    return this->setConfig2(reg);
}

bool QMC5883::checkDataReady()
{
    uint8_t reg=this->readStatus();

    return reg | 1<<0;
}

int16_t QMC5883::readXRaw()
{
    uint8_t top=0x00;
    uint8_t bottom=0x00;

    i2c_read8(this->i2c_num,this->address,QMC5883_X_BOTTOM,&bottom);
    i2c_read8(this->i2c_num,this->address,QMC5883_X_TOP,&top);

    return top << 8 | bottom;

}

int16_t QMC5883::readYRaw()
{
    uint8_t top=0x00;
    uint8_t bottom=0x00;

    i2c_read8(this->i2c_num,this->address,QMC5883_Y_BOTTOM,&bottom);
    i2c_read8(this->i2c_num,this->address,QMC5883_Y_TOP,&top);

    return top << 8 | bottom;
}

int16_t QMC5883::readZRaw()
{
    uint8_t top=0x00;
    uint8_t bottom=0x00;

    i2c_read8(this->i2c_num,this->address,QMC5883_Z_BOTTOM,&bottom);
    i2c_read8(this->i2c_num,this->address,QMC5883_Z_TOP,&top);

    return top << 8 | bottom;
}

float QMC5883::readX()
{
    return this->readX()/static_cast<float>(this->LSB);
}

float QMC5883::readY()
{
    return this->readY()/static_cast<float>(this->LSB);
}

float QMC5883::readZ()
{
    return this->readZ()/static_cast<float>(this->LSB);
}