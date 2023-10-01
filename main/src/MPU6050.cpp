#include "MPU6050.hpp"
#include "mpu6050_regs.h"
#include "esp_log.h"

MPU6050::MPU6050(i2c_port_t i2c_port,uint8_t i2c_address)
{
    this->i2c_port=i2c_port;
    this->i2c_addres=i2c_address;
    this->calibr_data.accelerometer=Vec3Di(0);
    this->calibr_data.gyroscope=Vec3Di(0);

}

uint8_t MPU6050::who_am_i() const
{
    uint8_t reg=0x00;

    i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_WHO_AM_I,&reg);

    return reg;

}

bool MPU6050::interrupt_status(uint8_t* _int) const
{
    return i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_INT_STATUS,_int);
}

void MPU6050::DoGyroCalibration(uint16_t n,uint16_t steps)
{
    // check if intterupt has been configured

    uint8_t reg=0x00;

    bool use_int=false;

    i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_INT_ENABLE,&reg);

    use_int=reg & (1<<0);

    this->calibr_data.gyroscope=Vec3Di(0);

    while(steps--)
    {

    Vec3Di gyro=Vec3Di(0);

    for(uint16_t i=0;i<n;++i)
    {
        
        
        gyro+=readRawGyroscope();

        if(use_int)
        {
            while(!(this->IntStatus() & (1<<0))){};
        }
        else
        {
            vTaskDelay(10/portTICK_PERIOD_MS);
        }

    }

    this->calibr_data.gyroscope+=gyro/=n;

    }

    //this->setGyroOffsets(this->calibr_data.gyroscope);
    //this->setAccelOffsets(this->calibr_data.accelerometer);
    
}

void MPU6050::DoAccelCalibration(uint16_t n,uint16_t steps,Vec3Df gravity)
{
    this->calibr_data.accelerometer=Vec3Di(0);

    Vec3Di ref=Vec3Di(this->accel_to_raw(gravity.x),this->accel_to_raw(gravity.y),this->accel_to_raw(gravity.z));

    uint8_t reg=0x00;

    bool use_int=false;

    i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_INT_ENABLE,&reg);

    use_int=reg & (1<<0);

    while(steps--)
    {

    Vec3Di accel=Vec3Di(0);

    for(uint16_t i=0;i<n;++i)
    {
        
        accel+=(readRawAccelerometer()-ref);

        if(use_int)
        {
            while(!(this->IntStatus() & (1<<0))){};
        }
        else
        {
            vTaskDelay(10/portTICK_PERIOD_MS);
        }

    }

    this->calibr_data.accelerometer+=accel/=n;

    }

}

void MPU6050::printOffsets() const
{
    ESP_LOGI("Sensors","Gyroscope offsets: X: %ld Y: %ld Z: %ld ",this->calibr_data.gyroscope.x,this->calibr_data.gyroscope.y,this->calibr_data.gyroscope.z);
    ESP_LOGI("Sensors","Accelerometer offsets: X: %ld Y: %ld Z: %ld ",this->calibr_data.accelerometer.x,this->calibr_data.accelerometer.y,this->calibr_data.accelerometer.z);
}

const MPU6050::calibr_data_t& MPU6050::getCalibrationData() const
{
    return this->calibr_data;
}

void MPU6050::setCalibrationData(const calibr_data_t& calibr)
{
    this->calibr_data=calibr;
}

bool MPU6050::FIFOInit()
{
    // int pin reacts on FIFO overflow
        uint8_t reg=0x00;

        if(!i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_INT_ENABLE,&reg))
        {
            return false;
        }

        reg|=(1<<4);

        if(!i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_INT_ENABLE,reg))
        {
            return false;
        }


        if(!this->FIFOEnable())
        {
            return false;
        }

    return true;
}

bool MPU6050::FIFODeInit()
{
// int pin reacts on FIFO overflow
        uint8_t reg=0x00;

        if(!i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_INT_ENABLE,&reg))
        {
            return false;
        }

        reg&=~(1<<4);

        if(!i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_INT_ENABLE,reg))
        {
            return false;
        }


        if(!this->resetFIFOInputFlag(FIFO_DATA_FLAGS::ALL_REG))
        {
            return false;
        }

        if(!this->FIFODisable())
        {
            return false;
        }

    return true;
}

bool MPU6050::init(bool int_en)
{
    // add basic configuration disable FSYNC, set filters etc.

    // check who am i register
    if(this->who_am_i()!=0x68)
    {
        return false;
    }

    // reset device
    if(!this->reset())
    {
        return false;
    }

    vTaskDelay(50/portTICK_PERIOD_MS);

    if(!this->setGyroscopeRange(GyroscopeRange::G250))
    {
        return false;
    }
    if(!this->setAccelerometerRange(AccelerometerRange::G2))
    {
        return false;
    }

    //disable FSYNC and low pass filter
    if(!this->setFrameSyncSource(FSYNC_SIGNAL_SRC::DISABLED))
    {
        return false;
    }

    if(!this->setDLPFMode(DPLF_MODE::_240))
    {
        return false;
    }

    // enable FIFO buffer when int gpio is set
    if(int_en)
    {

        // int pin configuration

        if(!i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_INT_PIN_CFG,0x70))
        {
            return false;
        }

        uint8_t reg=0x00;

        if(!i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_INT_ENABLE,&reg))
        {
            return false;
        }

        reg|=(1<<0);

        if(!i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_INT_ENABLE,reg))
        {
            return false;
        }

    
    }
   

    // power managment

    if(!i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_PWR_MGMT_1,0x00))
    {
        return false;
    }

    if(!i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_PWR_MGMT_2,0x00))
    {
        return false;
    }

    return true;
}

bool MPU6050::setSampleRateDivider(uint8_t divider) const
{
    return i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_SMPRT_DIV,divider);
}

bool MPU6050::getSampleRateDivider(uint8_t* divider) const
{
    return i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_SMPRT_DIV,divider);
}

bool MPU6050::isFIFOOverflow() const
{
    uint8_t reg=0x00;

    bool res=this->interrupt_status(&reg);

    // return false on read failure
    return res & (reg & (1<<4));
}

uint16_t MPU6050::FIFOCount() const
{

    uint8_t low=0x00;
    uint8_t high=0x00;

    i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_FIFO_COUNT_L,&low);
    i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_FIFO_COUNT_H,&high);

    return static_cast<uint16_t>((high<<8)|low);
}

bool MPU6050::readFIFOByte(uint8_t* byte) const
{
    return i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_FIFO_R_W,byte);
}

bool MPU6050::readFIFOBuffer(uint8_t *output,uint16_t size) const
{

   return i2c_readBytes(this->i2c_port,this->i2c_addres,MPU6050_FIFO_R_W,output,size);
}


bool MPU6050::setFIFOInputFlag(uint8_t flags) const
{
    uint8_t reg=0x00;

    bool res=true;

    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_FIFO_EN,&reg);

    reg|=flags;

    res&=i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_FIFO_EN,reg);

    return res;

}

bool MPU6050::resetFIFOInputFlag(uint8_t flags) const
{
    uint8_t reg=0x00;

    bool res=true;

    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_FIFO_EN,&reg);

    reg&=~flags;

    res&=i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_FIFO_EN,reg);

    return res;
}

bool MPU6050::setGyroscopeRange(GyroscopeRange range)
{
    uint8_t reg=0x00;

    bool res=true;

    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_GYRO_CONFIG,&reg);

    // clear FS_SEL bits 
    reg=reg&~(0x18);

    reg|=(this->gyro_range<<3 & 0x18);

    res&=i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_GYRO_CONFIG,reg);

    if(res)
    {
        this->gyro_range=range;
    }

    return res;
}

bool MPU6050::setAccelerometerRange(AccelerometerRange range)
{
    uint8_t reg=0x00;

    bool res=true;

    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_ACCEL_CONFIG,&reg);

    // clear FS_SEL bits 
    reg=reg&~(0x18);

    reg|=(this->accel_range<<3 & 0x18);

    res&=i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_ACCEL_CONFIG,reg);

    if(res)
    {
        this->accel_range=range;
    }

    return res;

}

const MPU6050::GyroscopeRange& MPU6050::getGyroscopeRange() const
{
    return this->gyro_range;
}

const MPU6050::AccelerometerRange& MPU6050::getAccelerometerRange() const
{
    return this->accel_range;
}

uint8_t MPU6050::IntStatus() const
{
    uint8_t _int=0x00;

    this->interrupt_status(&_int);

    return _int;
}

bool MPU6050::reset() const
{
    uint8_t reg=0x00;

    bool res=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_PWR_MGMT_1,&reg);

    reg|=(1<<7);

    res&=i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_PWR_MGMT_1,reg);

    return res;
}

float MPU6050::from_raw_gyro(const int16_t& input) const
{
    float ssf=(131.0/(2<<this->gyro_range));

    return static_cast<float>(input)/ssf;    
}

float MPU6050::from_raw_accel(const int16_t& input) const
{
    float ssf=(16384.0/(2<<this->accel_range));

    return static_cast<float>(input)/ssf; 
}

int16_t MPU6050::accel_to_raw(const float& input) const
{
    float ssf=((2<<this->accel_range)/16384.0);

    return static_cast<int16_t>(input/ssf);
}

int16_t MPU6050::gyro_to_raw(const float& input) const
{
    float ssf=((2<<this->gyro_range)/131.0);

    return static_cast<int16_t>(input/ssf);
}


bool MPU6050::Sleep() const
{
    bool res=true;
    
    uint8_t output=0x00;

    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_PWR_MGMT_1,&output);

    output|=1<<6;

    res&=i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_PWR_MGMT_1,output);

    return res;
}

bool MPU6050::Wakeup() const
{
    bool res=true;
    
    uint8_t output=0x00;

    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_PWR_MGMT_1,&output);

    output&=~(1<<6);

    res&=i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_PWR_MGMT_1,output);

    return res;
}


bool MPU6050::FIFOEnable() const
{
    bool res=true;
    
    uint8_t output=0x00;

    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_USER_CTRL,&output);

    output|=1<<6;

    res&=i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_USER_CTRL,output);

    return res;

}

bool MPU6050::FIFODisable() const
{
    bool res=true;
    
    uint8_t output=0x00;

    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_USER_CTRL,&output);

    output&=~(1<<6);

    res&=i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_USER_CTRL,output);

    return res;
}

bool MPU6050::I2CMasterEnable() const
{
    bool res=true;
    
    uint8_t output=0x00;

    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_USER_CTRL,&output);

    output|=1<<5;

    res&=i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_USER_CTRL,output);

    return res;
}

bool MPU6050::I2CMasterDisable() const
{
    bool res=true;
    
    uint8_t output=0x00;

    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_USER_CTRL,&output);

    output&=~(1<<5);

    res&=i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_USER_CTRL,output);

    return res;
}

bool MPU6050::FIFOReset() const
{
    bool res=true;
    
    uint8_t output=0x00;

    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_USER_CTRL,&output);

    output|=1<<2;

    res&=i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_USER_CTRL,output);

    return res;
}

bool MPU6050::I2CMasterReset() const
{
    bool res=true;
    
    uint8_t output=0x00;

    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_USER_CTRL,&output);

    output|=1<<1;

    res&=i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_USER_CTRL,output);

    return res;
}

bool MPU6050::setSlaveAddress(uint8_t slave,uint8_t address) const
{
    if(slave>3)
    {
        return false;
    }

    return i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_RA_I2C_SLV0_ADDR+slave*3,address);
}

bool MPU6050::setDLPFMode(DPLF_MODE mode) const
{
    uint8_t config=0x00;

    bool ret=true;

    ret&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_CONFIG,&config);

    // clear bits from 0-2
    config&=(~0x07);

    config|=(mode & 0x07);

    ret&=i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_CONFIG,config);

    return ret;
}

bool MPU6050::setFrameSyncSource(FSYNC_SIGNAL_SRC src) const
{
    uint8_t config=0x00;

    bool ret=true;

    ret&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_CONFIG,&config);

    // clear bits from 3-5
    config&=(~0x38);

    config|=(src<<3 & 0x38);

    ret&=i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_CONFIG,config);

    return ret;
}

bool MPU6050::setClockSource(CLOCK_SOURCE src) const
{
    uint8_t config=0x00;

    bool ret=true;

    ret&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_PWR_MGMT_1,&config);

    // clear bits from 0-2
    config&=(~0x07);

    config|=(src & 0x07);

    ret&=i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_PWR_MGMT_1,config);

    return src;
}


int16_t MPU6050::readRawGyroscopeX() const
{
    uint8_t low=0x00;
    uint8_t high=0x00;

    bool res=true;

    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_GYRO_XOUT_L,&low);
    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_GYRO_XOUT_H,&high);

    if(!res)
    {
        return 0;
    }

    return static_cast<int16_t>((high<<8 | low)- this->calibr_data.gyroscope.x);

}

int16_t MPU6050::readRawGyroscopeY() const
{
    uint8_t low=0x00;
    uint8_t high=0x00;

    bool res=true;

    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_GYRO_YOUT_L,&low);
    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_GYRO_YOUT_H,&high);

    if(!res)
    {
        return 0;
    }

    return static_cast<int16_t>((high<<8 | low) - this->calibr_data.gyroscope.y);
}

int16_t MPU6050::readRawGyroscopeZ() const
{
    uint8_t low=0x00;
    uint8_t high=0x00;

    bool res=true;

    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_GYRO_ZOUT_L,&low);
    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_GYRO_ZOUT_H,&high);

    if(!res)
    {
        return 0;
    }

    return static_cast<int16_t>((high<<8 | low) - this->calibr_data.gyroscope.z);
}

int16_t MPU6050::readRawAccelerometerX() const
{
    uint8_t low=0x00;
    uint8_t high=0x00;

    bool res=true;

    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_ACCEL_XOUT_L,&low);
    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_ACCEL_XOUT_H,&high);

    if(!res)
    {
        return 0;
    }

    return static_cast<int16_t>((high<<8 | low) - this->calibr_data.accelerometer.x);
}

int16_t MPU6050::readRawAccelerometerY() const
{
    uint8_t low=0x00;
    uint8_t high=0x00;

    bool res=true;

    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_ACCEL_YOUT_L,&low);
    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_ACCEL_YOUT_H,&high);

    if(!res)
    {
        return 0;
    }

    return static_cast<int16_t>((high<<8 | low)- this->calibr_data.accelerometer.y);
}

int16_t MPU6050::readRawAccelerometerZ() const
{
    uint8_t low=0x00;
    uint8_t high=0x00;

    bool res=true;

    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_ACCEL_ZOUT_L,&low);
    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_ACCEL_ZOUT_H,&high);

    if(!res)
    {
        return 0;
    }

    return static_cast<int16_t>((high<<8 | low)- this->calibr_data.accelerometer.z);

}

float MPU6050::readGyroscopeX() const
{
    return this->from_raw_gyro(readRawGyroscopeX());
}

float MPU6050::readGyroscopeY() const
{
    return this->from_raw_gyro(readRawGyroscopeY());
}

float MPU6050::readGyroscopeZ() const
{
    return this->from_raw_gyro(readRawGyroscopeZ());
}

float MPU6050::readAccelerometerX() const
{
    return this->from_raw_accel(readRawAccelerometerX());
}

float MPU6050::readAccelerometerY() const
{
    return this->from_raw_accel(readRawAccelerometerY());
}
    
float MPU6050::readAccelerometerZ() const
{
    return this->from_raw_accel(readRawAccelerometerZ());
}

Vec3Di MPU6050::readRawGyroscope() const
{
    return Vec3Di(this->readRawGyroscopeX(),
    this->readRawGyroscopeY(),
    this->readRawGyroscopeZ());
}

Vec3Di MPU6050::readRawAccelerometer() const
{
    return Vec3Di(this->readRawAccelerometerX(),
    this->readRawAccelerometerY(),
    this->readRawAccelerometerZ());
}

Vec3Df MPU6050::readGyroscope() const
{
    return Vec3Df(this->readGyroscopeX(),
    this->readGyroscopeY(),
    this->readGyroscopeZ());
}

Vec3Df MPU6050::readAccelerometer() const
{
    return Vec3Df(this->readAccelerometerX(),
    this->readAccelerometerY(),
    this->readAccelerometerZ());
}


bool MPU6050::setGyroOffsets(const Vec3Di& offsets) const
{
    return this->setGyroOffsetX(offsets.x)
        &this->setGyroOffsetY(offsets.y)
        &this->setGyroOffsetZ(offsets.z);
}

bool MPU6050::setAccelOffsets(const Vec3Di& offsets) const
{
    return this->setAccelOffsetX(offsets.x)
        &this->setAccelOffsetY(offsets.y)
        &this->setAccelOffsetZ(offsets.z);
}

bool MPU6050::setGyroOffsetX(uint16_t offset) const
{

    return i2c_write16(this->i2c_port,this->i2c_addres,MPU6050_RA_XG_OFFS_USRH,offset);

}

bool MPU6050::setGyroOffsetY(uint16_t offset) const
{

    return i2c_write16(this->i2c_port,this->i2c_addres,MPU6050_RA_YG_OFFS_USRH,offset);

}

bool MPU6050::setGyroOffsetZ(uint16_t offset) const
{

    return i2c_write16(this->i2c_port,this->i2c_addres,MPU6050_RA_ZG_OFFS_USRH,offset);

}

bool MPU6050::setAccelOffsetX(uint16_t offset) const
{
    return i2c_write16(this->i2c_port,this->i2c_addres,MPU6050_RA_XA_OFFS_H,offset);
}

bool MPU6050::setAccelOffsetY(uint16_t offset) const
{
    return i2c_write16(this->i2c_port,this->i2c_addres,MPU6050_RA_YA_OFFS_H,offset);
}

bool MPU6050::setAccelOffsetZ(uint16_t offset) const
{
    return i2c_write16(this->i2c_port,this->i2c_addres,MPU6050_RA_ZA_OFFS_H,offset);
}