#include "cmath"
#include "cstring"

extern "C"
{

#include "esp_timer.h"

}

#include "esp_log.h"
#include "dmp_memory.h"
#include "mpu6050_regs.h"
#include "DMP6050.hpp"

bool DMP6050::setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank)
{
    bank &= 0x1F;
    if(userBank)
    {
        bank|=0x20;
    }
    if(prefetchEnabled)
    {
        bank|=0x40;
    }

    return i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_RA_BANK_SEL,bank);

}

bool DMP6050::setMemoryStartAddress(uint8_t addr)
{

    return i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_RA_MEM_START_ADDR,addr);

}

uint8_t DMP6050::readMemoryByte()
{
    i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_RA_MEM_R_W,buffer);

    return buffer[0];
}

bool DMP6050::writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address)
{
    if(!this->setMemoryBank(bank))   
    {
        return false;
    }
    if(!this->setMemoryStartAddress(address))
    {
        return false;
    }

    uint8_t chunkSize;

    uint16_t i=0;

    while(i<dataSize)
    {
        chunkSize=MPU6050_DMP_MEMORY_CHUNK_SIZE;

        if(i+chunkSize > dataSize)
        {
            chunkSize=dataSize - i;
        }

        if (chunkSize > 256 - address) 
        {
            chunkSize = 256 - address;
        }

        if(!i2c_writeBytes(this->i2c_port,this->i2c_addres,MPU6050_RA_MEM_R_W,data+i,chunkSize))
        {
            return false;
        }

        i+=chunkSize;
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (address == 0) {bank++;}
        setMemoryBank(bank);
        setMemoryStartAddress(address);
        
    }

    return true;
}


bool DMP6050::setDMPConfig1(uint8_t config)
{
    return i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_RA_DMP_CFG_1,config);
}

bool DMP6050::setDMPConfig2(uint8_t config)
{
    return i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_RA_DMP_CFG_2,config);
}

bool DMP6050::setOTPBankValid(bool en)
{
    bool res=true;

    uint8_t reg=0x00;

    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_RA_XG_OFFS_TC,&reg);

    if(en)
    {
        reg|=1<<0;
    }
    else
    {
        reg&=~(1<<0);
    }

    res&=i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_RA_XG_OFFS_TC,reg);

    return res;

}

bool DMP6050::getOTPBankValid()
{
    bool res=true;

    uint8_t reg=0x00;

    res=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_RA_XG_OFFS_TC,&reg);

    return res & (reg&(1<<0));
}

bool DMP6050::resetDMP()
{

    bool res=true;

    uint8_t reg=0x00;

    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_USER_CTRL,&reg);

    reg|=1<<3;

    res&=i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_USER_CTRL,reg);

    return res;

}

bool DMP6050::DMPEnable()
{

    bool res=true;

    uint8_t reg=0x00;

    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_USER_CTRL,&reg);

    reg|=1<<7;

    res&=i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_USER_CTRL,reg);

    return res;

}

bool DMP6050::DMPDisable()
{

    bool res=true;

    uint8_t reg=0x00;

    res&=i2c_read8(this->i2c_port,this->i2c_addres,MPU6050_USER_CTRL,&reg);

    reg&=~(1<<7);

    res&=i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_USER_CTRL,reg);

    return res;

}

DMP6050::DMP6050(i2c_port_t i2c_port,uint8_t i2c_address)
: MPU6050(i2c_port,i2c_address)
{
    memset(buffer,0,sizeof(uint8_t)*14);
}

bool DMP6050::DMPinit()
{
    // reset device
    if(!this->reset())
    {
        ESP_LOGE("Sensors","Cannot reset device!");
        return false;
    }

    vTaskDelay(50/portTICK_PERIOD_MS);

    if(!this->Wakeup())
    {
        ESP_LOGE("Sensors","Cannot wake up DMP");
        return false;
    }

    if(!this->setMemoryBank(0x10,true,true))
    {
        ESP_LOGE("Sensors","Cannot set memory bank 0x10");
        return false;
    }

    if(!this->setMemoryStartAddress(0x06))
    {
        ESP_LOGE("Sensors","Cannot set start address 0x06");
        return false;
    }

    ESP_LOGI("Sensors","Revison: %x ",readMemoryByte());

    if(!this->setMemoryBank(0x00))
    {
        ESP_LOGE("Sensors","Cannot set memory bank 0x00");
        return false;
    }

    if(!this->getOTPBankValid())
    {
        ESP_LOGE("Sensors","OTP bank invalid");
        //return false;
        // it was in Arduino library but it wasn't critical
    }

    if(!this->setSlaveAddress(0,0x7F))
    {
        ESP_LOGE("Sensors","Cannot set slave address");
        return false;
    }

    if(!this->I2CMasterDisable())
    {
        ESP_LOGE("Sensors","Cannot disable i2c master");
        return false;
    }

    if(!this->setSlaveAddress(0,MPU6050_ADDRESS))
    {
        ESP_LOGE("Sensors","Cannot set slave address to itself");
        return false;
    }

    if(!this->I2CMasterReset())
    {
        ESP_LOGE("Sensors","Cannot reset i2c master");
        return false;
    }

    vTaskDelay(80/portTICK_PERIOD_MS);

    if(!this->setClockSource(MPU6050::CLOCK_SOURCE::PLL_GYRO_Z_REF))
    {
        ESP_LOGE("Sensors","Cannot set clock source to gyro pll z");
        return false;
    }

    // interrupt int enable, fifo and dmp interrupts
    if(!i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_INT_ENABLE,(1<<1)|(1<<4)))
    {
        ESP_LOGE("Sensors","Cannot enable interrupt for fifo and dmp");
        return false;
    }

    if(!this->setSampleRateDivider(4))
    {
        ESP_LOGE("Sensors","Cannot set sample rate divider to 5");
        return false;
    }

    if(!this->setFrameSyncSource(MPU6050::FSYNC_SIGNAL_SRC::TEMP_OUT_L))
    {
        ESP_LOGE("Sensors","Cannot set frame sync source");
        return false;
    }

    if(!this->setDLPFMode(MPU6050::DPLF_MODE::_44))
    {
        ESP_LOGE("Sensors","Cannot set low pass filter to 44Hz");
        return false;
    }

    if(!this->setGyroscopeRange(MPU6050::GyroscopeRange::G2000))
    {
        ESP_LOGE("Sensors","Cannot set gyroscope range");
        return false;
    }

    if(!this->writeMemoryBlock(dmpMemory,MPU6050_DMP_CODE_SIZE,0x00,0x00))
    {
        ESP_LOGE("Sensors","Cannot write DMP frimware");
        return false;
    }

    const uint8_t dmpUpdate[] = {0x00, MPU6050_DMP_FIFO_RATE_DIVISOR};
	if(!this->writeMemoryBlock(dmpUpdate, 2, 0x02, 0x16))
    {
        ESP_LOGE("Sensors","Cannot write dmpUpdate");
        return false;
    }

    if(!this->setDMPConfig1(0x03))
    {
        ESP_LOGE("Sensors","Cannot write to dmp config 1");
        return false;
    }

    if(!this->setDMPConfig2(0x00))
    {
        ESP_LOGE("Sensors","Cannot write to dmp config 2");
        return false;
    }

    if(!this->setOTPBankValid(false))
    {
        ESP_LOGE("Sensors","Cannot set otp bank to invalid");
        return false;
    }

    if(!this->setMotionDetectionThreshold(2))
    {
        ESP_LOGE("Sensors","Cannot set motion detection threshold");
        return false;
    }

    if(!this->setZeroMotionDetectionThreshold(156))
    {
        ESP_LOGE("Sensors","Cannot set zero motion detection threshold");
        return false;
    }

    if(!this->setMotionDetectionDuration(80))
    {
        ESP_LOGE("Sensors","Cannot set motion detection duration");
        return false;
    }

    if(!this->setZeroMotionDetectionDuration(0))
    {
        ESP_LOGE("Sensors","Cannot set zero motion detection duration");
        return false;
    }

    if(!this->FIFOEnable())
    {
        ESP_LOGE("Sensors","Cannot enable fifo");
        return false;
    }

    if(!this->resetDMP())
    {
        ESP_LOGE("Sensors","Cannot reset DMP");
        return false;
    }

    if(!this->DMPDisable())
    {
        ESP_LOGE("Sensors","Cannot disable DMP");
        return false;   
    }

    // reset fifo
    if(!this->FIFOReset())
    {
        ESP_LOGE("Sensors","Cannot reset fifo");
        return false;
    }

    uint8_t _int=0x00;

    this->interrupt_status(&_int);

    return true;
}

bool DMP6050::start()
{
    return this->DMPEnable();
}

bool DMP6050::stop()
{
    return this->DMPDisable();
}

bool DMP6050::setMotionDetectionThreshold(uint8_t threshold)
{
    return i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_RA_MOT_THR,threshold);
}

bool DMP6050::setZeroMotionDetectionThreshold(uint8_t threshold)
{
    return i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_RA_ZRMOT_THR,threshold);
}

bool DMP6050::setMotionDetectionDuration(uint8_t duration)
{
    return i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_RA_MOT_DUR,duration);
}

bool DMP6050::setZeroMotionDetectionDuration(uint8_t duration)
{
    return i2c_write8(this->i2c_port,this->i2c_addres,MPU6050_RA_ZRMOT_DUR,duration);
}

bool DMP6050::FIFOPacketReady() const
{
    return this->FIFOCount() >=MPU6050_DMP_FIFO_PACKET_SIZE;
}

uint16_t DMP6050::readFIFOPacket(uint8_t *buffer) const
{
    int16_t fifoC;
    int16_t length=MPU6050_DMP_FIFO_PACKET_SIZE;
     // This section of code is for when we allowed more than 1 packet to be acquired
     uint32_t BreakTimer = esp_timer_get_time();
     bool packetReceived = false;
     do {
         if ((fifoC = this->FIFOCount())  > length) {

             if (fifoC > 200) { // if you waited to get the FIFO buffer to > 200 bytes it will take longer to get the last packet in the FIFO Buffer than it will take to  reset the buffer and wait for the next to arrive
                 this->FIFOReset(); //, resetFIFO Fixes any overflow corruption
                 fifoC = 0;
                 while (!(fifoC = this->FIFOCount()) && ((esp_timer_get_time() - BreakTimer) <= 1000000)); // Get Next New Packet
                 } else { //We have more than 1 packet but less than 200 bytes of data in the FIFO Buffer
                 uint8_t Trash[32];
                 while ((fifoC = this->FIFOCount()) > length) {  // Test each time just in case the MPU is writing to the FIFO Buffer
                     fifoC = fifoC - length; // Save the last packet
                     uint16_t  RemoveBytes;
                     while (fifoC) { // fifo count will reach zero so this is safe
                         RemoveBytes = (fifoC < 32) ? fifoC : 32; // Buffer Length is different than the packet length this will efficiently clear the buffer
                         this->readFIFOBuffer(Trash, (uint8_t)RemoveBytes);
                         fifoC -= RemoveBytes;
                     }
                 }
             }
         }
         if (!fifoC) {return 0;} // Called too early no data or we timed out after FIFO Reset
         // We have 1 packet
         packetReceived = fifoC == length;
         if (!packetReceived && (esp_timer_get_time() - BreakTimer) > 1000000) {return 0;}
     } while (!packetReceived);
     this->readFIFOBuffer(buffer, length); //Get 1 packet
     return 1;
}

void DMP6050::readQuadriatic(uint8_t* buffer,Vec4Df& vec)
{
    int16_t data[4];

    data[0]=((buffer[0]<<8)|buffer[1]);
    data[1]=((buffer[4]<<8)|buffer[5]);
    data[2]=((buffer[8]<<8)|buffer[9]);
    data[3]=((buffer[12]<<8)|buffer[13]);

    vec.w=static_cast<float>(data[0])/(16384.0f);
    vec.x=static_cast<float>(data[1])/(16384.0f);
    vec.y=static_cast<float>(data[2])/(16384.0f);
    vec.z=static_cast<float>(data[3])/(16384.0f);

}

void DMP6050::readAngels(const Vec4Df& q,Vec3Df& vec)
{
    vec.x=atan2(2*q.x*q.y - 2*q.w*q.z, 2*q.w*q.w + 2*q.x*q.x - 1);
    vec.y=-asin(2*q.x*q.z + 2*q.w*q.y);
    vec.z=atan2(2*q.y*q.z - 2*q.w*q.x, 2*q.w*q.w + 2*q.z*q.z - 1);
}


void DMP6050::getGravity(const Vec4Df& q,Vec3Df& vec)
{
    vec.x=2*(q.x*q.z - q.w*q.y);
    vec.y=2*(q.w*q.x + q.y*q.z);
    vec.z= q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
}

void DMP6050::getYawPitchRoll(const Vec4Df& q,const Vec3Df& gravity,Vec3Df& vec)
{
    //yaw ( z axis )
    vec.x=atan2(2*q.x*q.y - 2*q.w*q.z, 2*q.w*q.w + 2*q.x*q.x -1);
    //pitch ( y axis )
    vec.y=atan2(gravity.x,sqrt(gravity.y*gravity.y + gravity.z*gravity.z));
    //roll ( x axis )
    vec.z=atan2(gravity.y,gravity.z);

    if(gravity.z<0)
    {
        if(vec.y>0)
        {
            vec.y=M_PI - vec.y;
        }
        else
        {
            vec.y=-M_PI - vec.y;
        }
    }
}

void DMP6050::getAcceleration(uint8_t* buffer,Vec3Di& aa)
{
    aa.x=(buffer[28]<<8)|(buffer[29]);
    aa.y=(buffer[32]<<8)|(buffer[33]);
    aa.z=(buffer[36]<<8)|(buffer[37]);
}

void DMP6050::getLinearAccel(const Vec3Di& aa,const Vec3Df& gravity,Vec3Df& out)
{
    out.x=aa.x - gravity.x*8192;
    out.y=aa.y - gravity.y*8192;
    out.z=aa.z - gravity.z*8192;
}

void DMP6050::getLinearAccelInWorld(const Vec3Df& aa,const Vec4Df& q,Vec3Df& out)
{
    out=aa;

    out.rotate(q);
}